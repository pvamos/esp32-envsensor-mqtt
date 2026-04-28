// components/mqtt_push/mqtt_push.c

#include "mqtt_push.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "pb_encode.h"
#include "envsensor.pb.h"

static const char *TAG = "MQTT_PUSH";

#define MQTT_BROKER_URI "mqtts://mqtt.example.com:8883"
#define MQTT_TOPIC      "envsensor/site/device-001"
#define MQTT_USERNAME   "example-user"
#define MQTT_PASSWORD   "example-password"

// set to false to disable verification (INSECURE)
static const bool MQTT_TLS_VERIFY_CERT = false;

#define MQTT_WAIT_CONNECTED_MS 10000
#define MQTT_WAIT_PUBLISHED_MS 10000

static inline bool f_ok(float v) { return !isnan(v); }

static uint64_t read_mac48_as_u64(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    return ((uint64_t)mac[0] << 40) |
           ((uint64_t)mac[1] << 32) |
           ((uint64_t)mac[2] << 24) |
           ((uint64_t)mac[3] << 16) |
           ((uint64_t)mac[4] <<  8) |
           ((uint64_t)mac[5] <<  0);
}

typedef struct {
    EventGroupHandle_t eg;
    int msg_id_wait;   // publish msg_id we are waiting for
} mqtt_once_ctx_t;

#define BIT_CONNECTED (1 << 0)
#define BIT_PUBLISHED (1 << 1)
#define BIT_ERROR     (1 << 2)

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)base; (void)event_id;

    mqtt_once_ctx_t *ctx = (mqtt_once_ctx_t *)handler_args;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        xEventGroupSetBits(ctx->eg, BIT_CONNECTED);
        break;
    case MQTT_EVENT_PUBLISHED:
        if (ctx->msg_id_wait >= 0 && event->msg_id == ctx->msg_id_wait) {
            xEventGroupSetBits(ctx->eg, BIT_PUBLISHED);
        }
        break;
    case MQTT_EVENT_ERROR:
        xEventGroupSetBits(ctx->eg, BIT_ERROR);
        break;
    default:
        break;
    }
}

/* Single connect+publish attempt using an already prepared payload. */
static esp_err_t mqtt_do_single_publish(const uint8_t *payload,
                                        size_t payload_len,
                                        int *out_msg_id)
{
    mqtt_once_ctx_t ctx = {
        .eg = xEventGroupCreate(),
        .msg_id_wait = -1,
    };
    if (!ctx.eg) {
        ESP_LOGE(TAG, "xEventGroupCreate() failed");
        return ESP_ERR_NO_MEM;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = { .uri = MQTT_BROKER_URI },
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication = { .password = MQTT_PASSWORD },
        },
        .network = {
            .disable_auto_reconnect = true, // important for "one-shot"
        },
    };

    if (MQTT_TLS_VERIFY_CERT) {
        // Normal secure mode: validate chain against ESP-IDF certificate bundle
        mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
        mqtt_cfg.broker.verification.skip_cert_common_name_check = false;
    } else {
        // INSECURE: no CA bundle => TLS with certificate verification disabled
        mqtt_cfg.broker.verification.crt_bundle_attach = NULL;
        mqtt_cfg.broker.verification.skip_cert_common_name_check = true;
        ESP_LOGI(TAG, "TLS certificate verification is DISABLED (INSECURE)!");
    }

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (!client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init() failed");
        vEventGroupDelete(ctx.eg);
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, &ctx);

    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start() failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(client);
        vEventGroupDelete(ctx.eg);
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(
        ctx.eg, BIT_CONNECTED | BIT_ERROR,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(MQTT_WAIT_CONNECTED_MS)
    );

    if (!(bits & BIT_CONNECTED)) {
        ESP_LOGE(TAG, "MQTT connect failed/timeout");
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        vEventGroupDelete(ctx.eg);
        return ESP_FAIL;
    }

    int msg_id = esp_mqtt_client_publish(
        client,
        MQTT_TOPIC,
        (const char *)payload,
        (int)payload_len,
        1, // qos
        0  // retain
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Publish failed (esp_mqtt_client_publish returned %d)", msg_id);
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        vEventGroupDelete(ctx.eg);
        return ESP_FAIL;
    }

    ctx.msg_id_wait = msg_id;

    bits = xEventGroupWaitBits(
        ctx.eg, BIT_PUBLISHED | BIT_ERROR,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(MQTT_WAIT_PUBLISHED_MS)
    );

    // Disconnect every time (required)
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    vEventGroupDelete(ctx.eg);

    if (!(bits & BIT_PUBLISHED)) {
        ESP_LOGE(TAG, "Publish not confirmed (timeout/error)");
        return ESP_FAIL;
    }

    if (out_msg_id) {
        *out_msg_id = msg_id;
    }
    return ESP_OK;
}

esp_err_t mqtt_publish_once(
    int32_t rssi_dbm,
    float esp32_temp,
    float bme280_temp, float bme280_pressure, float bme280_humidity,
    float sht4x_temp, float sht4x_humidity)
{
    // Build protobuf
    envsensor_Reading msg = envsensor_Reading_init_zero;
    msg.mac   = read_mac48_as_u64();
    msg.rssi  = rssi_dbm;
    msg.batt  = 0; // placeholder for now

    // Internal ESP32 temperature
    if (f_ok(esp32_temp)) {
        msg.esp32_t = esp32_temp;
    } else {
        // Leave at default (0.0f) if NAN was passed in
        msg.esp32_t = 0.0f;
    }

    // BME280 (optional)
    if (f_ok(bme280_temp) || f_ok(bme280_pressure) || f_ok(bme280_humidity)) {
        msg.has_bme280 = true;
        if (f_ok(bme280_temp))     msg.bme280.t = bme280_temp;
        if (f_ok(bme280_pressure)) msg.bme280.p = bme280_pressure;
        if (f_ok(bme280_humidity)) msg.bme280.h = bme280_humidity;
    }

    // SHT4x (optional)
    if (f_ok(sht4x_temp) || f_ok(sht4x_humidity)) {
        msg.has_sht4x = true;
        if (f_ok(sht4x_temp))     msg.sht4x.t = sht4x_temp;
        if (f_ok(sht4x_humidity)) msg.sht4x.h = sht4x_humidity;
    }

    uint8_t payload[envsensor_Reading_size];
    pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));
    if (!pb_encode(&stream, envsensor_Reading_fields, &msg)) {
        ESP_LOGE(TAG, "pb_encode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    size_t payload_len = stream.bytes_written;
    esp_err_t err;
    int msg_id = -1;

    // First attempt
    err = mqtt_do_single_publish(payload, payload_len, &msg_id);
    if (err == ESP_OK) {
        ESP_LOGW(TAG,
                 "Published protobuf (%u bytes) to '%s', msg_id=%d",
                 (unsigned)payload_len, MQTT_TOPIC, msg_id);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Initial publish failed: %s; retrying once",
             esp_err_to_name(err));

    // Second attempt (single retry)
    msg_id = -1;
    esp_err_t retry_err = mqtt_do_single_publish(payload, payload_len, &msg_id);
    if (retry_err == ESP_OK) {
        ESP_LOGW(TAG,
                 "Successful retry: Published protobuf (%u bytes) to '%s', msg_id=%d",
                 (unsigned)payload_len, MQTT_TOPIC, msg_id);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Publish failed even after retry: %s",
             esp_err_to_name(retry_err));
    return retry_err;
}

const char *mqtt_push_get_broker_uri(void)
{
    return MQTT_BROKER_URI;
}

const char *mqtt_push_get_topic(void)
{
    return MQTT_TOPIC;
}
