#include "mqtt_push.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_PUSH";
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Example topic for binary frames
#define MQTT_BINARY_TOPIC "test/binary"

/**
 * @brief Initialize the MQTT client and connect to broker
 */
void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        // Adjust these to suit your broker
        .broker = {
            .address = {
                .uri = "mqtt://192.168.1.83:1883", // your broker
            },
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MQTT client initialized and started");
}

/**
 * @brief Publish a raw binary frame to the configured MQTT broker.
 *
 * @param frame     pointer to binary data
 * @param frame_len number of bytes
 */
void mqtt_publish_binary(const uint8_t *frame, size_t frame_len) {
    if (!mqtt_client) {
        ESP_LOGE(TAG, "MQTT client not initialized, call mqtt_init() first");
        return;
    }
    if (!frame || frame_len == 0) {
        ESP_LOGW(TAG, "mqtt_publish_binary: invalid data");
        return;
    }

    // Publish raw bytes to a topic. QOS=1, no retain
    int msg_id = esp_mqtt_client_publish(mqtt_client,
                                         MQTT_BINARY_TOPIC,
                                         (const char *)frame,
                                         frame_len,
                                         1 /* qos */,
                                         0 /* retain */);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to publish binary data");
    } else {
        ESP_LOGI(TAG, "Published %d bytes of binary data to topic '%s' (msg_id=%d)",
                 (int)frame_len, MQTT_BINARY_TOPIC, msg_id);
    }
}
