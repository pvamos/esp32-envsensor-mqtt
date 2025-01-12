#include "mqtt_push.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;

// MQTT broker details
#define MQTT_BROKER_URI "mqtt://YourMQTTbroker"  // Replace with your MQTT broker URI
#define MQTT_TOPIC "test/topic"                // Replace with your topic

// MQTT 5 user properties
static esp_mqtt5_user_property_item_t user_property[] = {
    {"device", "esp32"},
    {"version", "1.0"}
};

// MQTT 5 publish properties
static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1, // Indicate UTF-8 encoded text
    .message_expiry_interval = 60, // Message expiry in seconds
};

void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
        },
        .session = {
            .protocol_ver = MQTT_PROTOCOL_V_5, // Enable MQTT 5
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (!mqtt_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    // Set MQTT 5 user properties
    esp_err_t err = esp_mqtt5_client_set_user_property(
        &publish_property.user_property,
        user_property,
        sizeof(user_property) / sizeof(user_property[0])
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set user properties: %s", esp_err_to_name(err));
        return;
    }

    // Start MQTT client
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MQTT 5 client initialized and started");
}

void mqtt_publish_all(int32_t bme_raw_temp, int32_t bme_raw_pressure, int32_t bme_raw_humidity,
                      float bme_comp_temp, float bme_comp_pressure, float bme_comp_humidity,
                      int16_t tmp_raw_temp, float tmp_comp_temp,
                      int32_t aht_raw_temp, int32_t aht_raw_humidity,
                      float aht_comp_temp, float aht_comp_humidity) {
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"bme280\":{\"raw_temp\":%" PRId32 ",\"raw_pressure\":%" PRId32 ",\"raw_humidity\":%" PRId32
             ",\"comp_temp\":%.2f,\"comp_pressure\":%.4f,\"comp_humidity\":%.3f},"
             "\"tmp117\":{\"raw_temp\":%" PRId16 ",\"comp_temp\":%.4f},"
             "\"aht20\":{\"raw_temp\":%" PRId32 ",\"raw_humidity\":%" PRId32
             ",\"comp_temp\":%.2f,\"comp_humidity\":%.3f}}",
             bme_raw_temp, bme_raw_pressure, bme_raw_humidity,
             bme_comp_temp, bme_comp_pressure, bme_comp_humidity,
             tmp_raw_temp, tmp_comp_temp,
             aht_raw_temp, aht_raw_humidity,
             aht_comp_temp, aht_comp_humidity);

    // Set publish properties
    esp_err_t err = esp_mqtt5_client_set_publish_property(mqtt_client, &publish_property);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set publish properties: %s", esp_err_to_name(err));
        return;
    }

    // Publish the message
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to publish sensor data");
    } else {
        ESP_LOGI(TAG, "Published sensor data to MQTT 5, msg_id=%d", msg_id);
    }
}
