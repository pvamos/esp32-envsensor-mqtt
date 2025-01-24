#include "mqtt_push.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_PUSH";
static esp_mqtt_client_handle_t mqtt_client;

// MQTT broker details
#define MQTT_BROKER_URI "mqtt://YourMQTTbroker"  // Replace with your MQTT broker URI
#define MQTT_TOPIC "sensor/data"                // Replace with your desired topic

void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
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

void mqtt_publish(float bme280_temp, float bme280_pressure, float bme280_humidity,
                  float tmp117_temp, float aht20_temp, float aht20_humidity,
                  float sht41_temp, float sht41_humidity) {
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"bme280\":{\"temperature\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f},"
             "\"tmp117\":{\"temperature\":%.2f},"
             "\"aht20\":{\"temperature\":%.2f,\"humidity\":%.2f},"
             "\"sht41\":{\"temperature\":%.2f,\"humidity\":%.2f}}",
             bme280_temp, bme280_pressure, bme280_humidity,
             tmp117_temp,
             aht20_temp, aht20_humidity,
             sht41_temp, sht41_humidity);

    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to publish sensor data");
    } else {
        ESP_LOGI(TAG, "Published sensor data to MQTT topic '%s', msg_id=%d", MQTT_TOPIC, msg_id);
    }
}

