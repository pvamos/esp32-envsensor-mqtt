#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include "wifi_sta.h"
#include "mqtt_push.h"
#include "sht41.h"

static const char *TAG = "MAIN";

void app_main() {
    ESP_LOGI(TAG, "Starting application...");

    // Initialize Wi-Fi
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed");
        return;
    }

    // Initialize I2C bus
    i2c_master_bus_handle_t i2c_bus_handle;
    if (i2c_master_init(&i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return;
    }

    // Initialize sensors
    if (bme280_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280 sensor");
        return;
    }

    if (tmp117_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TMP117 sensor");
        return;
    }

    if (aht20_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AHT20 sensor");
        return;
    }

    // Initialize SHT41 sensor
    if (sht41_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, 400000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SHT41 sensor");
        return;
    }

    // Initialize MQTT
    mqtt_init();

    // Main loop to read sensor data and publish to MQTT
    while (true) {
        float bme_temp, bme_pressure, bme_humidity;
        float tmp_temp;
        float aht_temp, aht_humidity;
        float sht41_temp, sht41_humidity;

        // Read BME280 sensor values
        if (bme280_read_float(&bme_temp, &bme_pressure, &bme_humidity) == ESP_OK) {
            mqtt_publish_float("bme280/temperature", bme_temp);
            mqtt_publish_float("bme280/pressure", bme_pressure);
            mqtt_publish_float("bme280/humidity", bme_humidity);
        }

        // Read TMP117 sensor values
        if (tmp117_read(&tmp_temp) == ESP_OK) {
            mqtt_publish_float("tmp117/temperature", tmp_temp);
        }

        // Read AHT20 sensor values
        if (aht20_read(&aht_temp, &aht_humidity) == ESP_OK) {
            mqtt_publish_float("aht20/temperature", aht_temp);
            mqtt_publish_float("aht20/humidity", aht_humidity);
        }

        // Read SHT41 sensor values
        if (sht41_read(&sht41_temp, &sht41_humidity) == ESP_OK) {
            mqtt_publish_float("sht41/temperature", sht41_temp);
            mqtt_publish_float("sht41/humidity", sht41_humidity);
        }

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

