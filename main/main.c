#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include "wifi_sta.h"
#include "mqtt_push.h"

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

    // Initialize MQTT
    mqtt_init();

    // Main loop to read sensor data and publish to MQTT
    while (true) {
        int32_t bme_raw_temp = 0, bme_raw_pressure = 0, bme_raw_humidity = 0;
        float bme_comp_temp = 0.0f, bme_comp_pressure = 0.0f, bme_comp_humidity = 0.0f;
        int16_t tmp_raw_temp = 0;
        float tmp_comp_temp = 0.0f;
        int32_t aht_raw_temp = 0, aht_raw_humidity = 0;
        float aht_comp_temp = 0.0f, aht_comp_humidity = 0.0f;

        // Read BME280 sensor values
        if (bme280_read_raw(&bme_raw_temp, &bme_raw_pressure, &bme_raw_humidity) == ESP_OK &&
            bme280_calculate_compensated(bme_raw_temp, bme_raw_pressure, bme_raw_humidity,
                                         &bme_comp_temp, &bme_comp_pressure, &bme_comp_humidity) == ESP_OK) {
            ESP_LOGI(TAG, "BME280 data collected successfully");
        } else {
            ESP_LOGE(TAG, "Failed to read BME280 data");
        }

        // Read TMP117 sensor values
        if (tmp117_read_raw(&tmp_raw_temp) == ESP_OK &&
            tmp117_calculate_compensated(tmp_raw_temp, &tmp_comp_temp) == ESP_OK) {
            ESP_LOGI(TAG, "TMP117 data collected successfully");
        } else {
            ESP_LOGE(TAG, "Failed to read TMP117 data");
        }

        // Read AHT20 sensor values
        if (aht20_read_raw(&aht_raw_temp, &aht_raw_humidity) == ESP_OK &&
            aht20_calculate(aht_raw_temp, aht_raw_humidity, &aht_comp_temp, &aht_comp_humidity) == ESP_OK) {
            ESP_LOGI(TAG, "AHT20 data collected successfully");
        } else {
            ESP_LOGE(TAG, "Failed to read AHT20 data");
        }

        // Publish all sensor data to MQTT
        mqtt_publish_all(bme_raw_temp, bme_raw_pressure, bme_raw_humidity,
                         bme_comp_temp, bme_comp_pressure, bme_comp_humidity,
                         tmp_raw_temp, tmp_comp_temp,
                         aht_raw_temp, aht_raw_humidity,
                         aht_comp_temp, aht_comp_humidity);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
