#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_sta.h"
#include "mqtt_push.h"
#include "math.h"
#include "i2c.h"
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include "sht41.h"

static const char *TAG = "MAIN";

void app_main() {
    ESP_LOGI(TAG, "Starting application...");

    // Initialize Wi-Fi
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed");
        return;
    }

    // Initialize MQTT
    mqtt_init();

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

    if (sht41_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SHT41 sensor");
        return;
    }

    float bme280_temp = NAN, bme280_pressure = NAN, bme280_humidity = NAN;
    float tmp117_temp = NAN;
    float aht20_temp = NAN, aht20_humidity = NAN;
    float sht41_temp = NAN, sht41_humidity = NAN;

    // Main loop to read sensor data and publish to MQTT
    while (true) {
        // Read BME280 sensor values
        if (bme280_read(&bme280_temp, &bme280_pressure, &bme280_humidity) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read BME280 sensor");
            bme280_temp = bme280_pressure = bme280_humidity = NAN;
        }

        // Read TMP117 sensor values
        if (tmp117_read(&tmp117_temp) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read TMP117 sensor");
            tmp117_temp = NAN;
        }

        // Read AHT20 sensor values
        if (aht20_read(&aht20_temp, &aht20_humidity) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read AHT20 sensor");
            aht20_temp = aht20_humidity = NAN;
        }

        // Read SHT41 sensor values
        if (sht41_read(&sht41_temp, &sht41_humidity) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read SHT41 sensor");
            sht41_temp = sht41_humidity = NAN;
        }

        // Publish sensor data
        mqtt_publish(bme280_temp, bme280_pressure, bme280_humidity,
                     tmp117_temp, aht20_temp, aht20_humidity,
                     sht41_temp, sht41_humidity);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

