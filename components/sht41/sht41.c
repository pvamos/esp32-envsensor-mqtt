#include "sht41.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"

static const char *TAG = "SHT41";

static i2c_master_dev_handle_t sht41_dev_handle = NULL;

// Initialize the SHT41 sensor
esp_err_t sht41_init(i2c_master_bus_handle_t bus_handle) {
    // Add SHT41 to the I2C bus
    ESP_ERROR_CHECK(i2c_add_device(bus_handle, SHT41_I2C_ADDR, &sht41_dev_handle));
    ESP_LOGI(TAG, "SHT41 initialized successfully");
    return ESP_OK;
}

// Read temperature and humidity data from the SHT41 sensor
esp_err_t sht41_read(float *temperature, float *humidity) {
    if (!temperature || !humidity) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = SHT41_MEASURE_HIGH_PRECISION;
    uint8_t data[6]; // 6 bytes: 2 for temperature, 2 for humidity, 2 for CRC
    uint16_t raw_temperature, raw_humidity;

    // Send measurement command
    esp_err_t err = i2c_write(sht41_dev_handle, &cmd, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command to SHT41: %s", esp_err_to_name(err));
        return err;
    }

    // Wait for the measurement to complete
    vTaskDelay(pdMS_TO_TICKS(SHT41_MEASURE_DELAY_MS));

    // Read measurement results
    err = i2c_read(sht41_dev_handle, data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from SHT41: %s", esp_err_to_name(err));
        return err;
    }

    // Process raw data
    raw_temperature = (data[0] << 8) | data[1];
    raw_humidity = (data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * (raw_temperature / 65535.0f);
    *humidity = 100.0f * (raw_humidity / 65535.0f);

    ESP_LOGI(TAG, "SHT41: Temperature = %.2f °C, Humidity = %.2f %%", *temperature, *humidity);
    return ESP_OK;
}

