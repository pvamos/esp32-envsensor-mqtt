#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmp117.h"
#include "i2c.h"
#include "esp_log.h"

static const char *TAG = "TMP117";

static i2c_master_dev_handle_t tmp117_dev_handle = NULL;

// Initialize TMP117 sensor
esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle) {
    // Add TMP117 to the I2C bus
    ESP_ERROR_CHECK(i2c_add_device(bus_handle, TMP117_I2C_ADDR, &tmp117_dev_handle));

    // Configure TMP117 for continuous conversion
    uint8_t config[3] = {TMP117_REG_CONFIG, 0x02, 0x00}; // Continuous conversion mode
    ESP_ERROR_CHECK(i2c_write(tmp117_dev_handle, config, sizeof(config)));

    ESP_LOGI(TAG, "TMP117 initialized successfully");

    return ESP_OK;
}

// Read temperature from TMP117
esp_err_t tmp117_read(float *temperature) {
    uint8_t reg = TMP117_REG_TEMP_RESULT;
    uint8_t temp_data[2];

    ESP_ERROR_CHECK(i2c_write(tmp117_dev_handle, &reg, 1));
    ESP_ERROR_CHECK(i2c_read(tmp117_dev_handle, temp_data, sizeof(temp_data)));

    int16_t raw_temperature = (int16_t)((temp_data[0] << 8) | temp_data[1]);
    *temperature = raw_temperature * TMP117_RESOLUTION; // TMP117 resolution is 0.0078125°C per LSB

    ESP_LOGI(TAG, "TMP117 Temperature: %.4f °C", *temperature);

    return ESP_OK;
}

