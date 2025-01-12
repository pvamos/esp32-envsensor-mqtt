#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmp117.h"
#include "i2c.h"
#include "esp_log.h"
#include <inttypes.h> // For PRI macros

static const char *TAG = "TMP117";

static i2c_master_dev_handle_t tmp117_dev_handle = NULL;

// Initialize TMP117 sensor
esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle) {
    // Add TMP117 to the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &(i2c_device_config_t) {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP117_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    }, &tmp117_dev_handle));

    // Configure TMP117 for continuous conversion
    uint8_t config[3] = {TMP117_REG_CONFIG, 0x02, 0x00}; // Continuous conversion mode
    ESP_ERROR_CHECK(i2c_master_transmit(tmp117_dev_handle, config, sizeof(config), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    ESP_LOGI(TAG, "TMP117 initialized successfully");
    
    return ESP_OK;
}

// Read raw temperature from TMP117
esp_err_t tmp117_read_raw(int16_t *raw_temperature) {
    uint8_t reg = TMP117_REG_TEMP_RESULT;
    uint8_t temp_data[2];

    ESP_ERROR_CHECK(i2c_master_transmit(tmp117_dev_handle, &reg, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_receive(tmp117_dev_handle, temp_data, sizeof(temp_data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    *raw_temperature = (int16_t)((temp_data[0] << 8) | temp_data[1]);

    ESP_LOGI(TAG, "TMP117 RAW Temperature: %" PRId16, *raw_temperature);

    return ESP_OK;
}

// Convert raw temperature to Celsius
esp_err_t tmp117_calculate_compensated(int16_t raw_temperature, float *compensated_temperature) {
    *compensated_temperature = raw_temperature * 0.0078125f; // TMP117 resolution is 0.0078125°C per LSB
    ESP_LOGI(TAG, "TMP117 Temperature: %.4f °C", *compensated_temperature);

    return ESP_OK;
}
