#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmp117.h"
#include "i2c.h"
#include "esp_log.h"
#include <string.h>

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
    if (!temperature) {
        ESP_LOGE(TAG, "Null pointer for temperature");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t reg = TMP117_REG_TEMP_RESULT;
    uint8_t temp_data[2];

    ESP_ERROR_CHECK(i2c_write(tmp117_dev_handle, &reg, 1));
    ESP_ERROR_CHECK(i2c_read(tmp117_dev_handle, temp_data, sizeof(temp_data)));

    int16_t raw_temperature = (int16_t)((temp_data[0] << 8) | temp_data[1]);
    *temperature = raw_temperature * TMP117_RESOLUTION; // 0.0078125°C per LSB

    ESP_LOGI(TAG, "TMP117 Temperature: %.4f °C", *temperature);
    return ESP_OK;
}

/**
 * @brief Read EEPROM1(0x05), EEPROM2(0x06), EEPROM3(0x08).
 * Each is 16 bits => total 6 bytes. We store them in out[2..7],
 * with out[0..1] = 0.
 */
esp_err_t tmp117_get_serial(uint8_t out[8])
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, 8);

    // Helper function to read 2 bytes from a given register
    auto read_eeprom_16 = [&](uint8_t reg) -> uint16_t {
        uint8_t data[2];
        // Write register pointer
        esp_err_t err = i2c_write(tmp117_dev_handle, &reg, 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set TMP117 EEPROM register=0x%02X: %s",
                     reg, esp_err_to_name(err));
            return 0; // fallback
        }
        // Read 2 bytes
        err = i2c_read(tmp117_dev_handle, data, 2);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read 2 bytes from TMP117 reg=0x%02X: %s",
                     reg, esp_err_to_name(err));
            return 0;
        }
        return (uint16_t)((data[0] << 8) | data[1]);
    };

    // read 3 registers
    uint16_t e1 = read_eeprom_16(TMP117_REG_EEPROM1);
    uint16_t e2 = read_eeprom_16(TMP117_REG_EEPROM2);
    uint16_t e3 = read_eeprom_16(TMP117_REG_EEPROM3);

    ESP_LOGI(TAG, "TMP117 EEPROM1=0x%04X, EEPROM2=0x%04X, EEPROM3=0x%04X", e1, e2, e3);

    // place them in out[2..7], leaving out[0..1] = 0
    out[2] = (e1 >> 8) & 0xFF;
    out[3] =  e1       & 0xFF;
    out[4] = (e2 >> 8) & 0xFF;
    out[5] =  e2       & 0xFF;
    out[6] = (e3 >> 8) & 0xFF;
    out[7] =  e3       & 0xFF;

    return ESP_OK;
}
