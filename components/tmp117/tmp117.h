#ifndef TMP117_H
#define TMP117_H

#include "esp_err.h"
#include "i2c.h"

// TMP117 I2C Address
#define TMP117_I2C_ADDR 0x48 // Default I2C address

// TMP117 Registers
#define TMP117_REG_TEMP_RESULT 0x00
#define TMP117_REG_CONFIG      0x01
#define TMP117_REG_DEVICE_ID   0x0F

// TMP117 Expected Device ID
#define TMP117_DEVICE_ID 0x0117

// TMP117 resolution in °C/LSB
#define TMP117_RESOLUTION 0.0078125f

/**
 * @brief Initialize the TMP117 sensor.
 *
 * @param bus_handle I2C master bus handle.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read the temperature from TMP117 sensor.
 *
 * @param temperature Pointer to store the temperature in °C.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t tmp117_read(float *temperature);

#endif // TMP117_H

