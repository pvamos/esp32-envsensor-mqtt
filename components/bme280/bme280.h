#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "i2c.h"

// BME280 I2C Address
#define BME280_I2C_ADDR 0x77

// BME280 Registers
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_DATA      0xF7

// BME280 Chip ID
#define BME280_CHIP_ID 0x60

/**
 * @brief Initialize the BME280 sensor.
 *
 * @param bus_handle I2C master bus handle.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read and compensate temperature, pressure, and humidity from the BME280 sensor.
 *
 * @param temperature Pointer to store the temperature (in °C).
 * @param pressure Pointer to store the pressure (in hPa).
 * @param humidity Pointer to store the humidity (in %RH).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t bme280_read(float *temperature, float *pressure, float *humidity);

#endif // BME280_H

