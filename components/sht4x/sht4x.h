#ifndef SHT4X_H
#define SHT4X_H

#include <esp_err.h>
#include "i2c.h"

// SHT4X I2C Address
#define SHT4X_I2C_ADDR 0x44

// SHT4X Commands
#define SHT4X_MEASURE_HIGH_PRECISION 0xFD
#define SHT4X_MEASURE_DELAY_MS 20

/**
 * @brief Initialize the SHT41 sensor.
 *
 * @param bus_handle The I2C bus handle.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t sht4x_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read temperature and humidity from the SHT41 sensor.
 *
 * @param temperature Pointer to store temperature in degrees Celsius.
 * @param humidity Pointer to store humidity in percentage.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t sht4x_read(float *temperature, float *humidity);

#endif // SHT4X_H

