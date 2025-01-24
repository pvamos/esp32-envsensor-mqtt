#ifndef SHT41_H
#define SHT41_H

#include <esp_err.h>
#include "i2c.h"

// SHT41 I2C Address
#define SHT41_I2C_ADDR 0x44

// SHT41 Commands
#define SHT41_MEASURE_HIGH_PRECISION 0xFD
#define SHT41_MEASURE_DELAY_MS 10

/**
 * @brief Initialize the SHT41 sensor.
 *
 * @param bus_handle The I2C bus handle.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t sht41_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read temperature and humidity from the SHT41 sensor.
 *
 * @param temperature Pointer to store temperature in degrees Celsius.
 * @param humidity Pointer to store humidity in percentage.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t sht41_read(float *temperature, float *humidity);

#endif // SHT41_H

