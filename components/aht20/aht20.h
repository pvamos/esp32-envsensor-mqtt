#ifndef AHT20_H
#define AHT20_H

#include <stdint.h>
#include <esp_err.h>
#include "i2c.h"

#define AHT20_I2C_ADDRESS 0x38
#define AHT20_CMD_INIT 0xBE
#define AHT20_CMD_TRIGGER 0xAC
#define AHT20_CMD_RESET 0xBA

#define AHT20_INIT_DELAY_MS 10
#define AHT20_MEASURE_DELAY_MS 80

/**
 * @brief Initialize the AHT20 sensor.
 * 
 * @param bus_handle The I2C bus handle.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t aht20_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read raw temperature and humidity data from the AHT20 sensor.
 * 
 * @param raw_temperature Pointer to store the raw temperature value.
 * @param raw_humidity Pointer to store the raw humidity value.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t aht20_read_raw(int32_t *raw_temperature, int32_t *raw_humidity);

/**
 * @brief Calculate temperature and humidity in human-readable format.
 * 
 * @param raw_temperature Raw temperature value.
 * @param raw_humidity Raw humidity value.
 * @param temperature Pointer to store the temperature in °C.
 * @param humidity Pointer to store the humidity in %.
 * @return esp_err_t ESP_OK on success or error code on failure.
 */
esp_err_t aht20_calculate(int32_t raw_temperature, int32_t raw_humidity, float *temperature, float *humidity);

#endif // AHT20_H
