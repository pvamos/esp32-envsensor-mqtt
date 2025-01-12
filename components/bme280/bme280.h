#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// BME280 I2C Address
#define BME280_I2C_ADDR 0x77

// BME280 Registers
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_DATA      0xF7
#define BME280_REG_CALIB     0x88

// BME280 Chip ID
#define BME280_CHIP_ID 0x60

// Function prototypes
/**
 * @brief Initialize the BME280 sensor.
 * 
 * @param bus_handle I2C master bus handle.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read raw data from the BME280 sensor.
 * 
 * @param temperature Pointer to store the raw temperature value.
 * @param pressure Pointer to store the raw pressure value.
 * @param humidity Pointer to store the raw humidity value.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t bme280_read_raw(int32_t *temperature, int32_t *pressure, int32_t *humidity);

/**
 * @brief Calculate compensated temperature, pressure, and humidity values.
 * 
 * @param raw_temp Raw temperature value from the sensor.
 * @param raw_pressure Raw pressure value from the sensor.
 * @param raw_humidity Raw humidity value from the sensor.
 * @param comp_temp Pointer to store the compensated temperature (in °C).
 * @param comp_pressure Pointer to store the compensated pressure (in hPa).
 * @param comp_humidity Pointer to store the compensated humidity (in %).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t bme280_calculate_compensated(
    int32_t raw_temp, int32_t raw_pressure, int32_t raw_humidity,
    float *comp_temp, float *comp_pressure, float *comp_humidity);

#endif // BME280_H
