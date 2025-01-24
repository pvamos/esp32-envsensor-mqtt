#ifndef SHT41_H
#define SHT41_H

#include <esp_err.h>
#include <i2cdev.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

/**
 * @brief Initialize the SHT41 sensor on the given I2C bus.
 *
 * @param i2c_port I2C port number (e.g., I2C_NUM_0)
 * @param sda_pin GPIO number for SDA line
 * @param scl_pin GPIO number for SCL line
 * @param freq I2C clock frequency
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t sht41_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq);

/**
 * @brief Read temperature and humidity data from the SHT41 sensor.
 *
 * @param temperature Pointer to store temperature in degrees Celsius
 * @param humidity Pointer to store humidity in percentage
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t sht41_read(float *temperature, float *humidity);

#endif // SHT41_H

