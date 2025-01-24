#ifndef I2C_H
#define I2C_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// I2C Configuration
#define I2C_MASTER_NUM I2C_NUM_0          // I2C port number
#define I2C_MASTER_SCL_IO 22              // GPIO for SCL
#define I2C_MASTER_SDA_IO 21              // GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000         // I2C clock frequency (100 kHz)
#define I2C_MASTER_TIMEOUT_MS 1000        // I2C operation timeout (ms)

// Initialize the I2C master
esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle);

/**
 * @brief Add a device to the I2C bus.
 *
 * @param bus_handle I2C bus handle.
 * @param dev_addr I2C device address.
 * @param dev_handle Pointer to store the I2C device handle.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Write data to an I2C device.
 *
 * @param dev_handle I2C device handle.
 * @param data Pointer to the data to write.
 * @param data_len Length of the data to write.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t *data, size_t data_len);

/**
 * @brief Read data from an I2C device.
 *
 * @param dev_handle I2C device handle.
 * @param data Pointer to buffer to store the read data.
 * @param data_len Length of data to read.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t data_len);

#endif // I2C_H

