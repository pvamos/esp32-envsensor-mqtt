#ifndef I2C_H
#define I2C_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// I2C Configuration
#define I2C_MASTER_NUM I2C_NUM_0          // I2C port number
#define I2C_MASTER_SCL_IO 22              // GPIO for SCL
#define I2C_MASTER_SDA_IO 21              // GPIO for SDA
#define I2C_MASTER_FREQ_HZ 100000         // I2C clock frequency (100 kHz)
#define I2C_MASTER_TIMEOUT_MS 500        // I2C operation timeout (ms)

// Initialize the I2C master
esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle);

#endif // I2C_H
