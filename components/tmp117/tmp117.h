#ifndef TMP117_H
#define TMP117_H

#include "esp_err.h"
#include "driver/i2c_master.h"

// TMP117 I2C Address
#define TMP117_I2C_ADDR 0x48 // Default I2C address

// TMP117 Registers
#define TMP117_REG_TEMP_RESULT 0x00
#define TMP117_REG_CONFIG 0x01
#define TMP117_REG_DEVICE_ID 0x0F

// TMP117 Expected Device ID
#define TMP117_DEVICE_ID 0x0117

// Function to initialize the TMP117 sensor
esp_err_t tmp117_init(i2c_master_bus_handle_t bus_handle);

// Function to read raw temperature value from TMP117
esp_err_t tmp117_read_raw(int16_t *raw_temperature);

// Function to calculate the compensated temperature value
esp_err_t tmp117_calculate_compensated(int16_t raw_temperature, float *compensated_temperature);

#endif // TMP117_H
