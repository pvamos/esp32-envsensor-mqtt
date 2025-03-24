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

// Additional EEPROM registers (scratch / ID):
//  - EEPROM1 = 0x05
//  - EEPROM2 = 0x06
//  - EEPROM3 = 0x08

#define TMP117_REG_EEPROM1    0x05
#define TMP117_REG_EEPROM2    0x06
#define TMP117_REG_EEPROM3    0x08

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

/**
 * @brief Read the EEPROM1, EEPROM2, EEPROM3 registers (each 2 bytes),
 *        and store them (left-padded with 2 bytes of 0) into an 8-byte array:
 *
 *        out[0..1] = 0
 *        out[2..3] = EEPROM1
 *        out[4..5] = EEPROM2
 *        out[6..7] = EEPROM3
 *
 * This yields 8 total bytes, with the 6 bytes from the 3 EEPROM registers
 * in the final positions. The first 2 bytes are zero.
 *
 * @param out 8-byte buffer to store the ID
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tmp117_get_serial(uint8_t out[8]);

#endif // TMP117_H
