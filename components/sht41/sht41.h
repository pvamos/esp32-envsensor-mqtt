#ifndef SHT41_H
#define SHT41_H

#include <esp_err.h>
#include "i2c.h"

// SHT41 I2C Address
#define SHT41_I2C_ADDR 0x44

// Measurement command
#define SHT41_MEASURE_HIGH_PRECISION 0xFD
#define SHT41_MEASURE_DELAY_MS 20

// SHT41 serial-number command
#define SHT41_CMD_READ_SERIAL 0x89
#define SHT41_SERIAL_DELAY_MS 1

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

/**
 * @brief Read the unique 4-byte SHT41 serial from OTP memory, returning
 *        it as an 8-byte array with the first 4 bytes = 0 and the last 4
 *        bytes = the sensor’s ID.
 *
 * Data layout from command 0x89 is:
 *   Word1(2 bytes) + CRC1(1 byte), Word2(2 bytes) + CRC2(1 byte) = 6 bytes total
 *
 * We store Word1/Word2 in out[4..7], leaving out[0..3] = 0.
 *
 * @param out 8-byte buffer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sht41_get_serial(uint8_t out[8]);

#endif // SHT41_H
