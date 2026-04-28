#include "sht4x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"

#include <stddef.h>
#include <stdint.h>

static const char *TAG = "SHT4X";

static i2c_master_dev_handle_t sht4x_dev_handle = NULL;

/* SHT4x CRC-8 calculation
 *
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Init value: 0xFF
 * This matches Sensirion's datasheet reference implementation.
 */
static uint8_t sht4x_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Initialize the SHT4X sensor
esp_err_t sht4x_init(i2c_master_bus_handle_t bus_handle)
{
    // Add SHT4X to the I2C bus
    esp_err_t err = i2c_add_device(bus_handle, SHT4X_I2C_ADDR, &sht4x_dev_handle);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "SHT4X initialized successfully");
    return ESP_OK;
}

// Read temperature and humidity data from the SHT4X sensor
esp_err_t sht4x_read(float *temperature, float *humidity)
{
    if (!temperature || !humidity) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }
    if (!sht4x_dev_handle) {
        ESP_LOGE(TAG, "SHT4X not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = SHT4X_MEASURE_HIGH_PRECISION;
    uint8_t data[6]; // [0..1]=T, [2]=T CRC, [3..4]=RH, [5]=RH CRC
    uint16_t raw_temperature, raw_humidity;

    // Send measurement command
    esp_err_t err = i2c_write(sht4x_dev_handle, &cmd, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command to SHT4X: %s",
                 esp_err_to_name(err));
        return err;
    }

    // Wait for the measurement to complete
    vTaskDelay(pdMS_TO_TICKS(SHT4X_MEASURE_DELAY_MS));

    // Read measurement results
    err = i2c_read(sht4x_dev_handle, data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from SHT4X: %s",
                 esp_err_to_name(err));
        return err;
    }

    // CRC check for temperature and humidity
    uint8_t crc_t  = sht4x_crc8(&data[0], 2);
    uint8_t crc_rh = sht4x_crc8(&data[3], 2);

    if (crc_t != data[2] || crc_rh != data[5]) {
        ESP_LOGE(TAG,
                 "SHT4X CRC error: T_crc_calc=0x%02X, T_crc_rx=0x%02X; "
                 "RH_crc_calc=0x%02X, RH_crc_rx=0x%02X",
                 crc_t, data[2], crc_rh, data[5]);
        // Caller will log and treat readings as NAN on ESP_FAIL
        return ESP_FAIL;
    }

    // Process raw data (CRC already validated)
    raw_temperature = (uint16_t)((data[0] << 8) | data[1]);
    raw_humidity    = (uint16_t)((data[3] << 8) | data[4]);

    *temperature = -45.0f + 175.0f * (raw_temperature / 65535.0f);
    *humidity    = 100.0f * (raw_humidity / 65535.0f);

    ESP_LOGI(TAG, "SHT4X: Temperature = %.2f °C, Humidity = %.2f %%",
             *temperature, *humidity);

    return ESP_OK;
}
