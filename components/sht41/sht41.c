#include "sht41.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"
#include <string.h>

static const char *TAG = "SHT41";

static i2c_master_dev_handle_t sht41_dev_handle = NULL;

esp_err_t sht41_init(i2c_master_bus_handle_t bus_handle) {
    // Add SHT41 to the I2C bus
    ESP_ERROR_CHECK(i2c_add_device(bus_handle, SHT41_I2C_ADDR, &sht41_dev_handle));
    ESP_LOGI(TAG, "SHT41 initialized successfully");
    return ESP_OK;
}

esp_err_t sht41_read(float *temperature, float *humidity) {
    if (!temperature || !humidity) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = SHT41_MEASURE_HIGH_PRECISION;
    uint8_t data[6]; // 2 for temperature, 2 for humidity, 2 for CRC
    uint16_t raw_temperature, raw_humidity;

    // Send measurement command
    esp_err_t err = i2c_write(sht41_dev_handle, &cmd, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command to SHT41: %s", esp_err_to_name(err));
        return err;
    }

    // Wait for measurement
    vTaskDelay(pdMS_TO_TICKS(SHT41_MEASURE_DELAY_MS));

    // Read measurement results
    err = i2c_read(sht41_dev_handle, data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from SHT41: %s", esp_err_to_name(err));
        return err;
    }

    raw_temperature = (data[0] << 8) | data[1];
    raw_humidity    = (data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * (raw_temperature / 65535.0f);
    *humidity    = 100.0f * (raw_humidity / 65535.0f);

    ESP_LOGI(TAG, "SHT41: Temperature = %.2f °C, Humidity = %.2f %%", *temperature, *humidity);
    return ESP_OK;
}

esp_err_t sht41_get_serial(uint8_t out[8])
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, 8);  // fill [0..3] = 0, plus default for entire 8 bytes

    // Send command 0x89, read 6 bytes
    uint8_t cmd = SHT41_CMD_READ_SERIAL;
    esp_err_t err = i2c_write(sht41_dev_handle, &cmd, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SHT41 serial command: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(SHT41_SERIAL_DELAY_MS));

    uint8_t data[6];
    err = i2c_read(sht41_dev_handle, data, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SHT41 serial data: %s", esp_err_to_name(err));
        return err;
    }

    // data layout:
    //  [0]=Word1Hi, [1]=Word1Lo, [2]=CRC1,
    //  [3]=Word2Hi, [4]=Word2Lo, [5]=CRC2

    // store these 4 ID bytes in out[4..7]:
    out[4] = data[0];  // Word1Hi
    out[5] = data[1];  // Word1Lo
    out[6] = data[3];  // Word2Hi
    out[7] = data[4];  // Word2Lo

    ESP_LOGI(TAG, "SHT41 serial: 0x%02X%02X %02X%02X (CRCs 0x%02X,0x%02X)",
             data[0], data[1], data[3], data[4], data[2], data[5]);

    return ESP_OK;
}
