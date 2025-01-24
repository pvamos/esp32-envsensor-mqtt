#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "aht20.h"
#include "i2c.h"
#include "esp_log.h"

static const char *TAG = "AHT20";

static i2c_master_dev_handle_t aht20_dev_handle = NULL;

// Initialize AHT20 sensor
esp_err_t aht20_init(i2c_master_bus_handle_t bus_handle) {
    // Add AHT20 to the I2C bus
    ESP_ERROR_CHECK(i2c_add_device(bus_handle, AHT20_I2C_ADDR, &aht20_dev_handle));

    // Send initialization command
    uint8_t init_cmd[] = { AHT20_CMD_INIT, 0x08, 0x00 };
    esp_err_t err = i2c_write(aht20_dev_handle, init_cmd, sizeof(init_cmd));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initialization command: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(AHT20_INIT_DELAY_MS));
    ESP_LOGI(TAG, "AHT20 initialized successfully");

    return ESP_OK;
}

// Read temperature and humidity data from AHT20
esp_err_t aht20_read(float *temperature, float *humidity) {
    if (!temperature || !humidity) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t trigger_cmd[] = { AHT20_CMD_TRIGGER, 0x33, 0x00 };
    uint8_t data[6];

    // Send measurement trigger command
    esp_err_t err = i2c_write(aht20_dev_handle, trigger_cmd, sizeof(trigger_cmd));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement trigger command: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(AHT20_MEASURE_DELAY_MS));

    // Read sensor data
    err = i2c_read(aht20_dev_handle, data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from AHT20: %s", esp_err_to_name(err));
        return err;
    }

    if ((data[0] & 0x80) != 0) { // Check if the sensor is busy
        ESP_LOGE(TAG, "AHT20 sensor data not ready");
        return ESP_FAIL;
    }

    // Parse raw humidity (20 bits)
    int32_t raw_humidity = ((int32_t)data[1] << 12) | ((int32_t)data[2] << 4) | ((int32_t)data[3] >> 4);
    *humidity = ((float)raw_humidity / 1048576.0f) * 100.0f;

    // Parse raw temperature (20 bits)
    int32_t raw_temperature = (((int32_t)data[3] & 0x0F) << 16) | ((int32_t)data[4] << 8) | (int32_t)data[5];
    *temperature = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f;

    ESP_LOGI(TAG, "AHT20: Temperature = %.2f °C, Humidity = %.3f %%RH", *temperature, *humidity);

    return ESP_OK;
}

