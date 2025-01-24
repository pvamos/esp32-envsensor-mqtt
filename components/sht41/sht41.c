#include "sht41.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SHT41";

// SHT41 I2C address
#define SHT41_I2C_ADDRESS 0x44

// SHT41 Commands
#define SHT41_MEASURE_HIGH_PRECISION 0xFD

// Helper macros for delays
#define SHT41_SLEEP_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))

// Internal I2C device object
static i2c_dev_t dev;

/**
 * @brief Initialize the SHT41 sensor on the given I2C bus.
 *
 * @param i2c_port I2C port number (e.g., I2C_NUM_0)
 * @param sda_pin GPIO number for SDA line
 * @param scl_pin GPIO number for SCL line
 * @param freq I2C clock frequency
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t sht41_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq) {
    memset(&dev, 0, sizeof(dev));
    dev.port = i2c_port;
    dev.addr = SHT41_I2C_ADDRESS;
    dev.cfg.mode = I2C_MODE_MASTER;
    dev.cfg.sda_io_num = sda_pin;
    dev.cfg.scl_io_num = scl_pin;
    dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    dev.cfg.master.clk_speed = freq;

    esp_err_t err = i2c_dev_create_mutex(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C mutex for SHT41: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_param_config(dev.port, &dev.cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C for SHT41: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(dev.port, dev.cfg.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver for SHT41: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "SHT41 initialized successfully");
    return err;
}

/**
 * @brief Read temperature and humidity data from the SHT41 sensor.
 *
 * @param temperature Pointer to store temperature in degrees Celsius
 * @param humidity Pointer to store humidity in percentage
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t sht41_read(float *temperature, float *humidity) {
    if (!temperature || !humidity) {
        ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = SHT41_MEASURE_HIGH_PRECISION;
    uint8_t data[6]; // 6 bytes: 2 for temperature, 2 for humidity, 2 for CRC
    uint16_t raw_temperature, raw_humidity;

    // Send measurement command
    I2C_DEV_TAKE_MUTEX(&dev);
    esp_err_t err = i2c_dev_write(&dev, NULL, 0, &cmd, 1);
    if (err != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev);
        ESP_LOGE(TAG, "Failed to send measurement command to SHT41: %s", esp_err_to_name(err));
        return err;
    }
    I2C_DEV_GIVE_MUTEX(&dev);

    // Wait for the measurement to complete
    SHT41_SLEEP_MS(10);

    // Read measurement results
    I2C_DEV_TAKE_MUTEX(&dev);
    err = i2c_dev_read(&dev, NULL, 0, data, 6);
    I2C_DEV_GIVE_MUTEX(&dev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from SHT41: %s", esp_err_to_name(err));
        return err;
    }

    // Process raw data
    raw_temperature = (data[0] << 8) | data[1];
    raw_humidity = (data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * (raw_temperature / 65535.0f);
    *humidity = 100.0f * (raw_humidity / 65535.0f);

    ESP_LOGI(TAG, "SHT41: Temperature = %.2f °C, Humidity = %.2f %%", *temperature, *humidity);
    return ESP_OK;
}

