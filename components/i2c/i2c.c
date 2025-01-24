#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "I2C";

// Initialize the I2C master
esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// Add an I2C device to the bus
esp_err_t i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, i2c_master_dev_handle_t *dev_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at address 0x%02X: %s", dev_addr, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C device at address 0x%02X added successfully", dev_addr);
    return ESP_OK;
}

// Write data to an I2C device
esp_err_t i2c_write(i2c_master_dev_handle_t dev_handle, const uint8_t *data, size_t data_len) {
    esp_err_t err = i2c_master_transmit(dev_handle, data, data_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to I2C device: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Wrote %d bytes to I2C device", data_len);
    return ESP_OK;
}

// Read data from an I2C device
esp_err_t i2c_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t data_len) {
    esp_err_t err = i2c_master_receive(dev_handle, data, data_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from I2C device: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Read %d bytes from I2C device", data_len);
    return ESP_OK;
}

