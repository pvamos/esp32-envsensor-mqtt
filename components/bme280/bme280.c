#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280.h"
#include "i2c.h"
#include "esp_log.h"
#include <math.h>
#include <inttypes.h> // For PRI macros

// Calibration parameters
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5, dig_H6;

static int32_t t_fine; // Shared variable for compensation calculations

static const char *TAG = "BME280";

static i2c_master_dev_handle_t bme280_dev_handle = NULL;

// Function to read calibration data
static esp_err_t read_calibration_data() {
    uint8_t calib[26];
    uint8_t reg = 0x88;
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, calib, 26, pdMS_TO_TICKS(1000)));

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    reg = 0xA1;
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, &dig_H1, 1, pdMS_TO_TICKS(1000)));

    reg = 0xE1;
    uint8_t hum_calib[7];
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, hum_calib, 7, pdMS_TO_TICKS(1000)));

    dig_H2 = (hum_calib[1] << 8) | hum_calib[0];
    dig_H3 = hum_calib[2];
    dig_H4 = (hum_calib[3] << 4) | (hum_calib[4] & 0x0F);
    dig_H5 = (hum_calib[5] << 4) | (hum_calib[4] >> 4);
    dig_H6 = hum_calib[6];

    ESP_LOGI(TAG, "Calibration data read successfully.");
    return ESP_OK;
}

// Initialize the BME280 sensor
esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &bme280_dev_handle));

    uint8_t reg = BME280_REG_ID;
    uint8_t chip_id;

    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, &chip_id, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    if (chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: expected 0x%02X, got 0x%02X", BME280_CHIP_ID, chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "BME280 detected, chip ID: 0x%02X", chip_id);

    // Reset the sensor
    uint8_t reset_cmd[2] = {BME280_REG_RESET, 0xB6};
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, reset_cmd, sizeof(reset_cmd), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    vTaskDelay(pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)); // Wait for reset to complete

    // Load calibration data
    ESP_ERROR_CHECK(read_calibration_data());

    // Configure the sensor
    uint8_t ctrl_hum[2] = {BME280_REG_CTRL_HUM, 0x01};  // Humidity oversampling x1
    uint8_t ctrl_meas[2] = {BME280_REG_CTRL_MEAS, 0x27}; // Temp and Pressure oversampling x1, Normal mode
    uint8_t config[2] = {BME280_REG_CONFIG, 0xA0};       // Standby time 1000ms, Filter x4

    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, ctrl_hum, sizeof(ctrl_hum), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, ctrl_meas, sizeof(ctrl_meas), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, config, sizeof(config), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    ESP_LOGI(TAG, "BME280 initialized successfully");
    return ESP_OK;
}

// Combined read and compensate function
esp_err_t bme280_read(float *temperature, float *pressure, float *humidity) {
    uint8_t reg = BME280_REG_DATA;
    uint8_t data[8];

    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    int32_t raw_temp = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    int32_t raw_press = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t raw_hum = (int32_t)((data[6] << 8) | data[7]);

    // Temperature compensation
    int32_t var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature /= 100.0f;

    // Pressure compensation
    int64_t varp1, varp2, press;
    varp1 = ((int64_t)t_fine) - 128000;
    varp2 = varp1 * varp1 * (int64_t)dig_P6;
    varp2 += ((varp1 * (int64_t)dig_P5) << 17);
    varp2 += (((int64_t)dig_P4) << 35);
    varp1 = ((varp1 * varp1 * (int64_t)dig_P3) >> 8) + ((varp1 * (int64_t)dig_P2) << 12);
    varp1 = (((((int64_t)1) << 47) + varp1)) * ((int64_t)dig_P1) >> 33;
    if (varp1 == 0) {
        *pressure = 0; // Avoid division by zero
    } else {
        press = 1048576 - raw_press;
        press = (((press << 31) - varp2) * 3125) / varp1;
        varp1 = (((int64_t)dig_P9) * (press >> 13) * (press >> 13)) >> 25;
        varp2 = (((int64_t)dig_P8) * press) >> 19;
        press = ((press + varp1 + varp2) >> 8) + (((int64_t)dig_P7) << 4);
        *pressure = press / 256.0f / 100.0f; // Convert to hPa
    }

    // Humidity compensation
    int32_t varh = (t_fine - ((int32_t)76800));
    varh = (((((raw_hum << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * varh)) + ((int32_t)16384)) >> 15) *
            (((((((varh * ((int32_t)dig_H6)) >> 10) * (((varh * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
               ((int32_t)2097152)) *
                  ((int32_t)dig_H2) +
              8192) >>
             14));
    varh -= (((((varh >> 15) * (varh >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4);
    varh = (varh < 0 ? 0 : varh);
    varh = (varh > 419430400 ? 419430400 : varh);
    *humidity = (varh >> 12) / 1024.0f;

    ESP_LOGI(TAG, "BME280: Temperature=%.2f °C, Pressure=%.2f hPa, Humidity=%.2f %%RH", *temperature, *pressure, *humidity);
    return ESP_OK;
}

