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

// Read calibration data from BME280
esp_err_t read_calibration_data() {
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
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(1000)));
    uint8_t hum_calib[7];
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

// Read raw data from the BME280
esp_err_t bme280_read_raw(int32_t *temperature, int32_t *pressure, int32_t *humidity) {
    uint8_t reg = BME280_REG_DATA;
    uint8_t data[8];

    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, &reg, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_receive(bme280_dev_handle, data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));

    *pressure = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    *temperature = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    *humidity = (int32_t)((data[6] << 8) | data[7]);

    ESP_LOGI(TAG, "BME280 RAW Temperature: %" PRId32 ", Pressure: %" PRId32 ", Humidity: %" PRId32, 
             *temperature, *pressure, *humidity);
    return ESP_OK;
}

// Calculate compensated values
esp_err_t bme280_calculate_compensated(int32_t temp_raw, int32_t press_raw, int32_t hum_raw,
                      float *temp_final, float *press_final, float *hum_final) {
    int32_t var1, var2, T;
    var1 = ((((temp_raw >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((temp_raw >> 4) - ((int32_t)dig_T1)) * ((temp_raw >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >>
           14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    *temp_final = T / 100.0f;

    int64_t p1, p2;
    p1 = ((int64_t)t_fine) - 128000;
    p2 = p1 * p1 * (int64_t)dig_P6;
    p2 = p2 + ((p1 * (int64_t)dig_P5) << 17);
    p2 = p2 + (((int64_t)dig_P4) << 35);
    p1 = ((p1 * p1 * (int64_t)dig_P3) >> 8) + ((p1 * (int64_t)dig_P2) << 12);
    p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)dig_P1) >> 33;
    if (p1 == 0) {
        *press_final = 0; // Avoid division by zero
    } else {
        int64_t p = 1048576 - press_raw;
        p = (((p << 31) - p2) * 3125) / p1;
        p1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + p1 + p2) >> 8) + (((int64_t)dig_P7) << 4);
        *press_final = p / 256.0f / 100.0f;
    }

    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((hum_raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *hum_final = (v_x1_u32r >> 12) / 1024.0f;

    ESP_LOGI(TAG, "BME280 Temperature: %.2f °C, Pressure: %.4f hPa, Humidity %.3f %%RH",
         (double)*temp_final, (double)*press_final, (double)*hum_final);
    
    return ESP_OK;
}
