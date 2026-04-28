// components/bme280/bme280.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme280.h"
#include "i2c.h"
#include "esp_log.h"
#include <math.h>

#define BME280_REG_STATUS 0xF3

#define BME280_STATUS_MEASURING_MASK (1U << 3)
#define BME280_STATUS_IM_UPDATE_MASK (1U << 0)

#define BME280_WAIT_IM_UPDATE_TIMEOUT_MS 50
#define BME280_WAIT_MEAS_TIMEOUT_MS      250
#define BME280_POST_CONFIG_DELAY_MS      20

/* Oversampling codes (datasheet encoding)
 * 000=skipped, 001=x1, 010=x2, 011=x4, 100=x8, 101=x16
 */
#define BME280_OSRS_X1   0x01
#define BME280_OSRS_X4   0x03
#define BME280_OSRS_X16  0x05

#define BME280_CTRL_MEAS_OSRS_T(code)   ((uint8_t)((code) << 5))
#define BME280_CTRL_MEAS_OSRS_P(code)   ((uint8_t)((code) << 2))
#define BME280_CTRL_HUM_OSRS_H(code)    ((uint8_t)((code) & 0x07))

#define BME280_CTRL_MEAS_MODE_SLEEP     0x00       // mode = 00
#define BME280_CTRL_MEAS_MODE_FORCED    0x01       // mode = 01 (forced)

/* Calibration parameters */
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5;
static int8_t  dig_H6;

static int32_t t_fine; // Shared variable for compensation calculations

/* Base ctrl_meas value (oversampling only, without mode bits) */
static uint8_t s_ctrl_meas_base = 0;

static const char *TAG = "BME280";

static i2c_master_dev_handle_t bme280_dev_handle = NULL;

static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!bme280_dev_handle) return ESP_ERR_INVALID_STATE;
    return i2c_write_read(bme280_dev_handle, &reg, 1, buf, len);
}

static esp_err_t bme280_write_reg_u8(uint8_t reg, uint8_t val)
{
    if (!bme280_dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t w[2] = { reg, val };
    return i2c_write(bme280_dev_handle, w, sizeof(w));
}

static esp_err_t bme280_wait_status_clear(uint8_t mask, TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    while (1) {
        uint8_t st = 0;
        esp_err_t err = bme280_read_regs(BME280_REG_STATUS, &st, 1);
        if (err != ESP_OK) return err;

        if ((st & mask) == 0) return ESP_OK;

        if ((xTaskGetTickCount() - start) > timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* Read calibration data */
static esp_err_t read_calibration_data(void)
{
    uint8_t hum_calib[7]; // 0xE1..0xE7
    uint8_t calib[26];

    esp_err_t err = bme280_read_regs(0x88, calib, sizeof(calib));
    if (err != ESP_OK) return err;

    dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
    dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

    dig_P1 = (uint16_t)((calib[7] << 8) | calib[6]);
    dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);

    err = bme280_read_regs(0xA1, &dig_H1, 1);
    if (err != ESP_OK) return err;

    err = bme280_read_regs(0xE1, hum_calib, sizeof(hum_calib));
    if (err != ESP_OK) return err;

    dig_H2 = (int16_t)((hum_calib[1] << 8) | hum_calib[0]);
    dig_H3 = hum_calib[2];
    dig_H4 = (int16_t)((hum_calib[3] << 4) | (hum_calib[4] & 0x0F));
    dig_H5 = (int16_t)((hum_calib[5] << 4) | (hum_calib[4] >> 4));
    dig_H6 = (int8_t)hum_calib[6];

    /* Sanity checks: if these are bad, pressure/temp will often become 0 in compensation */
    if (dig_T1 == 0 || dig_T1 == 0xFFFF || dig_P1 == 0 || dig_P1 == 0xFFFF) {
        ESP_LOGE(TAG, "Calibration looks invalid (T1=%u, P1=%u). Likely read too early after reset.", dig_T1, dig_P1);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "Calibration data read successfully.");
    return ESP_OK;
}

/* Initialize the BME280 sensor */
esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t err = i2c_add_device(bus_handle, BME280_I2C_ADDR, &bme280_dev_handle);
    if (err != ESP_OK) return err;

    uint8_t chip_id = 0;
    err = bme280_read_regs(BME280_REG_ID, &chip_id, 1);
    if (err != ESP_OK) return err;

    if (chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: expected 0x%02X, got 0x%02X", BME280_CHIP_ID, chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "BME280 detected, chip ID: 0x%02X", chip_id);

    /* Reset the sensor */
    err = bme280_write_reg_u8(BME280_REG_RESET, 0xB6);
    if (err != ESP_OK) return err;

    /* Small minimum delay, then wait for NVM->image copy (im_update) to finish */
    vTaskDelay(pdMS_TO_TICKS(BME280_RESET_DELAY_MS));
    err = bme280_wait_status_clear(BME280_STATUS_IM_UPDATE_MASK, pdMS_TO_TICKS(BME280_WAIT_IM_UPDATE_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Timeout waiting for im_update to clear after reset");
        return err;
    }

    /* Load calibration data (now safe) */
    err = read_calibration_data();
    if (err != ESP_OK) return err;

    /* Configure the sensor */
    /* Ensure sleep before changing config/ctrl_hum (datasheet-safe) */
    err = bme280_write_reg_u8(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_MODE_SLEEP);
    if (err != ESP_OK) return err;

    /* Humidity oversampling x1 (must be written before ctrl_meas to take effect) */
    err = bme280_write_reg_u8(BME280_REG_CTRL_HUM, BME280_CTRL_HUM_OSRS_H(BME280_OSRS_X1));
    if (err != ESP_OK) return err;

    /* Standby 1000ms, IIR filter x4 (comment from original code; effective filter per value) */
    err = bme280_write_reg_u8(BME280_REG_CONFIG, 0xA8);
    if (err != ESP_OK) return err;

    /*
     * Keep the sensor in SLEEP.
     * Forced-mode conversions will be started in bme280_read().
     * s_ctrl_meas_base is retained as an "initialized" guard and default.
     */
    s_ctrl_meas_base = BME280_CTRL_MEAS_OSRS_T(BME280_OSRS_X1) |
                       BME280_CTRL_MEAS_OSRS_P(BME280_OSRS_X1);
    err = bme280_write_reg_u8(BME280_REG_CTRL_MEAS,
                              s_ctrl_meas_base | BME280_CTRL_MEAS_MODE_SLEEP);
    if (err != ESP_OK) return err;

    /* Give it time for internal settling so an immediate read won't see zeros */
    vTaskDelay(pdMS_TO_TICKS(BME280_POST_CONFIG_DELAY_MS));

    ESP_LOGI(TAG, "BME280 initialized successfully (forced mode; oversampling set per read)");
    return ESP_OK;
}

static esp_err_t bme280_forced_read_osrs(uint8_t osrs_t_code,
                                        uint8_t osrs_p_code,
                                        uint8_t osrs_h_code,
                                        float *temperature,
                                        float *pressure,
                                        float *humidity)
{
    if (!temperature || !pressure || !humidity) return ESP_ERR_INVALID_ARG;
    if (!bme280_dev_handle) return ESP_ERR_INVALID_STATE;

    uint8_t ctrl_meas_base =
        BME280_CTRL_MEAS_OSRS_T(osrs_t_code) |
        BME280_CTRL_MEAS_OSRS_P(osrs_p_code);

    /*
     * Ensure SLEEP before changing oversampling (datasheet-safe).
     * Also preload osrs_t/osrs_p in ctrl_meas while in SLEEP.
     */
    esp_err_t err = bme280_write_reg_u8(BME280_REG_CTRL_MEAS,
                                        ctrl_meas_base | BME280_CTRL_MEAS_MODE_SLEEP);
    if (err != ESP_OK) return err;

    /* Humidity oversampling must be written before ctrl_meas to take effect */
    err = bme280_write_reg_u8(BME280_REG_CTRL_HUM, BME280_CTRL_HUM_OSRS_H(osrs_h_code));
    if (err != ESP_OK) return err;

    /* Start forced conversion */
    err = bme280_write_reg_u8(BME280_REG_CTRL_MEAS,
                              ctrl_meas_base | BME280_CTRL_MEAS_MODE_FORCED);
    if (err != ESP_OK) return err;

    /* Wait until conversion finishes (MEASURING bit goes back to 0) */
    err = bme280_wait_status_clear(BME280_STATUS_MEASURING_MASK,
                                   pdMS_TO_TICKS(BME280_WAIT_MEAS_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    uint8_t data[8];
    uint8_t reg = BME280_REG_DATA;

    err = bme280_read_regs(reg, data, sizeof(data));
    if (err != ESP_OK) return err;

    int32_t raw_temp  = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    int32_t raw_press = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t raw_hum   = (int32_t)((data[6] << 8) | data[7]);

    /* Temperature compensation */
    int32_t var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature /= 100.0f;

    /* Pressure compensation */
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

    /* Humidity compensation */
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

    return ESP_OK;
}

/* Combined read and compensate function */
esp_err_t bme280_read(float *temperature, float *pressure, float *humidity)
{
    if (!temperature || !pressure || !humidity) return ESP_ERR_INVALID_ARG;
    if (!bme280_dev_handle) return ESP_ERR_INVALID_STATE;
    if (s_ctrl_meas_base == 0) return ESP_ERR_INVALID_STATE;

    /*
     * Requested sequence:
     *  1) Forced read at 1/1/4 and discard values
     *  2) Forced read at 16/16/16 and return values
     */

    /* 1) Pre-warming dummy read: T/P/H = 1/1/4, discard values */
    float t0 = 0.0f, p0 = 0.0f, h0 = 0.0f;
    esp_err_t err = bme280_forced_read_osrs(BME280_OSRS_X1, BME280_OSRS_X1, BME280_OSRS_X4,
                                           &t0, &p0, &h0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME280 warm-up (1/1/4) failed: %s; continuing",
                 esp_err_to_name(err));
    }

    /* 2) Real read: T/P/H = 16/16/16, return values */
    err = bme280_forced_read_osrs(BME280_OSRS_X16, BME280_OSRS_X16, BME280_OSRS_X16,
                                  temperature, pressure, humidity);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 final (16/16/16) failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BME280 (final 16/16/16): Temperature=%.2f °C, Pressure=%.2f hPa, Humidity=%.2f %%RH",
             *temperature, *pressure, *humidity);

    return ESP_OK;
}
