#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize internal temperature measurement.
 *
 * On ESP32-C3 this uses the ESP-IDF temperature_sensor driver
 * (esp_driver_tsens component) to read the internal, calibrated
 * on-die temperature in °C.
 */
esp_err_t esp_temp_init(void);

/**
 * @brief Read internal temperature in °C.
 *
 * @param out_temp Pointer to store temperature in degrees Celsius.
 * @return ESP_OK on success, or error code from the temperature_sensor driver.
 *
 * If the read fails, returns an error and does not modify out_temp.
 */
esp_err_t esp_temp_read_c(float *out_temp);

#ifdef __cplusplus
}
#endif
