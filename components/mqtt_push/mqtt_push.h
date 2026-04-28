// components/mqtt_push/mqtt_push.h

#ifndef MQTT_PUSH_H
#define MQTT_PUSH_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Publish a single sensor reading to the MQTT broker.
 *
 * The function performs a one-shot connect -> publish -> disconnect sequence.
 * Any float parameter may be NAN to indicate "missing".
 *
 * On MQTT transport/connection/publish failure it will retry the whole
 * connect+publish sequence exactly once. If both attempts fail, an error
 * is returned and log messages are emitted.
 *
 * For envsensor_Reading:
 *   - mac      : taken from Wi-Fi STA MAC (48-bit, encoded into fixed64)
 *   - rssi     : Wi-Fi RSSI in dBm (negative)
 *   - batt     : currently hardcoded to 0 (placeholder)
 *   - esp32_t  : internal ESP32 temperature in °C; if NAN is passed, 0.0f is encoded
 *   - bme280   : populated only if at least one BME280 value is non-NAN
 *   - sht4x    : populated only if at least one SHT4x value is non-NAN
 */
esp_err_t mqtt_publish_once(
    int32_t rssi_dbm,
    float esp32_temp,
    float bme280_temp, float bme280_pressure, float bme280_humidity,
    float sht4x_temp, float sht4x_humidity);

/**
 * @brief Return the broker URI used by mqtt_publish_once().
 */
const char *mqtt_push_get_broker_uri(void);

/**
 * @brief Return the MQTT topic used by mqtt_publish_once().
 */
const char *mqtt_push_get_topic(void);

#endif // MQTT_PUSH_H
