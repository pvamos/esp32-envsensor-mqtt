#ifndef MQTT_PUSH_H
#define MQTT_PUSH_H

#include <stdint.h>

/**
 * @brief Initialize the MQTT client and connect to the broker.
 */
void mqtt_init(void);

/**
 * @brief Publish all sensor data to the MQTT broker.
 *
 * @param bme280_temp Temperature from BME280.
 * @param bme280_pressure Pressure from BME280.
 * @param bme280_humidity Humidity from BME280.
 * @param tmp117_temp Temperature from TMP117.
 * @param aht20_temp Temperature from AHT20.
 * @param aht20_humidity Humidity from AHT20.
 * @param sht41_temp Temperature from SHT41.
 * @param sht41_humidity Humidity from SHT41.
 */
void mqtt_publish(float bme280_temp, float bme280_pressure, float bme280_humidity,
                  float tmp117_temp, float aht20_temp, float aht20_humidity,
                  float sht41_temp, float sht41_humidity);

#endif // MQTT_PUSH_H

