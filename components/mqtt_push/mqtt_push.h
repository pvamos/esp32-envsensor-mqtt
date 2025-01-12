#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

// Initialize the MQTT client and connect to the broker
void mqtt_init(void);

/**
 * @brief Publish all sensor data in a single MQTT 5 message.
 *
 * @param bme_raw_temp Raw temperature from BME280.
 * @param bme_raw_pressure Raw pressure from BME280.
 * @param bme_raw_humidity Raw humidity from BME280.
 * @param bme_comp_temp Compensated temperature from BME280.
 * @param bme_comp_pressure Compensated pressure from BME280.
 * @param bme_comp_humidity Compensated humidity from BME280.
 * @param tmp_raw_temp Raw temperature from TMP117.
 * @param tmp_comp_temp Compensated temperature from TMP117.
 * @param aht_raw_temp Raw temperature from AHT20.
 * @param aht_raw_humidity Raw humidity from AHT20.
 * @param aht_comp_temp Compensated temperature from AHT20.
 * @param aht_comp_humidity Compensated humidity from AHT20.
 */
void mqtt_publish_all(int32_t bme_raw_temp, int32_t bme_raw_pressure, int32_t bme_raw_humidity,
                      float bme_comp_temp, float bme_comp_pressure, float bme_comp_humidity,
                      int16_t tmp_raw_temp, float tmp_comp_temp,
                      int32_t aht_raw_temp, int32_t aht_raw_humidity,
                      float aht_comp_temp, float aht_comp_humidity);

#endif // MQTT_H
