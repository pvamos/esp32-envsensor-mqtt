#ifndef MQTT_PUSH_H
#define MQTT_PUSH_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize the MQTT client and connect to the broker.
 */
void mqtt_init(void);

/**
 * @brief Publish a raw binary frame to MQTT.
 *
 * @param frame Pointer to the binary data to send
 * @param frame_len Length of the data
 */
void mqtt_publish_binary(const uint8_t *frame, size_t frame_len);

#endif // MQTT_PUSH_H
