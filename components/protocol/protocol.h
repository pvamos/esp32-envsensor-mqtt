#ifndef MY_PROTOCOL_H
#define MY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the BCH(1023,1003) environment (once at startup).
 * Builds any needed Galois field tables for GF(2^10).
 */
void protocol_init_bch(void);

/**
 * @brief De-initialize BCH if needed (optional).
 */
void protocol_deinit_bch(void);

/**
 * @brief Build a minimal frame (no optional fields) with data blocks.
 *
 * The final ECC is not added here; call protocol_add_ecc() after building.
 *
 * @param[out] frame_buf  Buffer to hold up to 125 + 3 = 128 bytes.
 * @param[out] frame_len  Final length before ECC (0..125).
 * @param[in]  message_id 16-bit ID in big-endian.
 * @param[in]  location_id 5-byte location.
 * @param[in]  bme_temp   BME280 temperature
 * @param[in]  bme_press  BME280 pressure
 * @param[in]  bme_hum    BME280 humidity
 * @param[in]  tmp117_temp TMP117 temperature
 */
void protocol_build_minimal_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    uint16_t message_id,
    const uint8_t location_id[5],
    float bme_temp,
    float bme_press,
    float bme_hum,
    float tmp117_temp
);

/**
 * @brief Build a full frame (with optional fields) to show how to add MCU fields, etc.
 *
 * @param[out] frame_buf   Buffer up to ~128 bytes.
 * @param[out] frame_len   Used length before ECC.
 * @param[in]  message_id  16-bit ID
 * @param[in]  location_id 5-byte array
 * @param[in]  mcu_type    e.g. 1=ESP32, 2=STM32
 * @param[in]  mcu_serial  4 bytes
 * @param[in]  fw_major    Firmware major
 * @param[in]  fw_minor    Firmware minor
 * @param[in]  bme_temp    BME280 temperature
 * @param[in]  bme_press   BME280 pressure
 * @param[in]  bme_hum     BME280 humidity
 * @param[in]  aht_temp    AHT20 temperature
 * @param[in]  aht_hum     AHT20 humidity
 */
void protocol_build_full_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    uint16_t message_id,
    const uint8_t location_id[5],
    uint8_t  mcu_type,
    const uint8_t mcu_serial[4],
    uint8_t  fw_major,
    uint8_t  fw_minor,
    float bme_temp,
    float bme_press,
    float bme_hum,
    float aht_temp,
    float aht_hum
);

/**
 * @brief Pad to 125 bytes, then compute 20-bit ECC and append 3 bytes.
 *
 * For a total ~128 bytes (the top 4 bits of that last byte are wasted).
 *
 * @param[in,out] frame_buf  The partial frame data
 * @param[in,out] frame_len  Updated final length after ECC
 */
void protocol_add_ecc(uint8_t *frame_buf, size_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif /* MY_PROTOCOL_H */
