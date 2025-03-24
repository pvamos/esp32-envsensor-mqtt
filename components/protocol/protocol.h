#ifndef MY_PROTOCOL_H
#define MY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * BCH(1023,1003) Single-Bit Correction
 *************************************************************************/
#define BCH_M          10    // GF(2^10)
#define BCH_N          1023
#define BCH_K          1003
#define BCH_T          1     // 1-bit correct
#define BCH_PARITY     (BCH_N - BCH_K) // 20 bits
#define FRAME_DATA_LEN 125   // We pad data to 125 bytes
#define FRAME_ECC_LEN   3    // Then append 3 ECC bytes

/**************************************************************************
 * Protocol Field Sizes
 *************************************************************************/
// 2 magic bytes = 'S'(0x53), 'N'(0x4E)
#define PROTOCOL_MAGIC_BYTE_0  0x53
#define PROTOCOL_MAGIC_BYTE_1  0x4E

// We can define a version byte or flags byte if we wish
#define PROTOCOL_VERSION       0x01

// location ID is 6 bytes
#define PROTOCOL_LOCATION_BYTES 6

// message ID is 4 bytes
#define PROTOCOL_MSGID_BYTES    4

// mcu_serial is 12 bytes
#define PROTOCOL_MCUSERIAL_BYTES 12

/**************************************************************************
 * BCH Initialization / Deinit
 *************************************************************************/

/**
 * @brief Initialize the BCH(1023,1003) environment (once at startup).
 *        Builds any needed Galois field tables for GF(2^10).
 */
void protocol_init_bch(void);

/**
 * @brief De-initialize BCH if needed (optional).
 */
void protocol_deinit_bch(void);

/**************************************************************************
 * Frame-Building Functions
 *************************************************************************/

/**
 * @brief Build a MINIMAL frame (no optional fields).
 *
 * The final ECC is not added here; call protocol_add_ecc() after building.
 *
 * @param[out] frame_buf   Buffer for up to 125+3=128 bytes
 * @param[out] frame_len   Actual length before ECC
 * @param[in]  message_id  4-byte message ID (big-endian)
 * @param[in]  location_id 6-byte location ID
 * @param[in]  bme_temp    BME280 temperature
 * @param[in]  bme_press   BME280 pressure
 * @param[in]  bme_hum     BME280 humidity
 * @param[in]  tmp117_temp TMP117 temperature
 */
void protocol_build_minimal_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    const uint8_t message_id[PROTOCOL_MSGID_BYTES],
    const uint8_t location_id[PROTOCOL_LOCATION_BYTES],
    float bme_temp,
    float bme_press,
    float bme_hum,
    float tmp117_temp
);

/**
 * @brief Build a FULL frame (with optional fields).
 *
 * The final ECC is not added here; call protocol_add_ecc() after building.
 *
 * @param[out] frame_buf   Buffer for up to 125+3=128 bytes
 * @param[out] frame_len   Actual length before ECC
 * @param[in]  message_id  4-byte message ID (big-endian)
 * @param[in]  location_id 6-byte location ID
 * @param[in]  mcu_type    e.g. 1=ESP32
 * @param[in]  mcu_serial  12-byte MCU serial
 * @param[in]  fw_major    firmware major
 * @param[in]  fw_minor    firmware minor
 * @param[in]  bme_temp    BME280 temperature
 * @param[in]  bme_press   BME280 pressure
 * @param[in]  bme_hum     BME280 humidity
 * @param[in]  aht_temp    AHT20 temperature
 * @param[in]  aht_hum     AHT20 humidity
 */
void protocol_build_full_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    const uint8_t message_id[PROTOCOL_MSGID_BYTES],
    const uint8_t location_id[PROTOCOL_LOCATION_BYTES],
    uint8_t  mcu_type,
    const uint8_t mcu_serial[PROTOCOL_MCUSERIAL_BYTES],
    uint8_t  fw_major,
    uint8_t  fw_minor,
    float bme_temp,
    float bme_press,
    float bme_hum,
    float aht_temp,
    float aht_hum
);

/**************************************************************************
 * ECC
 *************************************************************************/

/**
 * @brief Pad to FRAME_DATA_LEN, then compute 20-bit BCH remainder (3 bytes).
 *
 * @param[in,out] frame_buf  partial frame data
 * @param[in,out] frame_len  updated final length after ECC appended
 */
void protocol_add_ecc(uint8_t *frame_buf, size_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif // MY_PROTOCOL_H
