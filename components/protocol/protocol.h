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
#define BCH_T          1     // correct up to 1 bit
#define BCH_PARITY     (BCH_N - BCH_K) // 20 bits total
#define FRAME_DATA_LEN 125   // we pad data to 125 bytes
#define FRAME_ECC_LEN   3    // then append 3 ECC bytes => 128 total

/**************************************************************************
 * Protocol Field Sizes
 *************************************************************************/
// 2 magic bytes = 'S'(0x53), 'N'(0x4E)
#define PROTOCOL_MAGIC_BYTE_0  0x53
#define PROTOCOL_MAGIC_BYTE_1  0x4E

// protocol version or usage
#define PROTOCOL_VERSION       0x01

// minimal vs. full frames may differ in flags bits
// but we define them in code for clarity

// location ID is 6 bytes
#define PROTOCOL_LOCATION_BYTES 6

// message ID is 4 bytes
#define PROTOCOL_MSGID_BYTES    4

// mcu_serial is 12 bytes
#define PROTOCOL_MCUSERIAL_BYTES 12

// sensor block approach:
// minimal does not store sensor serial
// full does store sensor serial => 8 bytes => 2 extra float slots in that block

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
 * @brief Build a MINIMAL frame with 4 sensor blocks:
 *        1) BME280 => 3 floats
 *        2) TMP117 => 1 float
 *        3) AHT20 => 2 floats
 *        4) SHT41 => 2 floats
 *
 * No sensor serial, no optional fields (MCU type/serial, FW).
 * The final ECC is not added here; call protocol_add_ecc().
 *
 * @param[out] frame_buf    Buffer for up to (125+3) bytes
 * @param[out] frame_len    Actual length before ECC
 * @param[in]  message_id   4-byte message ID
 * @param[in]  location_id  6-byte location ID
 * @param[in]  bme_temp,bme_press,bme_hum  BME280 readings
 * @param[in]  tmp117_temp  TMP117 reading
 * @param[in]  aht_temp,aht_hum   AHT20 readings
 * @param[in]  sht_temp,sht_hum   SHT41 readings
 */
void protocol_build_minimal_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    const uint8_t message_id[PROTOCOL_MSGID_BYTES],
    const uint8_t location_id[PROTOCOL_LOCATION_BYTES],
    float bme_temp,
    float bme_press,
    float bme_hum,
    float tmp117_temp,
    float aht_temp,
    float aht_hum,
    float sht_temp,
    float sht_hum
);

/**
 * @brief Build a FULL frame with:
 *         - optional fields: MCU type (1 byte), 12-byte MCU serial, FW major/minor
 *         - 4 sensor blocks, each with an 8-byte sensor serial plus the measured floats:
 *            1) BME280 => 8-byte serial + 3 floats => total 5 float slots
 *            2) TMP117 => 8-byte serial + 1 float => total 3 float slots
 *            3) AHT20  => 8-byte serial + 2 floats => total 4 float slots
 *            4) SHT41 =>  8-byte serial + 2 floats => total 4 float slots
 *
 * The final ECC is not added here; call protocol_add_ecc().
 *
 * @param[out] frame_buf    Buffer for up to (125+3) bytes
 * @param[out] frame_len    Actual length before ECC
 * @param[in]  message_id   4-byte message ID
 * @param[in]  location_id  6-byte location ID
 * @param[in]  mcu_type     e.g. 1=ESP32
 * @param[in]  mcu_serial   12-byte MCU serial
 * @param[in]  fw_major, fw_minor
 * @param[in]  bme_serial (8 bytes), bme_temp, bme_press, bme_hum
 * @param[in]  tmp_serial (8 bytes), tmp117_temp
 * @param[in]  aht_serial (8 bytes), aht_temp,aht_hum
 * @param[in]  sht_serial (8 bytes), sht_temp,sht_hum
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
    const uint8_t bme_serial[8],
    float bme_temp,
    float bme_press,
    float bme_hum,
    const uint8_t tmp_serial[8],
    float tmp117_temp,
    const uint8_t aht_serial[8],
    float aht_temp,
    float aht_hum,
    const uint8_t sht_serial[8],
    float sht_temp,
    float sht_hum
);

/**************************************************************************
 * ECC
 *************************************************************************/

/**
 * @brief Pad to 125 bytes, then compute the 20-bit BCH remainder and append 3 bytes.
 *
 * @param[in,out] frame_buf partial frame data
 * @param[in,out] frame_len updated final length after ECC appended
 */
void protocol_add_ecc(uint8_t *frame_buf, size_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif // MY_PROTOCOL_H
