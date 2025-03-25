/*
 * protocol.c
 *
 * This file now ONLY handles sensor-block formatting for minimal vs. full frames.
 * We removed BFS-based ECC or any bch init calls. 
 * ECC generation is done externally by bch_encode() after we finish building the frame.
 */

#include "protocol.h"
#include <string.h> // memset, memcpy

// Helper: write float as big-endian 4 bytes
static void put_float_be(uint8_t *dest, float val)
{
    union {
        float f;
        uint8_t b[4];
    } conv;
    conv.f = val;
    dest[0] = conv.b[3];
    dest[1] = conv.b[2];
    dest[2] = conv.b[1];
    dest[3] = conv.b[0];
}

/****************************************************************
 * Minimal Frame
 * 4 sensor blocks, no optional fields
 ****************************************************************/
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
){
    // zero out the buffer first
    memset(frame_buf, 0, 125);

    size_t offset = 0;

    // 1) Magic "SN"
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_0; // 0x53
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_1; // 0x4E

    // 2) Version
    frame_buf[offset++] = PROTOCOL_VERSION;

    // 3) Flags => e.g. 0 => minimal
    uint8_t flags = 0x00;
    frame_buf[offset++] = flags;

    // 4) 4-byte message ID
    for (int i = 0; i < PROTOCOL_MSGID_BYTES; i++) {
        frame_buf[offset++] = message_id[i];
    }

    // 5) 6-byte location
    memcpy(&frame_buf[offset], location_id, PROTOCOL_LOCATION_BYTES);
    offset += PROTOCOL_LOCATION_BYTES;

    // 6) sensorCount => 4
    frame_buf[offset++] = 4;

    // --- Block #1 => BME280 => type=3 => 3 floats
    frame_buf[offset++] = 0x00; // high byte of type
    frame_buf[offset++] = 0x03; // low  byte => type=3
    frame_buf[offset++] = 3;    // 3 float32

    put_float_be(&frame_buf[offset], bme_temp);  offset += 4;
    put_float_be(&frame_buf[offset], bme_press); offset += 4;
    put_float_be(&frame_buf[offset], bme_hum);   offset += 4;

    // --- Block #2 => TMP117 => type=6 => 1 float
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x06;
    frame_buf[offset++] = 1;

    put_float_be(&frame_buf[offset], tmp117_temp);
    offset += 4;

    // --- Block #3 => AHT20 => type=5 => 2 floats
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x05;
    frame_buf[offset++] = 2;

    put_float_be(&frame_buf[offset], aht_temp);  offset += 4;
    put_float_be(&frame_buf[offset], aht_hum);   offset += 4;

    // --- Block #4 => SHT41 => type=4 => 2 floats
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x04;
    frame_buf[offset++] = 2;

    put_float_be(&frame_buf[offset], sht_temp);  offset += 4;
    put_float_be(&frame_buf[offset], sht_hum);   offset += 4;

    *frame_len = offset;
}

/****************************************************************
 * Full Frame
 * includes optional fields + sensor serial in each block
 ****************************************************************/
void protocol_build_full_frame(
    uint8_t *frame_buf,
    size_t  *frame_len,
    const uint8_t message_id[PROTOCOL_MSGID_BYTES],
    const uint8_t location_id[PROTOCOL_LOCATION_BYTES],
    uint8_t  mcu_type,
    const uint8_t mcu_serial[PROTOCOL_MCUSERIAL_BYTES],
    uint8_t  fw_major,
    uint8_t  fw_minor,
    const uint8_t bme_serial[8], float bme_temp, float bme_press, float bme_hum,
    const uint8_t tmp_serial[8], float tmp117_temp,
    const uint8_t aht_serial[8], float aht_temp, float aht_hum,
    const uint8_t sht_serial[8], float sht_temp, float sht_hum
){
    // zero out the buffer
    memset(frame_buf, 0, 125);

    size_t offset = 0;

    // 1) Magic "SN"
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_0; 
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_1;

    // 2) Version
    frame_buf[offset++] = PROTOCOL_VERSION;

    // 3) Flags => bit1..3 => 0x0E => optional fields
    uint8_t flags = 0x0E;
    frame_buf[offset++] = flags;

    // 4) 4-byte message ID
    for (int i = 0; i < PROTOCOL_MSGID_BYTES; i++) {
        frame_buf[offset++] = message_id[i];
    }

    // 5) 6-byte location
    memcpy(&frame_buf[offset], location_id, PROTOCOL_LOCATION_BYTES);
    offset += PROTOCOL_LOCATION_BYTES;

    // Optional fields => bit1=1 => 1 byte => mcu_type
    frame_buf[offset++] = mcu_type;

    // bit2=1 => 12-byte MCU serial
    memcpy(&frame_buf[offset], mcu_serial, PROTOCOL_MCUSERIAL_BYTES);
    offset += PROTOCOL_MCUSERIAL_BYTES;

    // bit3=1 => 2-byte firmware version
    frame_buf[offset++] = fw_major;
    frame_buf[offset++] = fw_minor;

    // sensorCount => 4
    frame_buf[offset++] = 4;

    // Block #1 => BME280 => type=3 => 8-byte serial + 3 floats => total 5
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x03;
        frame_buf[offset++] = 5; // 2 for 8-byte serial + 3 actual floats

        // 8-byte sensor serial
        memcpy(&frame_buf[offset], bme_serial, 8);
        offset += 8;

        // 3 floats
        put_float_be(&frame_buf[offset], bme_temp);   offset += 4;
        put_float_be(&frame_buf[offset], bme_press);  offset += 4;
        put_float_be(&frame_buf[offset], bme_hum);    offset += 4;
    }

    // Block #2 => TMP117 => type=6 => 8-byte serial + 1 float => total 3
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x06;
        frame_buf[offset++] = 3;

        memcpy(&frame_buf[offset], tmp_serial, 8);
        offset += 8;

        put_float_be(&frame_buf[offset], tmp117_temp);
        offset += 4;
    }

    // Block #3 => AHT20 => type=5 => 8-byte serial + 2 floats => total 4
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x05;
        frame_buf[offset++] = 4;

        memcpy(&frame_buf[offset], aht_serial, 8);
        offset += 8;

        put_float_be(&frame_buf[offset], aht_temp); offset += 4;
        put_float_be(&frame_buf[offset], aht_hum);  offset += 4;
    }

    // Block #4 => SHT41 => type=4 => 8-byte serial + 2 floats => total 4
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x04;
        frame_buf[offset++] = 4;

        memcpy(&frame_buf[offset], sht_serial, 8);
        offset += 8;

        put_float_be(&frame_buf[offset], sht_temp); offset += 4;
        put_float_be(&frame_buf[offset], sht_hum);  offset += 4;
    }

    *frame_len = offset;
}
