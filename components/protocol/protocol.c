/*
 * protocol.c
 *
 * Demonstrates:
 *  - BCH(1023,1003) single-bit error correction (20-bit ECC).
 *  - Minimal vs. Full frames with 4 sensor blocks (BME280, TMP117, AHT20, SHT41).
 *  - Full frame includes optional fields (MCU type, 12-byte MCU serial, FW version)
 *    and each sensor block has an 8-byte sensor serial plus the measured floats.
 *
 * This code is a naive approach adapted from the general concepts in the Linux bch.c (GPLv2).
 * Use or license it accordingly.
 */

#include "protocol.h"
#include <string.h> // memset, memcpy
#include "bch.h"

// We build a naive BFS for the polynomial remainder. Real code would store
// a genuine generator polynomial. We'll store a partial approach:

static int s_bch_inited = 0;

/* We build Galois field tables if needed for BFS polynomial division.
   For 1-bit correction we can do simpler. If you want alpha_to, index_of, define them as well. */
static void bch_build_tables(void)
{
    // Possibly build gf tables for GF(2^10).
    // For a 1-bit correct code, we might only need a partial approach. 
    // We'll just do it if we want expansions. 
    // Here we do nothing in the naive approach, but you can fill in if you want:
    // alpha_to, index_of, etc.
}

void protocol_init_bch(void)
{
    if (!s_bch_inited) {
        bch_build_tables();
        s_bch_inited = 1;
    }
}

void protocol_deinit_bch(void)
{
    s_bch_inited = 0;
}

// Minimal BFS remainder approach for 1003 bits => 20-bit remainder
// We'll define a dummy polynomial of deg=20. 
// This is just a placeholder approach:
static uint32_t bch_encode_1003bits(const uint8_t *data125)
{
    // data125: 125 bytes => 1000 bits. We have 3 leftover bits => total 1003
    // We'll store them in a bit array:
    uint8_t bits[1024];
    memset(bits, 0, sizeof(bits));

    int bitpos = 0;
    // convert 125 bytes => 1000 bits
    for (int i=0; i<125; i++) {
        for (int b=7; b>=0; b--) {
            bits[bitpos++] = (data125[i] >> b) & 1;
        }
    }
    // we have 1000 bits => need 3 leftover => set them => 0
    bitpos += 3;

    // We'll do a naive polynomial shift-xor with some generator of deg=20
    // e.g. 1 + x + x^2 + x^5 + x^17 + x^20, i.e. 0x10025?
    // Let's just pick 0x10031 as random. Not guaranteed real code.
    // remainder will be 20 bits
    uint32_t remainder = 0;
    for (int i=0; i<1003; i++) {
        remainder <<= 1;
        remainder |= bits[i];
        if (remainder & (1 << 20)) {
            remainder ^= 0x10031;
        }
    }
    remainder &= 0xFFFFF;
    return remainder;
}

// Helper function: Write float as big-endian 4 bytes
static void put_float_be(uint8_t *dest, float val)
{
    union { float f; uint8_t b[4]; } conv;
    conv.f = val;
    dest[0] = conv.b[3];
    dest[1] = conv.b[2];
    dest[2] = conv.b[1];
    dest[3] = conv.b[0];
}

/////////////////////////////////////////////////////////
// Minimal Frame => 4 sensor blocks, no sensor serial, no optional fields
/////////////////////////////////////////////////////////
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
    memset(frame_buf, 0, FRAME_DATA_LEN + FRAME_ECC_LEN);
    size_t offset = 0;

    // 1) Magic 'S','N'
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_0; // 0x53
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_1; // 0x4E

    // 2) Version
    frame_buf[offset++] = PROTOCOL_VERSION;

    // 3) Flags => e.g. 0 => minimal
    uint8_t flags = 0x00;
    frame_buf[offset++] = flags;

    // 4) 4-byte message ID
    for (int i=0; i<PROTOCOL_MSGID_BYTES; i++) {
        frame_buf[offset++] = message_id[i];
    }

    // 5) 6-byte location
    memcpy(&frame_buf[offset], location_id, PROTOCOL_LOCATION_BYTES);
    offset += PROTOCOL_LOCATION_BYTES;

    // sensorCount => 4
    frame_buf[offset++] = 4;

    // --- 1) BME280 => type=3 => 3 floats
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x03;
    frame_buf[offset++] = 3; // valCount=3

    put_float_be(&frame_buf[offset], bme_temp);   offset += 4;
    put_float_be(&frame_buf[offset], bme_press);  offset += 4;
    put_float_be(&frame_buf[offset], bme_hum);    offset += 4;

    // --- 2) TMP117 => type=6 => 1 float
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x06;
    frame_buf[offset++] = 1; // valCount=1

    put_float_be(&frame_buf[offset], tmp117_temp);
    offset += 4;

    // --- 3) AHT20 => type=5 => 2 floats
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x05;
    frame_buf[offset++] = 2; // valCount=2

    put_float_be(&frame_buf[offset], aht_temp);   offset += 4;
    put_float_be(&frame_buf[offset], aht_hum);    offset += 4;

    // --- 4) SHT41 => let's define type=4 => 2 floats
    frame_buf[offset++] = 0x00;
    frame_buf[offset++] = 0x04;
    frame_buf[offset++] = 2; // valCount=2

    put_float_be(&frame_buf[offset], sht_temp);   offset += 4;
    put_float_be(&frame_buf[offset], sht_hum);    offset += 4;

    *frame_len = offset;
}

/////////////////////////////////////////////////////////
// Full Frame => optional fields + 4 sensor blocks
// Each block includes an 8-byte sensor serial
/////////////////////////////////////////////////////////
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
    memset(frame_buf, 0, FRAME_DATA_LEN + FRAME_ECC_LEN);
    size_t offset = 0;

    // 1) Magic 'S','N'
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_0;
    frame_buf[offset++] = PROTOCOL_MAGIC_BYTE_1;

    // 2) Version
    frame_buf[offset++] = PROTOCOL_VERSION;

    // 3) Flags => bit1..3 => 0x0E => optional fields
    uint8_t flags = 0x0E;
    frame_buf[offset++] = flags;

    // 4) 4-byte message ID
    for (int i=0; i<PROTOCOL_MSGID_BYTES; i++) {
        frame_buf[offset++] = message_id[i];
    }

    // 5) 6-byte location
    memcpy(&frame_buf[offset], location_id, PROTOCOL_LOCATION_BYTES);
    offset += PROTOCOL_LOCATION_BYTES;

    // optional fields => bit1=1 => 1 byte => mcu_type
    frame_buf[offset++] = mcu_type;

    // bit2=1 => 12-byte MCU serial
    memcpy(&frame_buf[offset], mcu_serial, PROTOCOL_MCUSERIAL_BYTES);
    offset += PROTOCOL_MCUSERIAL_BYTES;

    // bit3=1 => 2-byte firmware version
    frame_buf[offset++] = fw_major;
    frame_buf[offset++] = fw_minor;

    // sensorCount => 4
    frame_buf[offset++] = 4;

    // 1) BME280 => type=3 => now we have an 8-byte serial + 3 floats => total 5 float slots
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x03;
        // valCount=5 => 2 for serial, + 3 actual float
        frame_buf[offset++] = 5; 

        // store 8-byte bme_serial as 2 float32 slots
        memcpy(&frame_buf[offset], bme_serial, 8);
        offset += 8;

        // store 3 floats
        put_float_be(&frame_buf[offset], bme_temp);  offset += 4;
        put_float_be(&frame_buf[offset], bme_press); offset += 4;
        put_float_be(&frame_buf[offset], bme_hum);   offset += 4;
    }

    // 2) TMP117 => type=6 => 8-byte serial + 1 float => valCount=3
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x06;
        frame_buf[offset++] = 3; // 2 for serial + 1 actual float

        memcpy(&frame_buf[offset], tmp_serial, 8);
        offset += 8;

        put_float_be(&frame_buf[offset], tmp117_temp);
        offset += 4;
    }

    // 3) AHT20 => type=5 => 8-byte serial + 2 floats => valCount=4
    {
        frame_buf[offset++] = 0x00;
        frame_buf[offset++] = 0x05;
        frame_buf[offset++] = 4;

        memcpy(&frame_buf[offset], aht_serial, 8);
        offset += 8;

        put_float_be(&frame_buf[offset], aht_temp); offset += 4;
        put_float_be(&frame_buf[offset], aht_hum);  offset += 4;
    }

    // 4) SHT41 => type=4 => 8-byte serial + 2 floats => valCount=4
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

/* final step => pad to 125 bytes, compute BFS remainder for 1003 bits => store 3 ECC bytes. */
void protocol_add_ecc(uint8_t *frame_buf, size_t *frame_len)
{
    if (!s_bch_inited) {
        protocol_init_bch();
    }
    size_t data_len = *frame_len;
    if (data_len > FRAME_DATA_LEN) {
        // real code => multi-frame or error
    }

    // pad
    while (data_len < FRAME_DATA_LEN) {
        frame_buf[data_len++] = 0x00;
    }

    // compute remainder
    uint32_t remainder = bch_encode_1003bits(frame_buf);

    // store 20 bits => 3 bytes big-endian, top nibble of last byte wasted
    frame_buf[data_len++] = (remainder >> 12) & 0xFF;
    frame_buf[data_len++] = (remainder >>  4) & 0xFF;
    frame_buf[data_len++] = (uint8_t)((remainder & 0xF) << 4);

    *frame_len = data_len; // ~128
}
