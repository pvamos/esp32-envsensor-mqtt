#ifndef BCH_H
#define BCH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Minimal context for BCH(1023,1003) 2-bit ECC (only encoding).
 *  - ECC is 20 bits (3 bytes).
 *  - Data up to 125 bytes (1000 bits).
 *  - If swap_bits=true, each data/ECC byte is bit-reversed upon process.
 */
struct bch_control {
    bool swap_bits;      /* whether to bit-reverse each byte */
    uint32_t remainder;  /* 20-bit partial remainder (top bits in a 32-bit) */
};

/**
 * @brief Allocate a BCH(1023,1003) 2-bit context for encoding only.
 *
 * @param swap_bits  If true, the code will bit-reverse each byte in data/ECC.
 * @return pointer to newly allocated bch_control, or NULL on error
 */
struct bch_control *bch_init_2bit(bool swap_bits);

/**
 * @brief Free the BCH context.
 *
 * @param bc pointer returned by bch_init_2bit()
 */
void bch_free(struct bch_control *bc);

/**
 * @brief Encode data up to 125 bytes => produce or update 3 bytes of ECC.
 *
 * The ECC 3 bytes must be zeroed on first usage or contain partial remainder if
 * doing incremental encoding. This function uses a BFS approach with polynomial
 * x^20 + x^17 + x^5 + x^2 + x + 1 (0x10031).
 *
 * @param bc     BCH context
 * @param data   pointer to data array
 * @param data_len length in bytes (<=125)
 * @param ecc    3-byte array (in/out). Must be zeroed before first usage.
 */
void bch_encode(struct bch_control *bc,
                const uint8_t *data,
                unsigned int data_len,
                uint8_t ecc[3]);

#ifdef __cplusplus
}
#endif

#endif /* BCH_H */
