/*
 * bch.c - Minimal BCH(1023,1003) 2-bit ECC library for encoding only
 * Derived from Linux bch.c (GPLv2). Original author: Ivan Djelic <ivan.djelic@parrot.com>
 *
 * License: GPL-2.0
 */

#include "bch.h"
#include <stdlib.h>   /* malloc/free */
#include <string.h>   /* memset/memcpy */
#include <stdio.h>    /* optional, if you want debug prints */

/* 
 * For BFS, we use polynomial:
 *   G(x) = x^20 + x^17 + x^5 + x^2 + x + 1 = 0x10031
 * This yields 20 parity bits. We'll store remainder in bc->remainder (top 20 bits).
 */

/* Hard-coded polynomial for 2-bit correction in GF(2^10). 
   This is not the full generator polynomial for decoding—just an example
   used in naive BFS from the Linux snippet. */
#define BCH_POLY_20  (0x10031u)

/* top 20 bits => we mask 0xFFFFF after each shift. */
#define BCH_MASK_20  (0xFFFFFu)

/* bit reverse for one byte if needed */
static inline unsigned char bitrev8(unsigned char x)
{
    /* standard bit-reverse approach */
    x = ((x & 0xf0) >> 4) | ((x & 0x0f) << 4);
    x = ((x & 0xcc) >> 2) | ((x & 0x33) << 2);
    x = ((x & 0xaa) >> 1) | ((x & 0x55) << 1);
    return x;
}

/* Helper to conditionally bit-reverse a byte */
static inline unsigned char swap_bits_if(struct bch_control *bc, unsigned char c)
{
    return bc->swap_bits ? bitrev8(c) : c;
}

/****************************************************************************
 * bch_init_2bit
 ****************************************************************************/
struct bch_control *bch_init_2bit(bool swap_bits)
{
    /* For 2-bit correct in GF(2^10) => we only do BFS encoding polynomial approach. */
    struct bch_control *bc = (struct bch_control *)malloc(sizeof(*bc));
    if (!bc) {
        return NULL;
    }
    bc->swap_bits = swap_bits;
    bc->remainder = 0; /* start with remainder=0 => means ECC=0 */
    return bc;
}

/****************************************************************************
 * bch_free
 ****************************************************************************/
void bch_free(struct bch_control *bc)
{
    if (bc) free(bc);
}

/****************************************************************************
 * bch_encode
 *
 * BFS approach:
 *  - Treat top 20 bits of bc->remainder as the current partial ECC state.
 *  - For each input byte:
 *     1) read top 8 bits from remainder
 *     2) XOR them with the (bit-reversed?) data byte
 *     3) then shift remainder left by 8 bits, injecting that XOR as new bits
 *     4) for each of those 8 shifts, if the new top bit is 1 => remainder ^= BCH_POLY_20
 *     5) keep remainder masked to 20 bits
 *  - At end, we store the final 20 bits into ecc[3], big-endian (with optional bit reverse)
 ****************************************************************************/
void bch_encode(struct bch_control *bc,
                const uint8_t *data,
                unsigned int data_len,
                uint8_t ecc[3])
{
    /* 1) Convert the existing ecc[3] -> 20-bit remainder. 
       This allows incremental usage (like partial streaming). 
       If the user zeroed ecc at first usage, we start from remainder=0. */

    /* read big-endian 20 bits from ecc => store in bc->remainder. 
       We'll do 3 bytes => top nibble of the last => BFS approach. */

    unsigned int old_rem = 0;
    {
        /* interpret ecc[0..2] big-endian => top 20 bits => ignoring the lower 4 bits in the last nibble. */
        unsigned char b0 = swap_bits_if(bc, ecc[0]);
        unsigned char b1 = swap_bits_if(bc, ecc[1]);
        unsigned char b2 = swap_bits_if(bc, ecc[2]);

        old_rem = ((unsigned int)b0 <<12)
                | ((unsigned int)b1 << 4)
                | ((unsigned int)(b2 >>4));
        /* now old_rem is up to 20 bits. The bottom 4 bits of b2 are ignored. */
    }

    bc->remainder = old_rem & BCH_MASK_20;

    /* 2) For each input byte => BFS shift x8 => snippet approach. */
    for (unsigned int i=0; i< data_len; i++) {
        unsigned char inbyte = swap_bits_if(bc, data[i]);
        /* XOR top 8 bits of remainder with inbyte => then we do 8 BFS steps. 
           top8 = remainder >> 12 => that yields 8 bits from a 20-bit remainder */
        unsigned int top8 = (bc->remainder >> 12) & 0xFF;
        unsigned int mix = top8 ^ inbyte;

        /* shift remainder left 8 bits, BFS approach: do each bit or do them in a single loop. 
           We'll do a single bit approach for clarity. */
        for (int bit=7; bit>=0; bit--) {
            /* get the top bit => if it's 1 => remainder ^= polynomial */
            unsigned int new_top = (bc->remainder & (1u <<19))?1:0; /* check bit #19 => top of 20 bits */
            /* shift left by 1 */
            bc->remainder = (bc->remainder <<1) & BCH_MASK_20;
            /* also incorporate the next bit from mix => bit #bit */
            if ((mix>>bit)&1) new_top ^=1;
            /* if new_top=1 => remainder ^= 0x10031 => BFS polynomial */
            if (new_top) {
                bc->remainder ^= BCH_POLY_20;
            }
        }
        bc->remainder &= BCH_MASK_20;
    }

    /* 3) At the end, bc->remainder holds the final 20-bit ECC state. Store back to ecc[] big-endian. 
       top nibble of last byte is used, bottom nibble is 0. */
    {
        unsigned int final_rem = bc->remainder & BCH_MASK_20;
        /* top 20 bits => b0 => bits [19..12], b1 => bits [11..4], b2 => bits [3..0 <<4]. */

        unsigned char b0 = (final_rem >>12) & 0xFF;
        unsigned char b1 = (final_rem >> 4) & 0xFF;
        unsigned char b2 = (final_rem & 0xF) <<4; /* top nibble => low nibble=0 */

        ecc[0] = swap_bits_if(bc, b0);
        ecc[1] = swap_bits_if(bc, b1);
        ecc[2] = swap_bits_if(bc, b2);
    }
}
