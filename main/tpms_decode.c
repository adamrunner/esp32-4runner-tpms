// Toyota / Pacific Industrial TPMS decoders, ported from rtl_433:
//   src/devices/tpms_pmv107j.c  (PMV-107J, 315 MHz, 100 us half-bit)
//   src/devices/tpms_toyota.c   (PMV-C210, 52 us half-bit)
// Both are FSK, differential Manchester, CRC-8.

#include "tpms_decode.h"

#include <stdio.h>
#include <string.h>

#define ROW_MAX_BITS 512
// A steady tone longer than this many bit times can't be inside a packet
// (preambles hold one tone for at most ~5 bits), so it ends the row.
#define MAX_RUN_BITS 16

typedef struct slicer slicer_t;
typedef int (*decode_fn_t)(slicer_t *s, const uint8_t *bits, unsigned len, bool inverted);

struct slicer {
    const char *name;
    uint32_t width_us;    // one PCM bit (Manchester half-bit)
    unsigned min_bits;    // shortest row worth decoding
    decode_fn_t decode;
    uint8_t row[ROW_MAX_BITS / 8];
    unsigned nbits;
    tpms_stats_t stats;
};

static tpms_callback_t s_cb;
static bool s_raw_dump;

static inline int bit_at(const uint8_t *bits, unsigned pos)
{
    return (bits[pos >> 3] >> (7 - (pos & 7))) & 1;
}

static inline void set_bit(uint8_t *bits, unsigned pos, int v)
{
    if (v) {
        bits[pos >> 3] |= 0x80 >> (pos & 7);
    } else {
        bits[pos >> 3] &= ~(0x80 >> (pos & 7));
    }
}

static unsigned bit_search(const uint8_t *bits, unsigned len, unsigned start,
        const uint8_t *pattern, unsigned pattern_len)
{
    unsigned ipos = start, ppos = 0;
    while (ipos < len && ppos < pattern_len) {
        if (bit_at(bits, ipos) == bit_at(pattern, ppos)) {
            ppos++;
            ipos++;
            if (ppos == pattern_len) {
                return ipos - pattern_len;
            }
        } else {
            ipos -= ppos;
            ipos++;
            ppos = 0;
        }
    }
    return len;
}

// Port of rtl_433 bitbuffer_differential_manchester_decode(). Returns end position in input.
static unsigned diff_manchester_decode(const uint8_t *bits, unsigned len, unsigned start,
        uint8_t *out, unsigned *out_len, unsigned max)
{
    unsigned ipos = start;
    int bit1, bit2 = 0;
    *out_len = 0;
    memset(out, 0, (max + 7) / 8);

    if (max && len > start + max * 2) {
        len = start + max * 2;
    }

    while (ipos < len) {
        bit1 = bit_at(bits, ipos++);
        bit2 = bit_at(bits, ipos++);
        int bit3 = bit_at(bits, ipos);
        if (bit1 != bit2) {
            if (bit2 != bit3) {
                set_bit(out, (*out_len)++, 0);
            } else {
                bit2 = bit1;
                ipos -= 1;
                break;
            }
        } else {
            bit2 = 1 - bit1;
            ipos -= 2;
            break;
        }
    }

    while (ipos < len && *out_len < max) {
        bit1 = bit_at(bits, ipos++);
        if (bit1 == bit2) {
            break; // clock missing
        }
        bit2 = bit_at(bits, ipos++);
        set_bit(out, (*out_len)++, bit1 == bit2);
    }
    return ipos;
}

static uint8_t crc8(const uint8_t *msg, unsigned n, uint8_t poly, uint8_t init)
{
    uint8_t rem = init;
    for (unsigned i = 0; i < n; i++) {
        rem ^= msg[i];
        for (int b = 0; b < 8; b++) {
            rem = (rem & 0x80) ? (uint8_t)((rem << 1) ^ poly) : (uint8_t)(rem << 1);
        }
    }
    return rem;
}

static int decode_pmv107j(slicer_t *s, const uint8_t *bits, unsigned len, bool inverted)
{
    static const uint8_t preamble[1] = {0xf8}; // 6 bits: 11111 0
    unsigned bitpos = 0;
    int events = 0;

    while ((bitpos = bit_search(bits, len, bitpos, preamble, 6)) + 67 * 2 <= len) {
        s->stats.preambles++;
        uint8_t pkt[10];
        unsigned pkt_len;
        unsigned start = bitpos + 6;
        unsigned end = diff_manchester_decode(bits, len, start, pkt, &pkt_len, 70);
        bitpos += 2;
        if (end - start < 67 * 2) {
            continue;
        }

        // Realign: prepend 6 zero bits so the 66-bit payload fills 9 bytes.
        uint8_t b[9];
        b[0] = pkt[0] >> 6;
        for (int i = 0; i < 8; i++) {
            b[i + 1] = (uint8_t)(pkt[i] << 2 | pkt[i + 1] >> 6);
        }
        if (crc8(b, 8, 0x13, 0x00) != b[8]) {
            s->stats.crc_fail++;
            continue;
        }
        unsigned pressure1 = b[5], pressure2 = b[6] ^ 0xff;
        if (pressure1 != pressure2) {
            s->stats.pressure_mismatch++;
            continue;
        }
        tpms_reading_t r = {
            .model = "PMV-107J",
            .id = (uint32_t)b[0] << 26 | (uint32_t)b[1] << 18 | (uint32_t)b[2] << 10 | (uint32_t)b[3] << 2 | b[4] >> 6,
            .status = b[4] & 0x3f,
            .battery_ok = !(b[4] & 0x20),
            .rapid_change = b[4] & 0x02,
            .failed = b[4] & 0x01,
            .pressure_kpa = (pressure1 - 40.0f) * 2.48f,
            .temp_c = b[7] - 40.0f,
            .inverted = inverted,
        };
        s->stats.decoded++;
        events++;
        s_cb(&r);
    }
    return events;
}

static int decode_toyota(slicer_t *s, const uint8_t *bits, unsigned len, bool inverted)
{
    static const uint8_t preamble[2] = {0xa9, 0xe0}; // 12 bits
    unsigned bitpos = 0;
    int events = 0;

    while ((bitpos = bit_search(bits, len, bitpos, preamble, 12)) + 156 <= len) {
        s->stats.preambles++;
        uint8_t b[10];
        unsigned b_len;
        unsigned start = bitpos + 11;
        unsigned end = diff_manchester_decode(bits, len, start, b, &b_len, 80);
        bitpos += 2;
        if (end - start < 144) {
            continue;
        }
        if (crc8(b, 8, 0x07, 0x80) != b[8]) {
            s->stats.crc_fail++;
            continue;
        }
        unsigned pressure1 = (b[4] & 0x7f) << 1 | b[5] >> 7;
        unsigned pressure2 = b[7] ^ 0xff;
        if (pressure1 != pressure2) {
            s->stats.pressure_mismatch++;
            continue;
        }
        tpms_reading_t r = {
            .model = "Toyota",
            .id = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3],
            .status = (b[4] & 0x80) | (b[6] & 0x7f),
            .battery_ok = -1,
            .pressure_kpa = (pressure1 * 0.25f - 7.0f) * 6.894757f,
            .temp_c = ((b[5] & 0x7f) << 1 | b[6] >> 7) - 40.0f,
            .inverted = inverted,
        };
        s->stats.decoded++;
        events++;
        s_cb(&r);
    }
    return events;
}

static slicer_t s_slicers[] = {
    {.name = "PMV-107J", .width_us = 100, .min_bits = 6 + 67 * 2, .decode = decode_pmv107j},
    {.name = "Toyota",   .width_us = 52,  .min_bits = 156,        .decode = decode_toyota},
};
#define NUM_SLICERS (sizeof(s_slicers) / sizeof(s_slicers[0]))

static void dump_row(const slicer_t *s, const uint8_t *bits, unsigned len)
{
    printf("RAW %-8s {%3u} ", s->name, len);
    for (unsigned i = 0; i < (len + 7) / 8; i++) {
        printf("%02x", bits[i]);
    }
    printf("\n");
}

static void finish_row(slicer_t *s)
{
    if (s->nbits >= s->min_bits) {
        s->stats.rows++;
        if (s_raw_dump) {
            dump_row(s, s->row, s->nbits);
        }
        // FSK polarity depends on which tone the receiver calls "1"; try both.
        int found = s->decode(s, s->row, s->nbits, false);
        if (!found) {
            uint8_t inv[ROW_MAX_BITS / 8];
            for (unsigned i = 0; i < (s->nbits + 7) / 8; i++) {
                inv[i] = ~s->row[i];
            }
            s->decode(s, inv, s->nbits, true);
        }
    }
    s->nbits = 0;
}

static void slicer_feed(slicer_t *s, int level, uint32_t dur_us)
{
    unsigned n = (dur_us + s->width_us / 2) / s->width_us;
    if (n == 0 || n > MAX_RUN_BITS) {
        // Glitch shorter than half a bit, or a gap/carrier: treat as a break.
        finish_row(s);
        return;
    }
    for (unsigned i = 0; i < n; i++) {
        if (s->nbits >= ROW_MAX_BITS) {
            finish_row(s);
        }
        set_bit(s->row, s->nbits++, level);
    }
}

void tpms_decoder_init(tpms_callback_t cb)
{
    s_cb = cb;
}

void tpms_feed_run(int level, uint32_t dur_us)
{
    for (size_t i = 0; i < NUM_SLICERS; i++) {
        slicer_feed(&s_slicers[i], level, dur_us);
    }
}

void tpms_flush(void)
{
    for (size_t i = 0; i < NUM_SLICERS; i++) {
        finish_row(&s_slicers[i]);
    }
}

void tpms_set_raw_dump(bool on)
{
    s_raw_dump = on;
}

void tpms_get_stats(int proto, tpms_stats_t *out)
{
    *out = s_slicers[proto].stats;
}
