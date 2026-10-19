#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const char *model;  // "PMV-107J" or "Toyota"
    uint32_t id;
    float pressure_kpa;
    float temp_c;
    uint8_t status;
    int8_t battery_ok;  // 1 ok, 0 low, -1 not reported by this protocol
    bool rapid_change;  // sensor flagged a fast pressure change
    bool failed;        // sensor self-test failure
    bool inverted;      // decoded with inverted FSK polarity
} tpms_reading_t;

typedef struct {
    uint32_t rows;        // candidate rows long enough to hold a packet
    uint32_t preambles;   // preamble matches
    uint32_t crc_fail;    // preamble found but CRC failed
    uint32_t pressure_mismatch;
    uint32_t decoded;
} tpms_stats_t;

typedef void (*tpms_callback_t)(const tpms_reading_t *reading);

void tpms_decoder_init(tpms_callback_t cb);

// Feed one demodulator run: `level` held for `dur_us` microseconds.
void tpms_feed_run(int level, uint32_t dur_us);

// End of transmission (carrier lost / long silence): decode whatever is buffered.
void tpms_flush(void);

void tpms_set_raw_dump(bool on);
void tpms_get_stats(int proto, tpms_stats_t *out); // proto 0 = PMV-107J, 1 = Toyota
