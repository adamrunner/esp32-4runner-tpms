// Host-side check of the TPMS slicers/decoders using synthetic FSK run trains.
// Build: cc -I../main -o host_test host_test.c ../main/tpms_decode.c && ./host_test
#include <stdio.h>
#include <stdlib.h>
#include "tpms_decode.h"

static int s_hits;
static void cb(const tpms_reading_t *r)
{
    printf("  decoded %s id=%08x %.1f kPa %.0f C status=0x%02x%s\n", r->model, r->id,
           r->pressure_kpa, r->temp_c, r->status, r->inverted ? " [inv]" : "");
    s_hits++;
}

static uint8_t crc8(const uint8_t *m, unsigned n, uint8_t poly, uint8_t init)
{
    uint8_t r = init;
    for (unsigned i = 0; i < n; i++) {
        r ^= m[i];
        for (int b = 0; b < 8; b++) r = (r & 0x80) ? (uint8_t)((r << 1) ^ poly) : (uint8_t)(r << 1);
    }
    return r;
}

// Raw PCM bits -> runs, with +/-jitter, optionally inverted.
static int raw[1024], nraw;
static void emit(int v) { raw[nraw++] = v; }
static void diffman(const uint8_t *bytes, unsigned first, unsigned nbits, int *lvl)
{
    for (unsigned i = first; i < first + nbits; i++) {
        int bit = (bytes[i / 8] >> (7 - i % 8)) & 1;
        *lvl = !*lvl; emit(*lvl);
        if (!bit) *lvl = !*lvl;
        emit(*lvl);
    }
}
static void play(unsigned width, int invert)
{
    tpms_feed_run(0, 5000); // leading noise gap
    int i = 0;
    while (i < nraw) {
        int j = i;
        while (j < nraw && raw[j] == raw[i]) j++;
        int jitter = (rand() % 9) - 4;
        tpms_feed_run(raw[i] ^ invert, (j - i) * width + jitter);
        i = j;
    }
    tpms_flush();
}

int main(void)
{
    tpms_decoder_init(cb);
    if (getenv("RAW")) tpms_set_raw_dump(true);
    for (int inv = 0; inv < 2; inv++) {
        // PMV-107J: 6 zero bits + 28-bit id + 6 flags + P + ~P + T + CRC(0x13)
        uint8_t b[9] = {0};
        uint32_t id = 0x0ABCDEF;
        unsigned p = 40 + 92, t = 40 + 21, flags = 0x08;
        uint64_t v = (uint64_t)id << 30 | (uint64_t)flags << 24 | p << 16 | (uint64_t)(p ^ 0xff) << 8 | t;
        for (int k = 0; k < 8; k++) b[k] = v >> (56 - 8 * k);
        b[8] = crc8(b, 8, 0x13, 0);
        nraw = 0;
        int pre[] = {0,1,0,1,0,1,1,1,1,1,0};
        for (unsigned k = 0; k < sizeof pre / sizeof *pre; k++) emit(pre[k]);
        int lvl = 0;
        diffman(b, 6, 66, &lvl);
        uint8_t trail[1] = {0};
        diffman(trail, 0, 3, &lvl);
        printf("PMV-107J synthetic (expect id=00abcdef, 228.2 kPa, 21 C), inverted=%d\n", inv);
        play(100, inv);

        // Toyota PMV-C210: 4B id, 1b state, P, T, 7b state, ~P, CRC(0x07, init 0x80)
        uint8_t c[9];
        unsigned tp = 4 * (33 + 7), tt = 40 + 18;
        c[0] = 0x12; c[1] = 0x34; c[2] = 0x56; c[3] = 0x78;
        c[4] = 0x80 | tp >> 1;
        c[5] = (tp & 1) << 7 | tt >> 1;
        c[6] = (tt & 1) << 7 | 0x05;
        c[7] = tp ^ 0xff;
        c[8] = crc8(c, 8, 0x07, 0x80);
        nraw = 0;
        const char *tpre = "01010101001111";
        for (const char *q = tpre; *q; q++) emit(*q - '0');
        lvl = 1;
        diffman(c, 0, 72, &lvl);
        diffman(trail, 0, 3, &lvl);
        printf("Toyota synthetic (expect id=12345678, 33.0 psi = 227.5 kPa, 18 C), inverted=%d\n", inv);
        play(52, inv);
    }
    printf("%s: %d/4 decoded\n", s_hits == 4 ? "PASS" : "FAIL", s_hits);
    return s_hits == 4 ? 0 : 1;
}
