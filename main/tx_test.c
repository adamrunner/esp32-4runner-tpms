#include "tx_test.h"

#include <string.h>

#include "cc1101.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CC1101_PATABLE 0x3E

static const char *TAG = "tx_test";

static uint8_t s_raw[400];
static unsigned s_nraw;

static uint8_t crc8(const uint8_t *m, unsigned n, uint8_t poly, uint8_t init)
{
    uint8_t r = init;
    for (unsigned i = 0; i < n; i++) {
        r ^= m[i];
        for (int b = 0; b < 8; b++) {
            r = (r & 0x80) ? (uint8_t)((r << 1) ^ poly) : (uint8_t)(r << 1);
        }
    }
    return r;
}

static void emit(int v)
{
    if (s_nraw < sizeof(s_raw)) {
        s_raw[s_nraw++] = v;
    }
}

static void emit_str(const char *bits)
{
    for (; *bits; bits++) {
        emit(*bits - '0');
    }
}

// Differential Manchester: transition at every bit start, extra mid-bit transition for 0.
static int diffman(const uint8_t *bytes, unsigned first, unsigned nbits, int lvl)
{
    for (unsigned i = first; i < first + nbits; i++) {
        int bit = (bytes[i / 8] >> (7 - i % 8)) & 1;
        lvl = !lvl;
        emit(lvl);
        if (!bit) {
            lvl = !lvl;
        }
        emit(lvl);
    }
    return lvl;
}

static void build_pmv107j(uint32_t id, float psi, int temp_c)
{
    uint8_t b[9] = {0};
    unsigned p = (unsigned)(psi / 0.363f + 40.0f + 0.5f);
    unsigned t = temp_c + 40;
    unsigned flags = 0x08; // repeat counter = 1
    uint64_t v = (uint64_t)(id & 0x0FFFFFFF) << 30 | (uint64_t)flags << 24 | p << 16 | (uint64_t)(p ^ 0xff) << 8 | t;
    for (int k = 0; k < 8; k++) {
        b[k] = v >> (56 - 8 * k);
    }
    b[8] = crc8(b, 8, 0x13, 0);
    static const uint8_t trail = 0;

    s_nraw = 0;
    emit_str("0101010101010101011111110");
    int lvl = diffman(b, 6, 66, 0);
    diffman(&trail, 0, 3, lvl);
}

static void build_toyota(uint32_t id, float psi, int temp_c)
{
    uint8_t c[9];
    unsigned p = (unsigned)((psi + 7.0f) * 4.0f + 0.5f);
    unsigned t = temp_c + 40;
    c[0] = id >> 24;
    c[1] = id >> 16;
    c[2] = id >> 8;
    c[3] = id;
    c[4] = 0x80 | p >> 1;
    c[5] = (p & 1) << 7 | t >> 1;
    c[6] = (t & 1) << 7;
    c[7] = p ^ 0xff;
    c[8] = crc8(c, 8, 0x07, 0x80);
    static const uint8_t trail = 0;

    s_nraw = 0;
    emit_str("0101010101010101001111");
    int lvl = diffman(c, 0, 72, 1);
    diffman(&trail, 0, 3, lvl);
}

// Key the carrier only for the packet, like a real sensor. A carrier held on
// between packets makes rtl_433 see one long FSK package and pick the idle
// tone as F1, which breaks the PMV-107J preamble.
static void play(int pin, uint32_t width_us)
{
    gpio_set_level(pin, s_raw[0]);
    cc1101_strobe(0x35); // STX (autocal from IDLE takes ~0.8 ms)
    vTaskDelay(pdMS_TO_TICKS(2));
    int64_t t = esp_timer_get_time() + 50;
    portDISABLE_INTERRUPTS();
    for (unsigned i = 0; i < s_nraw; i++) {
        gpio_set_level(pin, s_raw[i]);
        t += width_us;
        while (esp_timer_get_time() < t) {
        }
    }
    portENABLE_INTERRUPTS();
    cc1101_strobe(CC1101_SIDLE);
    gpio_set_level(pin, 0);
}

void tx_test_send(int pin_gdo0, int repeats)
{
    gpio_intr_disable(pin_gdo0);
    cc1101_strobe(CC1101_SIDLE);
    cc1101_write_reg(CC1101_IOCFG0, 0x2E);     // GDO0 hi-Z so it can act as TX data input
    cc1101_write_reg(CC1101_PATABLE, 0x12);    // ~-30 dBm
    gpio_set_direction(pin_gdo0, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_gdo0, 0);

    for (int r = 0; r < repeats; r++) {
        build_pmv107j(0x0ABCDEF, 32.0f, 21);
        play(pin_gdo0, 100);
        vTaskDelay(pdMS_TO_TICKS(150));
        build_toyota(0x12345678, 33.0f, 18);
        play(pin_gdo0, 52);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    cc1101_strobe(CC1101_SIDLE);
    gpio_set_direction(pin_gdo0, GPIO_MODE_INPUT);
    cc1101_write_reg(CC1101_IOCFG0, 0x0D);
    cc1101_start_rx();
    gpio_intr_enable(pin_gdo0);
    ESP_LOGI(TAG, "sent %d x (PMV-107J id 0abcdef 32 psi, Toyota id 12345678 33 psi)", repeats);
}
