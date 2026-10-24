// 2018 4Runner TPMS receiver: CC1101 in async 2-FSK RX at 315 MHz.
// GDO0 carries the demodulated bitstream; we timestamp every edge in an ISR
// and slice the run lengths into bits for the rtl_433-derived decoders.

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cc1101.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "feedback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "nvs_flash.h"
#include "soc/gpio_struct.h"
#include "tires.h"
#include "tpms_decode.h"
#include "tx_test.h"
#include "web.h"

#define PIN_SCK  12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10
#define PIN_GDO0 4
#define PIN_GDO2 5
#define PIN_BUTTON 0 // DevKit BOOT button, active low
#define PIN_LED    48 // DevKitC-1 v1.0 onboard WS2812 (v1.1 boards use GPIO38)

// The truck's sensors center on ~314.99 MHz (RTL-SDR Blog V4, TCXO). This
// CC1101's crystal runs ~20 kHz low (measured via bench TX), so tune ~+20 kHz.
#define DEFAULT_FREQ_HZ 315010000
#define DEFAULT_CS_THR  0

#define RING_SIZE 8192 // power of two

static const char *TAG = "tpms";

// ISR -> capture task. Each entry: bit31 = level after the edge, bits 0..30 = timestamp (us).
static uint32_t s_ring[RING_SIZE];
static volatile uint32_t s_head, s_tail;
static volatile uint32_t s_overflows, s_edges;
// Carrier-sense gating is off by default: at full gain the CC1101 reports CS
// constantly on the bench (~-82 dBm noise floor), and the decoders reject the
// ~10k noise edges/s on their own. Toggle with 'g' to experiment.
static volatile bool s_gate = false;

static uint32_t s_freq_hz = DEFAULT_FREQ_HZ;
static int s_cs_thr = DEFAULT_CS_THR;
static bool s_raw;

static void IRAM_ATTR gdo0_isr(void *arg)
{
    // Optionally only record while the CC1101 reports carrier sense.
    if (s_gate && !gpio_ll_get_level(&GPIO, PIN_GDO2)) {
        return;
    }
    uint32_t now = (uint32_t)esp_timer_get_time() & 0x7FFFFFFF;
    uint32_t level = gpio_ll_get_level(&GPIO, PIN_GDO0);
    uint32_t head = s_head;
    uint32_t next = (head + 1) & (RING_SIZE - 1);
    s_edges++;
    if (next == s_tail) {
        s_overflows++;
        return;
    }
    s_ring[head] = now | (level << 31);
    s_head = next;
}

static void capture_task(void *arg)
{
    // Installed from here so the edge ISR lands on core 1, away from Wi-Fi on core 0.
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_GDO0, gdo0_isr, NULL));

    uint32_t last_ts = 0;
    int last_level = 0;
    bool have_last = false, pending = false;

    for (;;) {
        while (s_tail != s_head) {
            uint32_t e = s_ring[s_tail];
            s_tail = (s_tail + 1) & (RING_SIZE - 1);
            uint32_t ts = e & 0x7FFFFFFF;
            int level = e >> 31;
            if (have_last) {
                uint32_t dur = (ts - last_ts) & 0x7FFFFFFF;
                tpms_feed_run(last_level, dur);
                pending = true;
            }
            last_ts = ts;
            last_level = level;
            have_last = true;
        }
        if (pending) {
            uint32_t now = (uint32_t)esp_timer_get_time() & 0x7FFFFFFF;
            if (((now - last_ts) & 0x7FFFFFFF) > 2000) {
                tpms_flush();
                pending = false;
            }
        }
        vTaskDelay(1);
    }
}

static void print_status(void)
{
    static const char *names[] = {"PMV-107J", "Toyota"};
    ESP_LOGI(TAG, "freq=%" PRIu32 " Hz  cs_thr=%d  gate=%s  json=%s  rssi=%.1f dBm  CS=%d  edges=%" PRIu32 "  overflows=%" PRIu32,
             s_freq_hz, s_cs_thr, s_gate ? "on" : "off", tires_json_output() ? "on" : "off", cc1101_rssi_dbm(), gpio_get_level(PIN_GDO2),
             s_edges, s_overflows);
    for (int p = 0; p < 2; p++) {
        tpms_stats_t st;
        tpms_get_stats(p, &st);
        ESP_LOGI(TAG, "  %-8s rows=%" PRIu32 " preambles=%" PRIu32 " crc_fail=%" PRIu32 " p_mismatch=%" PRIu32 " decoded=%" PRIu32,
                 names[p], st.rows, st.preambles, st.crc_fail, st.pressure_mismatch, st.decoded);
    }
    tires_print_table();
}

static void tick_task(void *arg)
{
    for (int sec = 1;; sec++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tires_tick();
        if (sec % SNAPSHOT_INTERVAL_S == 0) {
            tires_emit_snapshot();
        }
    }
}

static volatile bool s_fake_press; // console 'b': press the button from the button task

static void button_task(void *arg)
{
    int held_ms = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (gpio_get_level(PIN_BUTTON) == 0) {
            held_ms += 20;
        } else {
            if (held_ms >= 60 || s_fake_press) { // debounced press, acted on at release
                s_fake_press = false;
                tires_button_press();
                ESP_LOGI(TAG, "button press, stack free %u bytes", (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
            held_ms = 0;
        }
    }
}

static void print_help(void)
{
    printf("\nCommands:\n"
           "  s            status / sensor table\n"
           "  m            start/cancel tire mapping (same as BOOT button)\n"
           "  b            simulate a BOOT button press\n"
           "  c            clear saved mapping\n"
           "  l<pos>       learn one wheel: next sensor to change 1.5 PSI either way (e.g. lFL; l cancels)\n"
           "  a<pos> <id>  assign a sensor to a wheel by hand (e.g. aFL 01069354; id 0 = unassign)\n"
           "  i<id> <psi>  inject a fake PMV-107J reading (hex id), for bench-testing mapping\n"
           "  t<n>         carrier-sense threshold, -7..7 dB (e.g. t3)\n"
           "  g            toggle carrier-sense gating (off = capture everything)\n"
           "  r            toggle raw row dump\n"
           "  j            toggle JSON output (tire/tpms/map lines)\n"
           "  f<hz>        set frequency (e.g. f314980000)\n"
           "  d            pin/radio diagnostics\n"
           "  w<addr> <v>  raw register write, hex (e.g. w02 3f)\n"
           "  x[n]         bench TX: n x synthetic PMV-107J + Toyota packets at ~-30 dBm\n"
           "  h            this help\n\n");
}

static void handle_command(char *line)
{
    switch (line[0]) {
    case 's':
        print_status();
        break;
    case 't':
        s_cs_thr = atoi(line + 1);
        cc1101_set_cs_threshold(s_cs_thr);
        ESP_LOGI(TAG, "cs threshold %d", s_cs_thr);
        break;
    case 'g':
        s_gate = !s_gate;
        ESP_LOGI(TAG, "gating %s", s_gate ? "on" : "off");
        break;
    case 'r':
        s_raw = !s_raw;
        tpms_set_raw_dump(s_raw);
        ESP_LOGI(TAG, "raw dump %s", s_raw ? "on" : "off");
        break;
    case 'j':
        tires_set_json_output(!tires_json_output());
        ESP_LOGI(TAG, "json output %s", tires_json_output() ? "on" : "off");
        break;
    case 'f':
        s_freq_hz = strtoul(line + 1, NULL, 10);
        cc1101_strobe(CC1101_SIDLE);
        cc1101_set_freq(s_freq_hz);
        cc1101_start_rx();
        ESP_LOGI(TAG, "frequency %" PRIu32 " Hz", s_freq_hz);
        break;
    case 'd': {
        // Poll the GDO pins directly, independent of the ISR.
        int toggles = 0, cs_high = 0, last = gpio_get_level(PIN_GDO0);
        int64_t until = esp_timer_get_time() + 100000;
        while (esp_timer_get_time() < until) {
            int lvl = gpio_get_level(PIN_GDO0);
            toggles += lvl != last;
            last = lvl;
            cs_high += gpio_get_level(PIN_GDO2);
        }
        ESP_LOGI(TAG, "GDO0 level=%d GDO2 level=%d", gpio_get_level(PIN_GDO0), gpio_get_level(PIN_GDO2));
        ESP_LOGI(TAG, "MARCSTATE=0x%02x GDO0 toggles/100ms=%d GDO2(CS) high samples=%d IOCFG0=0x%02x PKTCTRL0=0x%02x",
                 cc1101_read_status(CC1101_MARCSTATE) & 0x1F, toggles, cs_high,
                 cc1101_read_reg(CC1101_IOCFG0), cc1101_read_reg(CC1101_PKTCTRL0));
        break;
    }
    case 'w': {
        // w<addr> <val>, both hex: raw register write
        char *end;
        unsigned addr = strtoul(line + 1, &end, 16);
        unsigned val = strtoul(end, NULL, 16);
        cc1101_write_reg(addr, val);
        ESP_LOGI(TAG, "reg 0x%02x <- 0x%02x (readback 0x%02x)", addr, val, cc1101_read_reg(addr));
        break;
    }
    case 'u': {
        static bool pullup;
        pullup = !pullup;
        gpio_set_pull_mode(PIN_GDO0, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
        ESP_LOGI(TAG, "GDO0 pull-up %s", pullup ? "on" : "off");
        break;
    }
    case 'x':
        tx_test_send(PIN_GDO0, line[1] ? atoi(line + 1) : 3);
        break;
    case 'm':
        tires_button_press();
        break;
    case 'b':
        s_fake_press = true;
        break;
    case 'c':
        tires_clear_mapping();
        break;
    case 'a': {
        tire_pos_t pos;
        if (!tire_pos_from_name(line + 1, &pos)) {
            ESP_LOGW(TAG, "usage: a<FL|FR|RL|RR> <hex id>");
            break;
        }
        uint32_t id = strtoul(line + 3, NULL, 16);
        if (!tires_assign(pos, id)) {
            ESP_LOGW(TAG, "mapping run in progress; cancel it first (m)");
        }
        break;
    }
    case 'l': {
        tire_pos_t pos;
        if (!tire_pos_from_name(line + 1, &pos)) {
            tires_learn_cancel();
        } else if (!tires_learn_start(pos)) {
            ESP_LOGW(TAG, "mapping run in progress; cancel it first (m)");
        }
        break;
    }
    case 'i': {
        char *end;
        tpms_reading_t r = {
            .model = "PMV-107J",
            .id = strtoul(line + 1, &end, 16),
            .temp_c = 20,
            .battery_ok = 1,
        };
        r.pressure_kpa = strtof(end, NULL) * 6.894757f;
        tires_on_reading(&r);
        break;
    }
    case 'h':
    case '?':
        print_help();
        break;
    default:
        break;
    }
}

static void console_task(void *arg)
{
    char line[32];
    size_t len = 0;
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            line[len] = 0;
            if (len) {
                handle_command(line);
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    usb_serial_jtag_vfs_use_driver();

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    feedback_init(PIN_LED);
    tires_init();
    tpms_decoder_init(tires_on_reading);

    cc1101_pins_t pins = {.pin_sck = PIN_SCK, .pin_mosi = PIN_MOSI, .pin_miso = PIN_MISO, .pin_cs = PIN_CS};
    esp_err_t err = cc1101_init(&pins);
    while (err != ESP_OK) {
        ESP_LOGE(TAG, "CC1101 not detected; check wiring (SCK=%d MOSI=%d MISO=%d CS=%d). Retrying...",
                 PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS);
        vTaskDelay(pdMS_TO_TICKS(3000));
        err = cc1101_reset();
    }

    cc1101_config_async_fsk_rx(s_freq_hz);
    cc1101_set_cs_threshold(s_cs_thr);
    cc1101_start_rx();
    ESP_LOGI(TAG, "RX at %" PRIu32 " Hz, MARCSTATE=0x%02x", s_freq_hz, cc1101_read_status(CC1101_MARCSTATE) & 0x1F);

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_GDO0) | (1ULL << PIN_GDO2),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    gpio_config_t button = {
        .pin_bit_mask = 1ULL << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));
    gpio_set_intr_type(PIN_GDO0, GPIO_INTR_ANYEDGE);
    gpio_intr_enable(PIN_GDO0);

    xTaskCreatePinnedToCore(capture_task, "capture", 6144, NULL, 10, NULL, 1);
    xTaskCreate(tick_task, "tick", 4096, NULL, 3, NULL);
    xTaskCreate(button_task, "button", 4096, NULL, 3, NULL); // printf needs ~2 KB of stack
    xTaskCreate(console_task, "console", 4096, NULL, 4, NULL);

    web_start();

    print_help();
}
