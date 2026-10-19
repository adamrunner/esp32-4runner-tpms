#include "feedback.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"

typedef enum {
    CMD_MODE,
    CMD_PACKET,
    CMD_ASSIGNED,
    CMD_DONE,
    CMD_FAILED,
} cmd_kind_t;

typedef struct {
    cmd_kind_t kind;
    feedback_mode_t mode;
    int step;
} cmd_t;

static const char *TAG = "feedback";

static led_strip_handle_t s_led;
static QueueHandle_t s_queue;
static feedback_mode_t s_mode = FB_IDLE;
static int s_step;

static void led(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) {
        return;
    }
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static void flash(uint8_t r, uint8_t g, uint8_t b, int count, int on_ms, int off_ms)
{
    for (int i = 0; i < count; i++) {
        led(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

static void handle(const cmd_t *c)
{
    switch (c->kind) {
    case CMD_MODE:
        s_mode = c->mode;
        s_step = c->step;
        break;
    case CMD_PACKET:
        flash(0, 6, 6, 1, 40, 0);
        break;
    case CMD_ASSIGNED:
        flash(0, 40, 0, 1, 1500, 300);
        break;
    case CMD_DONE:
        flash(0, 40, 0, 3, 200, 200);
        break;
    case CMD_FAILED:
        flash(40, 0, 0, 3, 200, 200);
        break;
    }
}

// Sleep up to `ms`, returning early (true) if a command arrived.
static bool wait_cmd(int ms)
{
    cmd_t c;
    if (xQueueReceive(s_queue, &c, pdMS_TO_TICKS(ms)) == pdTRUE) {
        handle(&c);
        return true;
    }
    return false;
}

static void feedback_task(void *arg)
{
    for (;;) {
        if (s_mode == FB_MAP_WAIT) {
            bool interrupted = false;
            for (int i = 0; i < s_step && !interrupted; i++) {
                led(0, 0, 40);
                interrupted = wait_cmd(250);
                led(0, 0, 0);
                interrupted = interrupted || wait_cmd(250);
            }
            if (!interrupted) {
                wait_cmd(1000);
            }
        } else {
            led(0, 0, 0);
            wait_cmd(1000);
        }
    }
}

static void post(cmd_kind_t kind, feedback_mode_t mode, int step)
{
    if (!s_queue) {
        return;
    }
    cmd_t c = {.kind = kind, .mode = mode, .step = step};
    xQueueSend(s_queue, &c, 0);
}

void feedback_init(int gpio)
{
    led_strip_config_t strip = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&strip, &rmt, &s_led) != ESP_OK) {
        ESP_LOGW(TAG, "no RGB LED on GPIO%d", gpio);
        s_led = NULL;
    }
    s_queue = xQueueCreate(8, sizeof(cmd_t));
    xTaskCreate(feedback_task, "feedback", 3072, NULL, 2, NULL);
}

void feedback_set_mode(feedback_mode_t mode, int step) { post(CMD_MODE, mode, step); }
void feedback_packet(void) { post(CMD_PACKET, 0, 0); }
void feedback_assigned(void) { post(CMD_ASSIGNED, 0, 0); }
void feedback_done(void) { post(CMD_DONE, 0, 0); }
void feedback_failed(void) { post(CMD_FAILED, 0, 0); }
