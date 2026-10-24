#include "logbuf.h"

#include <stdarg.h>
#include <stdio.h>
#include <sys/reent.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static char s_ring[LOGBUF_SIZE];
static uint32_t s_total; // bytes ever written; s_ring holds the last LOGBUF_SIZE
static SemaphoreHandle_t s_lock; // a mutex, not a spinlock: keep interrupts on for the GDO0 ISR
static FILE *s_console; // the original stdout
static FILE *s_tee;

static int tee_write(void *cookie, const char *data, int len)
{
    fwrite(data, 1, len, s_console);
    fflush(s_console);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < len; i++) {
        s_ring[(s_total + i) & (LOGBUF_SIZE - 1)] = data[i];
    }
    s_total += len;
    xSemaphoreGive(s_lock);
    return len;
}

static int log_vprintf(const char *fmt, va_list ap)
{
    // Not stdout: tasks started before logbuf_init() still have the old one.
    return vfprintf(s_tee, fmt, ap);
}

void logbuf_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_console = stdout;
    s_tee = funopen(NULL, NULL, tee_write, NULL, NULL);
    setvbuf(s_tee, NULL, _IOLBF, 256);
    _GLOBAL_REENT->_stdout = s_tee; // inherited by tasks created from now on
    stdout = s_tee;                 // this task
    esp_log_set_vprintf(log_vprintf);
}

size_t logbuf_read(uint32_t *pos, char *out, size_t len)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    // Too old, or from before a reboot: start at the oldest byte kept.
    if (*pos > s_total || s_total - *pos > LOGBUF_SIZE) {
        *pos = s_total > LOGBUF_SIZE ? s_total - LOGBUF_SIZE : 0;
    }
    size_t n = s_total - *pos;
    if (n > len) {
        n = len;
    }
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(*pos + i) & (LOGBUF_SIZE - 1)];
    }
    *pos += n;
    xSemaphoreGive(s_lock);
    return n;
}
