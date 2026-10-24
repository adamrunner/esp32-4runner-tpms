#pragma once

// Keeps the last LOGBUF_SIZE bytes of console output (printf JSON lines and
// ESP_LOG) in RAM for the web UI's log page, while still writing everything
// to the USB console.

#include <stddef.h>
#include <stdint.h>

#define LOGBUF_SIZE 32768 // power of two

// Call first thing in app_main, after the USB console driver is installed.
// Tasks created afterwards inherit the tee'd stdout; ESP_LOG from any task is
// routed through it too.
void logbuf_init(void);

// Copy console output starting at byte offset *pos into out. If *pos is older
// than the buffer holds, it is moved up to the oldest byte kept. Returns the
// number of bytes copied and advances *pos past them.
size_t logbuf_read(uint32_t *pos, char *out, size_t len);
