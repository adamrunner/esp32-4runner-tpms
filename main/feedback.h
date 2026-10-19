#pragma once

// Single RGB LED feedback (the DevKitC-1's onboard WS2812).

#include <stdint.h>

typedef enum {
    FB_IDLE,     // off
    FB_MAP_WAIT, // blue: blink `step` times, pause, repeat
} feedback_mode_t;

void feedback_init(int gpio);
void feedback_set_mode(feedback_mode_t mode, int step);

// One-shot patterns, played over the current mode.
void feedback_packet(void);   // brief dim cyan blip: a packet was decoded
void feedback_assigned(void); // long green: a tire was mapped
void feedback_done(void);     // three green flashes: mapping saved
void feedback_failed(void);   // three red flashes: cancelled / timed out
