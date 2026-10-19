#pragma once

// Tire state, sensor-to-wheel mapping, and the per-tire message output.
//
// Every decoded TPMS packet goes through tires_on_reading(). The module keeps
// a table of every sensor heard, the saved position mapping (NVS), and runs
// the button-driven mapping procedure:
//
//   1. Short-press the button: a baseline pressure is taken for every sensor
//      heard recently, and the node waits for the first position in
//      MAP_ORDER (feedback LED blinks the step number in blue). If at least
//      four sensors were heard, only those are eligible; otherwise sensors
//      heard for the first time get a baseline from their first packet.
//   2. Let a few PSI out of that tire. The first sensor whose pressure drops
//      at least MAP_DROP_PSI below its baseline is assigned to the position
//      (green flash), and the node moves on to the next position.
//   3. After the last position the mapping is saved to NVS. A press during
//      mapping cancels; a step that waits longer than MAP_STEP_TIMEOUT_S
//      aborts. Either way the previous mapping is kept.
//
// Output is JSON lines on stdout (see docs/message-schema.md).

#include <stdbool.h>
#include <stdint.h>

#include "tpms_decode.h"

typedef enum {
    TIRE_FL, // driver front
    TIRE_FR, // passenger front
    TIRE_RL, // driver rear
    TIRE_RR, // passenger rear
    TIRE_COUNT,
} tire_pos_t;

// Mapping order: driver front, driver rear, passenger rear, passenger front
// (a walk around the truck).
#define MAP_ORDER {TIRE_FL, TIRE_RL, TIRE_RR, TIRE_FR}
#define MAP_DROP_PSI          1.5f // PMV-107J resolves 0.363 PSI, so this is ~4 steps
#define MAP_STEP_TIMEOUT_S    600
#define MAP_CANDIDATE_AGE_S   900  // sensors heard this recently get a baseline at start
#define TIRE_STALE_S          600  // flag a mapped tire as stale after this long
#define SNAPSHOT_INTERVAL_S   10

void tires_init(void);
void tires_on_reading(const tpms_reading_t *r);
// Call about once a second: mapping step timeouts.
void tires_tick(void);

// Button: starts mapping when idle, cancels it when running.
void tires_button_press(void);
void tires_clear_mapping(void);

// Emit the {"type":"tpms"} snapshot of all four positions.
void tires_emit_snapshot(void);
// Human-readable sensor table / mapping state (console).
void tires_print_table(void);

const char *tire_pos_name(tire_pos_t pos);
