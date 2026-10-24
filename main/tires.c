#include "tires.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "feedback.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#define MAX_SENSORS 16
#define KPA_PER_PSI 6.894757f
#define US_PER_S    1000000LL

typedef struct {
    uint32_t id;
    const char *model;
    float kpa;
    float temp_c;
    int8_t battery_ok;
    bool rapid_change;
    bool failed;
    int64_t last_seen_us;
    uint32_t count;
    // Mapping run: highest pressure seen since the run started.
    // Learn: pressure when the learn started (or at the first packet after).
    float baseline_kpa;
    bool has_baseline;
} sensor_t;

static const char *TAG = "tires";
static const tire_pos_t s_order[TIRE_COUNT] = MAP_ORDER;

static SemaphoreHandle_t s_lock;
static sensor_t s_sensors[MAX_SENSORS];
static uint32_t s_map[TIRE_COUNT]; // sensor id per position, 0 = unmapped

static bool s_mapping;
static int s_step;                      // index into s_order
static uint32_t s_pending[TIRE_COUNT];  // ids assigned so far in this run
static int64_t s_step_started_us;
static bool s_candidates_locked; // all four heard before start: ignore newcomers
static bool s_json = true;       // console 'j': JSON lines on/off

static int s_learn_pos = -1; // position being learned on its own, -1 = none
static int64_t s_learn_started_us;

const char *tire_pos_name(tire_pos_t pos)
{
    static const char *names[TIRE_COUNT] = {"FL", "FR", "RL", "RR"};
    return pos < TIRE_COUNT ? names[pos] : "?";
}

static void load_mapping(void)
{
    nvs_handle_t h;
    size_t len = sizeof(s_map);
    if (nvs_open("tpms", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_blob(h, "map", s_map, &len) != ESP_OK || len != sizeof(s_map)) {
            memset(s_map, 0, sizeof(s_map));
        }
        nvs_close(h);
    }
}

static void save_mapping(void)
{
    nvs_handle_t h;
    if (nvs_open("tpms", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed; mapping not saved");
        return;
    }
    nvs_set_blob(h, "map", s_map, sizeof(s_map));
    nvs_commit(h);
    nvs_close(h);
}

static sensor_t *find_sensor(uint32_t id)
{
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (s_sensors[i].count && s_sensors[i].id == id) {
            return &s_sensors[i];
        }
    }
    return NULL;
}

static int position_of(uint32_t id)
{
    for (int p = 0; p < TIRE_COUNT; p++) {
        if (s_map[p] == id) {
            return p;
        }
    }
    return -1;
}

// ---- JSON output -----------------------------------------------------------

static void print_tire_fields(int pos, uint32_t id, const sensor_t *s, int64_t now)
{
    if (pos >= 0) {
        printf("\"pos\":\"%s\",", tire_pos_name(pos));
    } else {
        printf("\"pos\":null,");
    }
    if (id) {
        printf("\"id\":\"%08" PRIx32 "\",", id);
    } else {
        printf("\"id\":null,");
    }
    if (!s) {
        printf("\"model\":null,\"psi\":null,\"kpa\":null,\"temp_c\":null,\"battery_ok\":null,"
               "\"rapid_change\":null,\"failed\":null,\"age_s\":null,\"stale\":true");
        return;
    }
    int64_t age_s = (now - s->last_seen_us) / US_PER_S;
    printf("\"model\":\"%s\",\"psi\":%.1f,\"kpa\":%.1f,\"temp_c\":%.0f,", s->model,
           s->kpa / KPA_PER_PSI, s->kpa, s->temp_c);
    printf("\"battery_ok\":%s,", s->battery_ok < 0 ? "null" : s->battery_ok ? "true" : "false");
    printf("\"rapid_change\":%s,\"failed\":%s,\"age_s\":%lld,\"stale\":%s",
           s->rapid_change ? "true" : "false", s->failed ? "true" : "false", (long long)age_s,
           age_s > TIRE_STALE_S ? "true" : "false");
}

static void emit_tire(const sensor_t *s, int64_t now)
{
    if (!s_json) {
        return;
    }
    printf("{\"type\":\"tire\",");
    print_tire_fields(position_of(s->id), s->id, s, now);
    printf("}\n");
}

static void emit_map_event(const char *event, const char *extra)
{
    if (!s_json) {
        // Still show mapping progress (timeouts especially) on the console.
        ESP_LOGI(TAG, "map %s%s%s", event, extra ? " " : "", extra ? extra : "");
        return;
    }
    printf("{\"type\":\"map\",\"event\":\"%s\"%s%s}\n", event, extra ? "," : "", extra ? extra : "");
}

static void emit_map_wait(void)
{
    char extra[64];
    snprintf(extra, sizeof(extra), "\"step\":%d,\"pos\":\"%s\"", s_step + 1,
             tire_pos_name(s_order[s_step]));
    emit_map_event("wait", extra);
}

void tires_set_json_output(bool on)
{
    s_json = on;
}

bool tires_json_output(void)
{
    return s_json;
}

void tires_emit_snapshot(void)
{
    if (!s_json) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    bool mapped = true;
    printf("{\"type\":\"tpms\",\"uptime_s\":%lld,\"mapping\":%s,\"tires\":[",
           (long long)(now / US_PER_S), s_mapping ? "true" : "false");
    for (int p = 0; p < TIRE_COUNT; p++) {
        mapped = mapped && s_map[p];
        printf("%s{", p ? "," : "");
        print_tire_fields(p, s_map[p], s_map[p] ? find_sensor(s_map[p]) : NULL, now);
        printf("}");
    }
    printf("],\"mapped\":%s}\n", mapped ? "true" : "false");
    xSemaphoreGive(s_lock);
}

// ---- Mapping ---------------------------------------------------------------

static void map_finish(void)
{
    memcpy(s_map, s_pending, sizeof(s_map));
    save_mapping();
    s_mapping = false;

    char extra[128];
    snprintf(extra, sizeof(extra),
             "\"map\":{\"FL\":\"%08" PRIx32 "\",\"FR\":\"%08" PRIx32 "\",\"RL\":\"%08" PRIx32 "\",\"RR\":\"%08" PRIx32 "\"}",
             s_map[TIRE_FL], s_map[TIRE_FR], s_map[TIRE_RL], s_map[TIRE_RR]);
    emit_map_event("done", extra);
    feedback_set_mode(FB_IDLE, 0);
    feedback_done();
}

static void map_abort(const char *event)
{
    char extra[48];
    snprintf(extra, sizeof(extra), "\"step\":%d,\"pos\":\"%s\"", s_step + 1, tire_pos_name(s_order[s_step]));
    s_mapping = false;
    emit_map_event(event, extra);
    feedback_set_mode(FB_IDLE, 0);
    feedback_failed();
}

static void map_start(void)
{
    int64_t now = esp_timer_get_time();
    int candidates = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        sensor_t *s = &s_sensors[i];
        s->has_baseline = s->count && now - s->last_seen_us < MAP_CANDIDATE_AGE_S * US_PER_S;
        s->baseline_kpa = s->kpa;
        candidates += s->has_baseline;
    }
    memset(s_pending, 0, sizeof(s_pending));
    s_candidates_locked = candidates >= TIRE_COUNT;
    s_mapping = true;
    s_step = 0;
    s_step_started_us = now;

    char extra[96];
    snprintf(extra, sizeof(extra), "\"order\":[\"%s\",\"%s\",\"%s\",\"%s\"],\"candidates\":%d,\"drop_psi\":%.1f",
             tire_pos_name(s_order[0]), tire_pos_name(s_order[1]), tire_pos_name(s_order[2]),
             tire_pos_name(s_order[3]), candidates, MAP_DROP_PSI);
    emit_map_event("start", extra);
    if (candidates < TIRE_COUNT) {
        // Sensors not heard yet get their baseline from their first packet,
        // so they need one packet before the deflation to be mappable.
        emit_map_event("warning", "\"msg\":\"fewer than 4 sensors heard recently; drive briefly first for best results\"");
    }
    emit_map_wait();
    feedback_set_mode(FB_MAP_WAIT, 1);
}

static void map_on_reading(sensor_t *s)
{
    for (int i = 0; i < s_step; i++) {
        if (s_pending[s_order[i]] == s->id) {
            return; // already assigned this run
        }
    }
    if (!s->has_baseline) {
        if (s_candidates_locked) {
            return; // e.g. a passing car's sensor
        }
        s->baseline_kpa = s->kpa;
        s->has_baseline = true;
        return;
    }
    if (s->kpa > s->baseline_kpa) {
        s->baseline_kpa = s->kpa; // e.g. tire warming up; keep the peak
        return;
    }
    float drop_psi = (s->baseline_kpa - s->kpa) / KPA_PER_PSI;
    if (drop_psi < MAP_DROP_PSI) {
        return;
    }

    tire_pos_t pos = s_order[s_step];
    s_pending[pos] = s->id;
    char extra[96];
    snprintf(extra, sizeof(extra), "\"step\":%d,\"pos\":\"%s\",\"id\":\"%08" PRIx32 "\",\"drop_psi\":%.1f",
             s_step + 1, tire_pos_name(pos), s->id, drop_psi);
    emit_map_event("assigned", extra);
    feedback_assigned();

    s_step++;
    s_step_started_us = esp_timer_get_time();
    if (s_step == TIRE_COUNT) {
        map_finish();
    } else {
        emit_map_wait();
        feedback_set_mode(FB_MAP_WAIT, s_step + 1);
    }
}

// ---- Single-tire learn -----------------------------------------------------

static void learn_stop(const char *event)
{
    char extra[32];
    snprintf(extra, sizeof(extra), "\"pos\":\"%s\"", tire_pos_name(s_learn_pos));
    emit_map_event(event, extra);
    s_learn_pos = -1;
    feedback_set_mode(FB_IDLE, 0);
    feedback_failed();
}

static void learn_on_reading(sensor_t *s)
{
    int pos = position_of(s->id);
    if (pos >= 0 && pos != s_learn_pos) {
        return; // already saved on another wheel
    }
    if (!s->has_baseline) {
        s->baseline_kpa = s->kpa;
        s->has_baseline = true;
        return;
    }
    // Either direction: filling a low tire counts as much as letting air out.
    float delta_psi = (s->kpa - s->baseline_kpa) / KPA_PER_PSI;
    if (fabsf(delta_psi) < MAP_DROP_PSI) {
        return;
    }

    for (int p = 0; p < TIRE_COUNT; p++) {
        if (s_map[p] == s->id) {
            s_map[p] = 0;
        }
    }
    s_map[s_learn_pos] = s->id;
    save_mapping();
    char extra[96];
    snprintf(extra, sizeof(extra), "\"pos\":\"%s\",\"id\":\"%08" PRIx32 "\",\"delta_psi\":%.1f",
             tire_pos_name(s_learn_pos), s->id, delta_psi);
    emit_map_event("learned", extra);
    s_learn_pos = -1;
    feedback_set_mode(FB_IDLE, 0);
    feedback_done();
}

// ---- Public API ------------------------------------------------------------

void tires_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    load_mapping();
    ESP_LOGI(TAG, "mapping: FL=%08" PRIx32 " FR=%08" PRIx32 " RL=%08" PRIx32 " RR=%08" PRIx32,
             s_map[TIRE_FL], s_map[TIRE_FR], s_map[TIRE_RL], s_map[TIRE_RR]);
}

void tires_on_reading(const tpms_reading_t *r)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    sensor_t *s = find_sensor(r->id);
    // Sensors repeat each packet; only emit the first of a burst.
    bool repeat = s && s->kpa == r->pressure_kpa && now - s->last_seen_us < 2 * US_PER_S;
    if (!s) {
        sensor_t *oldest = &s_sensors[0];
        for (int i = 0; i < MAX_SENSORS; i++) {
            if (!s_sensors[i].count) {
                oldest = &s_sensors[i];
                break;
            }
            if (position_of(s_sensors[i].id) < 0 && s_sensors[i].last_seen_us < oldest->last_seen_us) {
                oldest = &s_sensors[i];
            }
        }
        s = oldest;
        memset(s, 0, sizeof(*s));
        s->id = r->id;
    }
    s->model = r->model;
    s->kpa = r->pressure_kpa;
    s->temp_c = r->temp_c;
    s->battery_ok = r->battery_ok;
    s->rapid_change = r->rapid_change;
    s->failed = r->failed;
    s->last_seen_us = now;
    s->count++;

    if (!repeat) {
        emit_tire(s, now);
        feedback_packet();
    }
    if (s_mapping) {
        map_on_reading(s);
    } else if (s_learn_pos >= 0) {
        learn_on_reading(s);
    }
    xSemaphoreGive(s_lock);
}

void tires_tick(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_mapping && esp_timer_get_time() - s_step_started_us > MAP_STEP_TIMEOUT_S * US_PER_S) {
        map_abort("timeout");
    }
    if (s_learn_pos >= 0 && esp_timer_get_time() - s_learn_started_us > MAP_STEP_TIMEOUT_S * US_PER_S) {
        learn_stop("timeout");
    }
    xSemaphoreGive(s_lock);
}

void tires_button_press(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_learn_pos >= 0) {
        learn_stop("cancelled");
    } else if (s_mapping) {
        map_abort("cancelled");
    } else {
        map_start();
    }
    xSemaphoreGive(s_lock);
}

bool tire_pos_from_name(const char *name, tire_pos_t *pos)
{
    for (int p = 0; p < TIRE_COUNT; p++) {
        const char *n = tire_pos_name(p);
        if (toupper((unsigned char)name[0]) == n[0] && toupper((unsigned char)name[1]) == n[1]) {
            *pos = p;
            return true;
        }
    }
    return false;
}

bool tires_assign(tire_pos_t pos, uint32_t id)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_mapping) {
        xSemaphoreGive(s_lock);
        return false;
    }
    for (int p = 0; p < TIRE_COUNT; p++) {
        if (id && s_map[p] == id) {
            s_map[p] = 0; // a sensor can only be on one wheel
        }
    }
    s_map[pos] = id;
    save_mapping();
    char extra[48];
    snprintf(extra, sizeof(extra), "\"pos\":\"%s\",\"id\":\"%08" PRIx32 "\"", tire_pos_name(pos), id);
    emit_map_event("set", extra);
    if ((int)pos == s_learn_pos) {
        s_learn_pos = -1; // assigned by hand; nothing left to learn
        feedback_set_mode(FB_IDLE, 0);
    }
    xSemaphoreGive(s_lock);
    return true;
}

void tires_clear_mapping(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_map, 0, sizeof(s_map));
    save_mapping();
    emit_map_event("cleared", NULL);
    xSemaphoreGive(s_lock);
}

void tires_print_table(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    ESP_LOGI(TAG, "mapping %s, step %d, learn %s", s_mapping ? "RUNNING" : "idle", s_mapping ? s_step + 1 : 0,
             s_learn_pos >= 0 ? tire_pos_name(s_learn_pos) : "--");
    for (int p = 0; p < TIRE_COUNT; p++) {
        ESP_LOGI(TAG, "  %s = %08" PRIx32, tire_pos_name(p), s_map[p]);
    }
    for (int i = 0; i < MAX_SENSORS; i++) {
        sensor_t *s = &s_sensors[i];
        if (!s->count) {
            continue;
        }
        int pos = position_of(s->id);
        ESP_LOGI(TAG, "  sensor %08" PRIx32 " %-2s %5.1f psi %4.0f C seen %" PRIu32 "x, %llds ago%s",
                 s->id, pos >= 0 ? tire_pos_name(pos) : "--", s->kpa / KPA_PER_PSI, s->temp_c, s->count,
                 (long long)((now - s->last_seen_us) / US_PER_S),
                 (s_mapping || s_learn_pos >= 0) && s->has_baseline ? " [baseline]" : "");
    }
    xSemaphoreGive(s_lock);
}

bool tires_learn_start(tire_pos_t pos)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_mapping) {
        xSemaphoreGive(s_lock);
        return false;
    }
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_SENSORS; i++) {
        sensor_t *s = &s_sensors[i];
        s->has_baseline = s->count && now - s->last_seen_us < MAP_CANDIDATE_AGE_S * US_PER_S;
        s->baseline_kpa = s->kpa;
    }
    s_learn_pos = pos;
    s_learn_started_us = now;
    char extra[48];
    snprintf(extra, sizeof(extra), "\"pos\":\"%s\",\"delta_psi\":%.1f", tire_pos_name(pos), MAP_DROP_PSI);
    emit_map_event("learn", extra);
    int blinks = 1;
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (s_order[i] == pos) {
            blinks = i + 1;
        }
    }
    feedback_set_mode(FB_MAP_WAIT, blinks);
    xSemaphoreGive(s_lock);
    return true;
}

void tires_learn_cancel(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_learn_pos >= 0) {
        learn_stop("cancelled");
    } else if (s_mapping) {
        map_abort("cancelled");
    }
    xSemaphoreGive(s_lock);
}
