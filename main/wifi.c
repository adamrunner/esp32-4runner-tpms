#include "wifi.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mdns.h"

#define FAST_RETRIES  5         // back-to-back attempts after a drop, then...
#define RETRY_SLOW_US 30000000  // ...one every 30 s (each attempt scans, briefly pulling the AP off channel)
#define FAILED_AFTER  3         // attempts before the UI shows "failed" (it keeps retrying)
#define SCAN_MAX      24

static const char *TAG = "net";

typedef enum { STA_OFF, STA_CONNECTING, STA_CONNECTED, STA_FAILED } sta_state_t;
static const char *const STATE_NAMES[] = {"off", "connecting", "connected", "failed"};

static SemaphoreHandle_t s_lock; // guards everything below; the event task and httpd both touch it
static esp_netif_t *s_sta_netif;
static esp_timer_handle_t s_retry_timer;
static char s_ssid[33];          // saved network, "" = none
static sta_state_t s_state;
static int s_attempts;
static uint8_t s_last_reason;
static esp_ip4_addr_t s_ip;

static const char *reason_text(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "network not found";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        return "network found, but its security doesn't match (open vs. password)";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "wrong password?";
    default:
        return NULL;
    }
}

// Caller holds s_lock.
static void try_connect(void)
{
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        // e.g. a web scan is running; no disconnect event will follow, so retry later.
        ESP_LOGW(TAG, "connect: %s; retrying in 30 s", esp_err_to_name(err));
        esp_timer_stop(s_retry_timer);
        esp_timer_start_once(s_retry_timer, RETRY_SLOW_US);
    }
}

static void retry_cb(void *arg)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_ssid[0] && s_state != STA_CONNECTED) {
        try_connect();
    }
    xSemaphoreGive(s_lock);
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_ssid[0]) {
            ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
            s_state = STA_CONNECTING;
            try_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        // Without an IPv6 address mDNS can't answer AAAA queries, and macOS
        // waits ~5 s for one before using the A record for tpms.local.
        esp_netif_create_ip6_linklocal(s_sta_netif);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        // wifi_connect() drops the old link before joining the new one; that
        // drop's event lands here while the new attempt is already under way.
        bool ours = e->reason == WIFI_REASON_ASSOC_LEAVE && s_state == STA_CONNECTING;
        if (!ours && s_ssid[0]) {
            const char *why = reason_text(e->reason);
            if (s_state == STA_CONNECTED) {
                ESP_LOGW(TAG, "lost \"%s\" (reason %d); reconnecting", s_ssid, e->reason);
                s_attempts = 0;
            } else if (why) {
                ESP_LOGW(TAG, "\"%s\": %s (reason %d)", s_ssid, why, e->reason);
            } else {
                ESP_LOGW(TAG, "\"%s\": disconnected (reason %d)", s_ssid, e->reason);
            }
            s_last_reason = e->reason;
            s_ip.addr = 0;
            s_attempts++;
            s_state = s_attempts >= FAILED_AFTER ? STA_FAILED : STA_CONNECTING;
            if (s_attempts < FAST_RETRIES) {
                try_connect();
            } else {
                esp_timer_stop(s_retry_timer);
                esp_timer_start_once(s_retry_timer, RETRY_SLOW_US);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        s_ip = e->ip_info.ip;
        s_state = STA_CONNECTED;
        s_attempts = 0;
        s_last_reason = 0;
        ESP_LOGI(TAG, "joined \"%s\"; open http://" IPSTR, s_ssid, IP2STR(&s_ip));
    }
    xSemaphoreGive(s_lock);
}

static void start_ap_netif(void)
{
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    // No router option in the DHCP offer: with no default gateway on this
    // network, phones keep using cellular for internet while joined. (DNS is
    // off too, via CONFIG_LWIP_DHCPS_ADD_DNS=n.) Options only change while
    // the server is stopped.
    esp_netif_dhcps_stop(ap);
    uint8_t offer_router = 0;
    ESP_ERROR_CHECK(esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS,
                                           &offer_router, sizeof(offer_router)));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap));
}

void wifi_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    const esp_timer_create_args_t timer = {.callback = retry_cb, .name = "wifi_retry"};
    ESP_ERROR_CHECK(esp_timer_create(&timer, &s_retry_timer));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    start_ap_netif();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_sta_netif, WIFI_HOSTNAME);

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    // The driver loads the last station config from NVS (WIFI_STORAGE_FLASH, the default).
    wifi_config_t sta = {0};
    esp_wifi_get_config(WIFI_IF_STA, &sta);
    memcpy(s_ssid, sta.sta.ssid, sizeof(sta.sta.ssid));
    s_ssid[sizeof(s_ssid) - 1] = 0;

    wifi_config_t ap = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASSWORD,
            .channel = WIFI_AP_CHANNEL,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP \"%s\" up; open http://192.168.4.1", WIFI_AP_SSID);

    // Answers on both the AP and the station interface.
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(WIFI_MDNS_NAME);
        mdns_instance_name_set("4Runner TPMS");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOGI(TAG, "mDNS: http://%s.local", WIFI_MDNS_NAME);
    } else {
        ESP_LOGE(TAG, "mDNS init failed; use the IP address");
    }
    if (!s_ssid[0]) {
        ESP_LOGI(TAG, "no network saved; add one at http://192.168.4.1/wifi");
    }
}

bool wifi_connect(const char *ssid, const char *password)
{
    size_t ssid_len = strlen(ssid), pass_len = strlen(password);
    if (ssid_len == 0 || ssid_len > 32 || pass_len > 63 || (pass_len && pass_len < 8)) {
        return false;
    }
    wifi_config_t cfg = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN, // then pick the strongest AP (mesh / extenders)
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.authmode = pass_len ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
            .pmf_cfg = {.capable = true},
        },
    };
    memcpy(cfg.sta.ssid, ssid, ssid_len);
    memcpy(cfg.sta.password, password, pass_len);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_timer_stop(s_retry_timer);
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    s_state = STA_CONNECTING;
    s_attempts = 0;
    s_last_reason = 0;
    s_ip.addr = 0;
    esp_wifi_disconnect();
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg); // also saves it to NVS
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_config: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
    try_connect();
    xSemaphoreGive(s_lock);
    return true;
}

void wifi_forget(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_timer_stop(s_retry_timer);
    s_ssid[0] = 0;
    s_state = STA_OFF;
    s_attempts = 0;
    s_last_reason = 0;
    s_ip.addr = 0;
    esp_wifi_disconnect();
    wifi_config_t empty = {0};
    esp_wifi_set_config(WIFI_IF_STA, &empty);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "saved network forgotten");
}

#define APPEND(...)                                                              \
    do {                                                                         \
        if (n < len) {                                                           \
            n += snprintf(buf + n, len - n, __VA_ARGS__);                        \
        }                                                                        \
    } while (0)

// Appends s as a JSON string literal. SSIDs are arbitrary bytes.
static size_t append_json_str(char *buf, size_t len, size_t n, const char *s)
{
    APPEND("\"");
    for (; *s; s++) {
        unsigned char c = *s;
        if (c == '"' || c == '\\') {
            APPEND("\\%c", c);
        } else if (c < 0x20) {
            APPEND("\\u%04x", c);
        } else {
            APPEND("%c", c);
        }
    }
    APPEND("\"");
    return n;
}

size_t wifi_status_json(char *buf, size_t len)
{
    size_t n = 0;
    APPEND("{\"mdns\":\"%s.local\",\"ap\":{\"ssid\":", WIFI_MDNS_NAME);
    n = append_json_str(buf, len, n, WIFI_AP_SSID);
    APPEND(",\"ip\":\"192.168.4.1\"},\"sta\":{\"ssid\":");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_ssid[0]) {
        n = append_json_str(buf, len, n, s_ssid);
    } else {
        APPEND("null");
    }
    APPEND(",\"state\":\"%s\"", STATE_NAMES[s_state]);
    if (s_state == STA_CONNECTED) {
        wifi_ap_record_t info;
        APPEND(",\"ip\":\"" IPSTR "\"", IP2STR(&s_ip));
        if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
            APPEND(",\"rssi\":%d,\"channel\":%d", info.rssi, info.primary);
        }
    }
    if (s_last_reason) {
        const char *why = reason_text(s_last_reason);
        if (why) {
            APPEND(",\"error\":\"%s\"", why);
        } else {
            APPEND(",\"error\":\"disconnected (reason %d)\"", s_last_reason);
        }
    }
    xSemaphoreGive(s_lock);

    APPEND("}}");
    return n < len ? n : 0;
}

size_t wifi_scan_json(char *buf, size_t len)
{
    // Not under s_lock: the scan blocks for seconds and the event task needs the lock meanwhile.
    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan: %s", esp_err_to_name(err));
        return 0;
    }
    uint16_t count = SCAN_MAX;
    wifi_ap_record_t *recs = calloc(SCAN_MAX, sizeof(*recs));
    if (!recs) {
        esp_wifi_clear_ap_list();
        return 0;
    }
    esp_wifi_scan_get_ap_records(&count, recs); // sorted strongest first

    size_t n = 0;
    APPEND("[");
    bool first = true;
    for (int i = 0; i < count; i++) {
        const char *ssid = (const char *)recs[i].ssid;
        if (!ssid[0]) {
            continue; // hidden network
        }
        bool dup = false;
        for (int j = 0; j < i && !dup; j++) {
            dup = strcmp(ssid, (const char *)recs[j].ssid) == 0;
        }
        if (dup) {
            continue; // weaker AP of a network already listed
        }
        APPEND("%s{\"ssid\":", first ? "" : ",");
        n = append_json_str(buf, len, n, ssid);
        APPEND(",\"rssi\":%d,\"secure\":%s}", recs[i].rssi, recs[i].authmode == WIFI_AUTH_OPEN ? "false" : "true");
        first = false;
    }
    APPEND("]");
    free(recs);
    return n < len ? n : 0;
}
