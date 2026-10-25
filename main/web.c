#include "web.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "logbuf.h"
#include "tires.h"
#include "wifi.h"

#define STATE_BUF_LEN 6144 // 16 sensors at ~220 bytes each, plus the header
#define LOG_CHUNK     8192 // per /api/log response; the page asks again until caught up
#define CMD_MAX       32   // same as the USB console's line buffer
#define WIFI_JSON_LEN 384
#define SCAN_BUF_LEN  6144 // 24 networks, even with fully escaped SSIDs
#define WIFI_BODY_MAX 128  // 32-byte SSID + newline + 63-char password

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");
extern const char log_html_start[] asm("_binary_log_html_start");
extern const char log_html_end[] asm("_binary_log_html_end");
extern const char wifi_html_start[] asm("_binary_wifi_html_start");
extern const char wifi_html_end[] asm("_binary_wifi_html_end");

static web_cmd_fn s_cmd;

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    // EMBED_TXTFILES appends a NUL; don't send it.
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t reply(httpd_req_t *req, bool ok, const char *msg)
{
    if (!ok) {
        httpd_resp_set_status(req, "409 Conflict");
    }
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t log_page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, log_html_start, log_html_end - log_html_start - 1);
}

static esp_err_t log_get(httpd_req_t *req)
{
    char query[32], val[16];
    uint32_t pos = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "since", val, sizeof(val)) == ESP_OK) {
        pos = strtoul(val, NULL, 10);
    }
    char *buf = malloc(LOG_CHUNK);
    if (!buf) {
        return httpd_resp_send_500(req);
    }
    size_t n = logbuf_read(&pos, buf, LOG_CHUNK);
    char start[12], next[12];
    snprintf(start, sizeof(start), "%" PRIu32, (uint32_t)(pos - n));
    snprintf(next, sizeof(next), "%" PRIu32, pos);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Log-Start", start);
    httpd_resp_set_hdr(req, "X-Log-Next", next);
    esp_err_t err = httpd_resp_send(req, buf, n);
    free(buf);
    return err;
}

static esp_err_t cmd_post(httpd_req_t *req)
{
    char line[CMD_MAX];
    if (req->content_len == 0 || req->content_len >= sizeof(line)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "one command line, under 32 chars");
    }
    int n = httpd_req_recv(req, line, req->content_len);
    if (n <= 0) {
        return httpd_resp_send_500(req);
    }
    line[n] = 0;
    line[strcspn(line, "\r\n")] = 0;
    printf("> %s\n", line); // echo, so the log shows what was run
    s_cmd(line);
    return reply(req, true, "ok");
}

static esp_err_t state_get(httpd_req_t *req)
{
    char *buf = malloc(STATE_BUF_LEN);
    if (!buf) {
        return httpd_resp_send_500(req);
    }
    size_t n = tires_state_json(buf, STATE_BUF_LEN);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = n ? httpd_resp_send(req, buf, n) : httpd_resp_send_500(req);
    free(buf);
    return err;
}

static bool query_pos(httpd_req_t *req, char *query, size_t len, tire_pos_t *pos)
{
    char val[8];
    return httpd_req_get_url_query_str(req, query, len) == ESP_OK &&
           httpd_query_key_value(query, "pos", val, sizeof(val)) == ESP_OK && tire_pos_from_name(val, pos);
}

static esp_err_t learn_post(httpd_req_t *req)
{
    char query[64];
    tire_pos_t pos;
    if (!query_pos(req, query, sizeof(query), &pos)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "pos=FL|FR|RL|RR");
    }
    bool ok = tires_learn_start(pos);
    return reply(req, ok, ok ? "learning" : "a mapping run is in progress; cancel it first");
}

static esp_err_t cancel_post(httpd_req_t *req)
{
    tires_learn_cancel();
    return reply(req, true, "cancelled");
}

static esp_err_t assign_post(httpd_req_t *req)
{
    char query[64], id[16];
    tire_pos_t pos;
    if (!query_pos(req, query, sizeof(query), &pos) ||
        httpd_query_key_value(query, "id", id, sizeof(id)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "pos=FL|FR|RL|RR&id=<hex>");
    }
    bool ok = tires_assign(pos, strtoul(id, NULL, 16));
    return reply(req, ok, ok ? "saved" : "a mapping run is in progress; cancel it first");
}

static esp_err_t wifi_page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, wifi_html_start, wifi_html_end - wifi_html_start - 1);
}

static esp_err_t wifi_get(httpd_req_t *req)
{
    char buf[WIFI_JSON_LEN];
    size_t n = wifi_status_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return n ? httpd_resp_send(req, buf, n) : httpd_resp_send_500(req);
}

static esp_err_t wifi_scan_get(httpd_req_t *req)
{
    char *buf = malloc(SCAN_BUF_LEN);
    if (!buf) {
        return httpd_resp_send_500(req);
    }
    size_t n = wifi_scan_json(buf, SCAN_BUF_LEN);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err;
    if (n) {
        httpd_resp_set_type(req, "application/json");
        err = httpd_resp_send(req, buf, n);
    } else {
        err = reply(req, false, "scan failed; the radio may be busy joining a network. Try again in a few seconds.");
    }
    free(buf);
    return err;
}

static esp_err_t wifi_connect_post(httpd_req_t *req)
{
    char body[WIFI_BODY_MAX];
    if (req->content_len == 0 || req->content_len >= sizeof(body)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body = ssid, newline, password");
    }
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, body + got, req->content_len - got);
        if (n <= 0) {
            return httpd_resp_send_500(req);
        }
        got += n;
    }
    body[got] = 0;
    char *pass = strchr(body, '\n');
    if (pass) {
        *pass++ = 0;
    } else {
        pass = "";
    }
    if (!wifi_connect(body, pass)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "SSID must be 1-32 bytes; password empty (open network) or 8-63 chars");
    }
    return reply(req, true, "joining");
}

static esp_err_t wifi_forget_post(httpd_req_t *req)
{
    wifi_forget();
    return reply(req, true, "forgotten");
}

void web_start(web_cmd_fn cmd)
{
    s_cmd = cmd;
    wifi_start();

    httpd_handle_t server;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true; // phones keep idle sockets open
    cfg.stack_size = 6144;       // /api/cmd runs console commands, which printf floats
    cfg.max_uri_handlers = 16;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    static const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get},
        {.uri = "/api/state", .method = HTTP_GET, .handler = state_get},
        {.uri = "/api/learn", .method = HTTP_POST, .handler = learn_post},
        {.uri = "/api/cancel", .method = HTTP_POST, .handler = cancel_post},
        {.uri = "/api/assign", .method = HTTP_POST, .handler = assign_post},
        {.uri = "/log", .method = HTTP_GET, .handler = log_page_get},
        {.uri = "/api/log", .method = HTTP_GET, .handler = log_get},
        {.uri = "/api/cmd", .method = HTTP_POST, .handler = cmd_post},
        {.uri = "/wifi", .method = HTTP_GET, .handler = wifi_page_get},
        {.uri = "/api/wifi", .method = HTTP_GET, .handler = wifi_get},
        {.uri = "/api/wifi/scan", .method = HTTP_GET, .handler = wifi_scan_get},
        {.uri = "/api/wifi/connect", .method = HTTP_POST, .handler = wifi_connect_post},
        {.uri = "/api/wifi/forget", .method = HTTP_POST, .handler = wifi_forget_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uris[i]));
    }
}
