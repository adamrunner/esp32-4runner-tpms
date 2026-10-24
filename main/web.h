#pragma once

// Wi-Fi access point + phone web UI for learning tire sensors away from the
// laptop. Join WEB_AP_SSID and open http://192.168.4.1.
//
//   GET  /             the page (main/web/index.html)
//   GET  /api/state    tires_state_json()
//   POST /api/learn?pos=FL
//   POST /api/cancel
//   POST /api/assign?pos=FL&id=01069354   (id=0 unassigns)
//   GET  /log          raw console log page (main/web/log.html)
//   GET  /api/log?since=N   console output from byte offset N (logbuf.h);
//                      X-Log-Start = offset actually returned from, X-Log-Next = next N
//   POST /api/cmd      body = one console command line, e.g. "s"

#define WEB_AP_SSID     "4runner-tpms"
#define WEB_AP_PASSWORD "tpms4runner" // WPA2, at least 8 chars
#define WEB_AP_CHANNEL  6

// Runs one console command line (main.c); its output lands in the log.
typedef void (*web_cmd_fn)(const char *line);

void web_start(web_cmd_fn cmd);
