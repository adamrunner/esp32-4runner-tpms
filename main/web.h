#pragma once

// Phone web UI for learning tire sensors away from the laptop. Join the AP
// (WIFI_AP_SSID, wifi.h) and open http://192.168.4.1, or use the board's
// address on the saved network.
//
//   GET  /             the page (main/web/index.html)
//   GET  /api/state    tires_state_json()
//   POST /api/learn?pos=FL
//   POST /api/cancel
//   POST /api/assign?pos=FL&id=0a1b2c3d   (id=0 unassigns)
//   GET  /log          raw console log page (main/web/log.html)
//   GET  /api/log?since=N   console output from byte offset N (logbuf.h);
//                      X-Log-Start = offset actually returned from, X-Log-Next = next N
//   POST /api/cmd      body = one console command line, e.g. "s"
//   GET  /wifi         network settings page (main/web/wifi.html)
//   GET  /api/wifi     wifi_status_json()
//   GET  /api/wifi/scan     wifi_scan_json(); 409 if the radio is busy
//   POST /api/wifi/connect  body = "<ssid>\n<password>" (password may be empty)
//   POST /api/wifi/forget

// Runs one console command line (main.c); its output lands in the log.
typedef void (*web_cmd_fn)(const char *line);

void web_start(web_cmd_fn cmd);
