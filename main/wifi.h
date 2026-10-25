#pragma once

#include <stdbool.h>
#include <stddef.h>

// Wi-Fi: the access point is always up (so the phone UI works in the truck),
// and the board also joins one saved network as a station when it's in range.
// The station's SSID/password are kept in NVS by the Wi-Fi driver itself.
//
// While the station is connected the AP moves to the router's channel
// (the radio can only be on one); WIFI_AP_CHANNEL applies otherwise.

#define WIFI_AP_SSID     "4runner-tpms"
#define WIFI_AP_PASSWORD "tpms4runner" // WPA2, at least 8 chars
#define WIFI_AP_CHANNEL  6
#define WIFI_HOSTNAME    "4runner-tpms" // DHCP hostname, shows in the router's client list
#define WIFI_MDNS_NAME   "tpms"         // http://tpms.local on either network

// Brings up the AP, and the station if a network is saved. Call once, after nvs_flash_init().
void wifi_start(void);

// Saves the network and (re)connects to it. Empty password = open network.
// Returns false if ssid is empty or over 32 bytes, or a password isn't 8-63 chars.
bool wifi_connect(const char *ssid, const char *password);

// Disconnects and erases the saved network.
void wifi_forget(void);

// {"mdns":"tpms.local","ap":{...},"sta":{"ssid":..,"state":"off|connecting|connected|failed",..}}
size_t wifi_status_json(char *buf, size_t len);

// Blocking scan (~2-4 s): [{"ssid":..,"rssi":..,"secure":true},...], strongest first,
// one entry per SSID. Returns 0 if the radio is busy (e.g. mid-connect) or on error.
size_t wifi_scan_json(char *buf, size_t len);
