// components/wifi_sta/wifi_sta.h

#ifndef WIFI_STA_H
#define WIFI_STA_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     connected;     // true if station is associated with an AP
    bool     got_ip;        // true if we have an IPv4 address
    bool     fast_connect;  // true if RTC BSSID+channel fast-connect was used

    uint8_t  bssid[6];      // last connected BSSID (all zeros if unknown)
    uint8_t  channel;       // primary channel (0 if unknown)
    int32_t  rssi_dbm;      // last known RSSI in dBm (0 if unknown)

    uint32_t ip;            // IPv4 address (network byte order), 0 if unknown
    uint32_t gw;            // IPv4 gateway (network byte order), 0 if unknown
    uint32_t dns_main;      // primary DNS (network byte order), 0 if unknown
    uint32_t dns_backup;    // backup DNS (network byte order), 0 if unknown
    uint32_t dns_fallback;  // fallback DNS (network byte order), 0 if unknown
    int      dns_count;     // how many DNS addresses are valid (0..3)
} wifi_sta_status_t;

// Start Wi-Fi station asynchronously (connect-first, scan only on failure; RTC bssid+channel fast connect if valid)
esp_err_t wifi_start_async(void);

// Wait until we really have an IPv4 address (IP_EVENT_STA_GOT_IP). timeout_ms=0 means wait forever.
esp_err_t wifi_wait_ready(uint32_t timeout_ms);

// Non-blocking check for readiness (GOT_IP bit set)
bool wifi_is_ready(void);

// Current AP RSSI in dBm (negative). Returns 0 if unknown/not connected.
int32_t wifi_get_rssi_dbm(void);

/**
 * @brief Retrieve last-known Wi-Fi association + IPv4/DNS status.
 *
 * Safe to call after wifi_wait_ready() returned ESP_OK.
 */
esp_err_t wifi_get_status(wifi_sta_status_t *out_status);

// Backwards-compatible: start + wait with a default timeout
esp_err_t wifi_init(void);

// Clear RTC fast-connect cache (forces "cold" behavior on next boot)
void wifi_rtc_clear(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_STA_H
