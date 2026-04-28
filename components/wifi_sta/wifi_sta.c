// components/wifi_sta/wifi_sta.c

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_attr.h"
#include "wifi_sta.h"

#ifndef CONFIG_WIFI_SSID
#warning "Define CONFIG_WIFI_SSID and CONFIG_WIFI_PASSWORD (menuconfig) or update in components/wifi_sta/wifi_sta.c."
#define CONFIG_WIFI_SSID     "example-ssid"
#define CONFIG_WIFI_PASSWORD "example-password"
#endif

static const char *TAG = "WIFI_STA";

/* Event group bits */
#define BIT_CONNECTED   BIT0
#define BIT_GOT_IP      BIT1

/* RTC cache (FAST CONNECT ONLY: bssid+channel) */
#define RTC_MAGIC 0xC0FFEE42u
typedef struct {
    uint32_t magic;
    uint8_t  bssid[6];
    uint8_t  channel;   // primary channel
} wifi_rtc_cache_t;

RTC_DATA_ATTR static wifi_rtc_cache_t s_rtc;

/* State */
static EventGroupHandle_t    s_ev;
static esp_netif_t          *s_netif;
static esp_timer_handle_t    s_connect_timer;

static bool s_using_rtc_fast_connect = false;
static bool s_scan_fallback_pending  = false;
static bool s_scan_in_progress       = false;

/* Last-known connection/IP/DNS status (for logging in main) */
static wifi_sta_status_t s_status = {0};

static bool rtc_valid(void)
{
    return (s_rtc.magic == RTC_MAGIC) && (s_rtc.channel >= 1) && (s_rtc.channel <= 14);
}

static void rtc_store_bssid_channel(const uint8_t bssid[6], uint8_t channel)
{
    s_rtc.magic = RTC_MAGIC;
    memcpy(s_rtc.bssid, bssid, 6);
    s_rtc.channel = channel;
    ESP_LOGI(TAG, "RTC cache updated: bssid=%02x:%02x:%02x:%02x:%02x:%02x ch=%u",
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], channel);
}

static void connect_timer_start(uint32_t ms);
static void connect_timer_stop(void);
static void start_targeted_scan(void);
static void apply_best_ap_and_connect(void);

static void connect_timeout_cb(void *arg)
{
    (void)arg;

    EventBits_t bits = xEventGroupGetBits(s_ev);
    if (bits & BIT_CONNECTED) {
        return;
    }

    ESP_LOGW(TAG, "Connect timeout -> will scan fallback");
    s_scan_fallback_pending = true;

    // Force a clean state transition (scan is safer after DISCONNECTED)
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_disconnect (timeout path) -> %s", esp_err_to_name(err));
    }
}

static void connect_timer_start(uint32_t ms)
{
    if (!s_connect_timer) return;

    esp_err_t err = esp_timer_stop(s_connect_timer);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_timer_start_once(s_connect_timer, (uint64_t)ms * 1000ULL));
}

static void connect_timer_stop(void)
{
    if (!s_connect_timer) return;
    (void)esp_timer_stop(s_connect_timer);
}

/* --- WiFi event handlers --- */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    switch (id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "Wi-Fi started -> connect");
        connect_timer_start(2000);   // tune as you like
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

    case WIFI_EVENT_STA_CONNECTED: {
        const wifi_event_sta_connected_t *e = (const wifi_event_sta_connected_t *)data;
        xEventGroupSetBits(s_ev, BIT_CONNECTED);
        connect_timer_stop();

        /* Record association details for later logging */
        s_status.connected    = true;
        s_status.fast_connect = s_using_rtc_fast_connect;
        memcpy(s_status.bssid, e->bssid, sizeof(s_status.bssid));
        s_status.channel      = e->channel;

        wifi_ap_record_t ap = {0};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            s_status.rssi_dbm = ap.rssi;
        } else {
            s_status.rssi_dbm = 0;
        }

        ESP_LOGI(TAG, "Wi-Fi connected (ch=%u)", e->channel);
        rtc_store_bssid_channel(e->bssid, e->channel);
        break;
    }

    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *e = (const wifi_event_sta_disconnected_t *)data;

        xEventGroupClearBits(s_ev, BIT_CONNECTED | BIT_GOT_IP);
        connect_timer_stop();

        /* Clear runtime IP/DNS flags */
        s_status.connected   = false;
        s_status.got_ip      = false;
        s_status.rssi_dbm    = 0;
        s_status.ip          = 0;
        s_status.gw          = 0;
        s_status.dns_main    = 0;
        s_status.dns_backup  = 0;
        s_status.dns_fallback = 0;
        s_status.dns_count   = 0;

        ESP_LOGW(TAG, "Disconnected, reason=%d", e->reason);

        /* If fast-connect failed, or connect timed out, do scan fallback once */
        bool should_scan =
            s_scan_fallback_pending ||
            (s_using_rtc_fast_connect && (e->reason == WIFI_REASON_NO_AP_FOUND ||
                                          e->reason == WIFI_REASON_ASSOC_FAIL ||
                                          e->reason == WIFI_REASON_AUTH_FAIL ||
                                          e->reason == WIFI_REASON_BEACON_TIMEOUT));

        s_scan_fallback_pending = false;
        s_using_rtc_fast_connect = false;

        if (should_scan && !s_scan_in_progress) {
            start_targeted_scan();
            break;
        }

        /* Otherwise, just reconnect quickly */
        connect_timer_start(2000);
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    }

    case WIFI_EVENT_SCAN_DONE:
        s_scan_in_progress = false;
        apply_best_ap_and_connect();
        break;

    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;

    if (id != IP_EVENT_STA_GOT_IP) return;

    const ip_event_got_ip_t *e = (const ip_event_got_ip_t *)data;

    /* Update last-known IP + DNS information */
    s_status.got_ip = true;
    s_status.ip     = e->ip_info.ip.addr;
    s_status.gw     = e->ip_info.gw.addr;
    s_status.dns_main     = 0;
    s_status.dns_backup   = 0;
    s_status.dns_fallback = 0;
    s_status.dns_count    = 0;

    if (s_netif) {
        esp_netif_dns_info_t dns = {0};

        if (esp_netif_get_dns_info(s_netif, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK &&
            dns.ip.u_addr.ip4.addr != 0) {
            s_status.dns_main = dns.ip.u_addr.ip4.addr;
            s_status.dns_count++;
        }

        if (esp_netif_get_dns_info(s_netif, ESP_NETIF_DNS_BACKUP, &dns) == ESP_OK &&
            dns.ip.u_addr.ip4.addr != 0) {
            s_status.dns_backup = dns.ip.u_addr.ip4.addr;
            s_status.dns_count++;
        }

        if (esp_netif_get_dns_info(s_netif, ESP_NETIF_DNS_FALLBACK, &dns) == ESP_OK &&
            dns.ip.u_addr.ip4.addr != 0) {
            s_status.dns_fallback = dns.ip.u_addr.ip4.addr;
            s_status.dns_count++;
        }
    }

    // IMPORTANT: do not "fake DHCP" — only treat this event as ready.
    if (!(xEventGroupGetBits(s_ev) & BIT_GOT_IP)) {
        ESP_LOGI(TAG, "IPv4: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_ev, BIT_GOT_IP);
    } else {
        // harmless if it happens (DHCP renew etc.)
        ESP_LOGD(TAG, "Got IP again: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

/* --- Scan fallback --- */

static void start_targeted_scan(void)
{
    ESP_LOGW(TAG, "Starting scan fallback for SSID='%s' ...", CONFIG_WIFI_SSID);
    s_scan_in_progress = true;

    wifi_scan_config_t sc = {
        .ssid = (uint8_t *)CONFIG_WIFI_SSID,  // targeted scan
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_err_t err = esp_wifi_scan_start(&sc, false);
    if (err != ESP_OK) {
        s_scan_in_progress = false;
        ESP_LOGW(TAG, "scan_start failed (%s), retry connect", esp_err_to_name(err));
        connect_timer_start(2000);
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

static void apply_best_ap_and_connect(void)
{
    uint16_t ap_num = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));

    if (ap_num == 0) {
        ESP_LOGW(TAG, "Scan done: no APs found, reconnect normally");
        connect_timer_start(2000);
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    wifi_ap_record_t *recs = calloc(ap_num, sizeof(*recs));
    if (!recs) {
        ESP_LOGW(TAG, "OOM reading scan results, reconnect normally");
        connect_timer_start(2000);
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, recs));

    int best = -1;
    for (int i = 0; i < ap_num; i++) {
        if (strncmp((const char *)recs[i].ssid, CONFIG_WIFI_SSID, sizeof(recs[i].ssid)) != 0) continue;
        if (best < 0 || recs[i].rssi > recs[best].rssi) best = i;
    }

    if (best < 0) {
        ESP_LOGW(TAG, "Scan done: SSID not found in results, reconnect normally");
        free(recs);
        connect_timer_start(2000);
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    ESP_LOGI(TAG, "Scan best: bssid=%02x:%02x:%02x:%02x:%02x:%02x ch=%u rssi=%d",
             recs[best].bssid[0], recs[best].bssid[1], recs[best].bssid[2],
             recs[best].bssid[3], recs[best].bssid[4], recs[best].bssid[5],
             recs[best].primary, recs[best].rssi);

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, CONFIG_WIFI_SSID, sizeof(cfg.sta.ssid));
    strncpy((char *)cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(cfg.sta.password));

    cfg.sta.scan_method = WIFI_FAST_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    cfg.sta.bssid_set = 1;
    memcpy(cfg.sta.bssid, recs[best].bssid, 6);
    cfg.sta.channel = recs[best].primary;

    free(recs);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    connect_timer_start(2000);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

/* --- Public API --- */

esp_err_t wifi_start_async(void)
{
    if (!s_ev) s_ev = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_ev, ESP_ERR_NO_MEM, TAG, "event group create failed");

    // NVS init (robust)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_netif_init());

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    if (!s_netif) {
        s_netif = esp_netif_create_default_wifi_sta();
    }
    ESP_RETURN_ON_FALSE(s_netif, ESP_FAIL, TAG, "netif create failed");

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL));

    // Connect-timeout timer
    if (!s_connect_timer) {
        esp_timer_create_args_t tcfg = {
            .callback = &connect_timeout_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wifi_conn_to",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&tcfg, &s_connect_timer));
    }

    // Basic STA config
    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, CONFIG_WIFI_SSID, sizeof(cfg.sta.ssid));
    strncpy((char *)cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(cfg.sta.password));

    cfg.sta.scan_method = WIFI_FAST_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    // FAST CONNECT (no scan upfront): only if RTC is valid
    if (rtc_valid()) {
        cfg.sta.bssid_set = 1;
        memcpy(cfg.sta.bssid, s_rtc.bssid, 6);
        cfg.sta.channel = s_rtc.channel;
        s_using_rtc_fast_connect = true;
        ESP_LOGI(TAG, "Using RTC fast connect: bssid=%02x:%02x:%02x:%02x:%02x:%02x ch=%u",
                 s_rtc.bssid[0], s_rtc.bssid[1], s_rtc.bssid[2],
                 s_rtc.bssid[3], s_rtc.bssid[4], s_rtc.bssid[5],
                 s_rtc.channel);
    } else {
        // Cold boot: DO NOT scan upfront. Just connect. If it fails, timeout triggers scan fallback.
        cfg.sta.bssid_set = 0;
        cfg.sta.channel = 0;
        s_using_rtc_fast_connect = false;
        ESP_LOGI(TAG, "No RTC cache -> connect first, scan only if it fails");
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

esp_err_t wifi_wait_ready(uint32_t timeout_ms)
{
    if (!s_ev) return ESP_ERR_INVALID_STATE;

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_ev, BIT_GOT_IP, pdFALSE, pdTRUE, ticks);
    return (bits & BIT_GOT_IP) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool wifi_is_ready(void)
{
    return s_ev && ((xEventGroupGetBits(s_ev) & BIT_GOT_IP) != 0);
}

int32_t wifi_get_rssi_dbm(void)
{
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

esp_err_t wifi_get_status(wifi_sta_status_t *out_status)
{
    if (!out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_status = s_status;
    return ESP_OK;
}

esp_err_t wifi_init(void)
{
    esp_err_t err = wifi_start_async();
    if (err != ESP_OK) return err;

    // Default timeout for legacy callers
    return wifi_wait_ready(15000);
}

void wifi_rtc_clear(void)
{
    memset(&s_rtc, 0, sizeof(s_rtc));
}
