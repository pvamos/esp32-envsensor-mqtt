#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32C3
#error "This firmware is intended for ESP32-C3 (SparkFun Pro Micro ESP32-C3)."
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_err.h"
#include "esp_attr.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "wifi_sta.h"
#include "mqtt_push.h"
#include "sensors_enable.h"
#include "esp_temp.h"

#if (ENVSENSOR_USE_BME280 || ENVSENSOR_USE_SHT4X)
#include "i2c.h"
#endif

#if ENVSENSOR_USE_BME280
#include "bme280.h"
#endif
#if ENVSENSOR_USE_SHT4X
#include "sht4x.h"
#endif

static const char *TAG = "MAIN";

/* Base cadence and timers */
#define ENVSENSOR_PERIOD_US             (30ULL * 1000ULL * 1000ULL)        // 30 seconds
#define ENVSENSOR_BACKOFF_PERIOD_US    (15ULL * 60ULL * 1000ULL * 1000ULL) // 15 minutes
#define ENVSENSOR_MIN_SLEEP_US          (1ULL  * 1000ULL * 1000ULL)        // 1 second safety minimum

/* One-time random delay after true power-on (not after deep-sleep wakeup) */
#define ENVSENSOR_INITIAL_JITTER_MAX_MS (15000U) // 0..15 seconds on first boot

/* RTC state for Wi-Fi/MQTT backoff */
#define ENVSENSOR_RTC_MAGIC 0xE512FA17u

typedef struct {
    uint32_t magic;
    uint32_t consecutive_failures; /* consecutive cycles with Wi-Fi or MQTT failure */
    uint8_t  in_backoff;           /* 0 = normal 30 s, 1 = 15 min backoff */
    uint8_t  _pad[3];
} envsensor_rtc_state_t;

RTC_DATA_ATTR static envsensor_rtc_state_t s_env_rtc_state;

static void env_rtc_state_init(esp_sleep_wakeup_cause_t cause)
{
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED ||
        s_env_rtc_state.magic != ENVSENSOR_RTC_MAGIC) {
        memset(&s_env_rtc_state, 0, sizeof(s_env_rtc_state));
        s_env_rtc_state.magic = ENVSENSOR_RTC_MAGIC;
        ESP_LOGI(TAG, "RTC state initialised (power-on/reset).");
    } else {
        ESP_LOGI(TAG,
                 "RTC state: consecutive_failures=%u, in_backoff=%u",
                 (unsigned)s_env_rtc_state.consecutive_failures,
                 (unsigned)s_env_rtc_state.in_backoff);
    }
}

static void deep_sleep_until_next_period(uint64_t cycle_start_us)
{
    uint64_t now_us     = (uint64_t)esp_timer_get_time();
    uint64_t elapsed_us = now_us - cycle_start_us;

    /* Choose target period based on backoff state */
    uint64_t target_period_us = s_env_rtc_state.in_backoff
                                ? ENVSENSOR_BACKOFF_PERIOD_US
                                : ENVSENSOR_PERIOD_US;

    /*
     * If active time < period: sleep the remaining time so that
     * (active + sleep) ≈ target_period_us.
     * If active time >= period: still sleep at least ENVSENSOR_MIN_SLEEP_US.
     */
    uint64_t sleep_us = (elapsed_us < target_period_us)
                        ? (target_period_us - elapsed_us)
                        : ENVSENSOR_MIN_SLEEP_US;

    ESP_LOGW(TAG,
             "MAIN: Cycle took %llu ms, sleeping for %llu ms",
             (unsigned long long)(elapsed_us / 1000ULL),
             (unsigned long long)(sleep_us  / 1000ULL));

    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
}

/* Update RTC-backed state based on Wi-Fi/MQTT outcome of this cycle */
static void env_rtc_record_cycle_result(bool network_attempted, bool network_success)
{
    if (!network_attempted) {
        /* No Wi-Fi/MQTT work done in this cycle: do not touch backoff state. */
        return;
    }

    if (network_success) {
        /* Successful Wi-Fi + MQTT publish: reset failure counter and backoff. */
        if (s_env_rtc_state.consecutive_failures != 0 ||
            s_env_rtc_state.in_backoff != 0) {
            ESP_LOGI(TAG,
                     "Network OK after %u consecutive failures, leaving backoff (if active).",
                     (unsigned)s_env_rtc_state.consecutive_failures);
        }
        s_env_rtc_state.consecutive_failures = 0;
        s_env_rtc_state.in_backoff           = 0;
        return;
    }

    /* Wi-Fi connect/DHCP failure or MQTT publish failure */
    if (!s_env_rtc_state.in_backoff) {
        if (s_env_rtc_state.consecutive_failures < UINT32_MAX) {
            s_env_rtc_state.consecutive_failures++;
        }
        ESP_LOGW(TAG,
                 "Network failure, consecutive_failures=%u",
                 (unsigned)s_env_rtc_state.consecutive_failures);

        if (s_env_rtc_state.consecutive_failures >= 6) {
            s_env_rtc_state.in_backoff = 1;
            ESP_LOGW(TAG,
                     "Entering 15-minute backoff after %u consecutive Wi-Fi/MQTT failures.",
                     (unsigned)s_env_rtc_state.consecutive_failures);
        }
    } else {
        /* Already in backoff, keep counting but stay in backoff. */
        if (s_env_rtc_state.consecutive_failures < UINT32_MAX) {
            s_env_rtc_state.consecutive_failures++;
        }
        ESP_LOGW(TAG,
                 "Network failure during backoff, consecutive_failures=%u",
                 (unsigned)s_env_rtc_state.consecutive_failures);
    }
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Boot, wakeup_cause=%d", cause);

    /* Initialise RTC-backed backoff state */
    env_rtc_state_init(cause);

    /*
     * Only on true power-on / reset (not deep-sleep wake):
     * add a random 0..15 second delay to de-synchronise devices
     * after a simultaneous power outage.
     */
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        uint32_t initial_jitter_ms = 0;

        if (ENVSENSOR_INITIAL_JITTER_MAX_MS > 0) {
            /* 0 .. ENVSENSOR_INITIAL_JITTER_MAX_MS - 1 ms */
            initial_jitter_ms = esp_random() % ENVSENSOR_INITIAL_JITTER_MAX_MS;
        }

        ESP_LOGW(TAG,
                 "Power-on boot: initial random delay %u ms (0..%u ms)",
                 (unsigned)initial_jitter_ms,
                 (unsigned)ENVSENSOR_INITIAL_JITTER_MAX_MS);

        if (initial_jitter_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(initial_jitter_ms));
        }
    }

    /* Start the cadence AFTER the optional initial jitter */
    uint64_t cycle_start_us = (uint64_t)esp_timer_get_time();

    bool network_attempted = false;
    bool network_success   = false;

    /* Start Wi-Fi connection in the background immediately */
    if (wifi_start_async() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi start failed, sleeping anyway");
        network_attempted = true;
        network_success   = false;
        env_rtc_record_cycle_result(network_attempted, network_success);
        deep_sleep_until_next_period(cycle_start_us);
    }

    /* Internal ESP32-C3 temperature (very coarse, but stable) */
    float esp32_temp = NAN;

    esp_err_t init_err = esp_temp_init();
    ESP_LOGI(TAG, "esp_temp_init() -> %s", esp_err_to_name(init_err));

    if (init_err == ESP_OK) {
        esp_err_t read_err = esp_temp_read_c(&esp32_temp);
        ESP_LOGI(TAG, "esp_temp_read_c() -> %s, value=%.2f",
                 esp_err_to_name(read_err), esp32_temp);

        if (read_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read internal ESP32 temperature");
            esp32_temp = NAN;
        } else {
            ESP_LOGI(TAG, "ESP32 internal temperature (final): %.2f °C", esp32_temp);
        }
    } else {
        ESP_LOGW(TAG, "Internal temperature sensor init failed");
    }

#if (ENVSENSOR_USE_BME280 || ENVSENSOR_USE_SHT4X)
    /* I2C bus + sensors (run in parallel with Wi-Fi connect) */
    i2c_master_bus_handle_t i2c_bus_handle;
    if (i2c_master_init(&i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        /* No Wi-Fi/MQTT attempt accounted for in backoff logic */
        deep_sleep_until_next_period(cycle_start_us);
    }

#if ENVSENSOR_USE_BME280
    if (bme280_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BME280 sensor");
    }
#endif
#if ENVSENSOR_USE_SHT4X
    if (sht4x_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SHT4X sensor");
    }
#endif
#endif // any sensor enabled

    float bme280_temp      = NAN;
    float bme280_pressure  = NAN;
    float bme280_humidity  = NAN;
    float sht4x_temp       = NAN;
    float sht4x_humidity   = NAN;

#if ENVSENSOR_USE_BME280
    if (bme280_read(&bme280_temp, &bme280_pressure, &bme280_humidity) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read BME280");
        bme280_temp = bme280_pressure = bme280_humidity = NAN;
    }
#endif

#if ENVSENSOR_USE_SHT4X
    if (sht4x_read(&sht4x_temp, &sht4x_humidity) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read SHT4X");
        sht4x_temp = sht4x_humidity = NAN;
    }
#endif

    /*
     * Wi-Fi + MQTT path
     * Wait until we really have an IPv4 address (or give up)
     */
    if (wifi_wait_ready(15000) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi not ready, skipping MQTT publish");
        network_attempted = true;
        network_success   = false;
        env_rtc_record_cycle_result(network_attempted, network_success);
        deep_sleep_until_next_period(cycle_start_us);
    }

	network_attempted = true;

	wifi_sta_status_t st;
	memset(&st, 0, sizeof(st));

	if (wifi_get_status(&st) != ESP_OK) {
		ESP_LOGW(TAG, "Wi-Fi ready but failed to get status");
	}

	int32_t rssi_dbm = wifi_get_rssi_dbm();

	ESP_LOGI(TAG,
			 "Wi-Fi connection:\n"
			 "  connected : %s\n"
			 "  fast_conn : %s\n"
			 "  BSSID     : %02X:%02X:%02X:%02X:%02X:%02X\n"
			 "  channel   : %u\n"
			 "  RSSI      : %d dBm\n"
			 "  IP        : 0x%08X\n"
			 "  Gateway   : 0x%08X\n"
			 "  DNS main  : 0x%08X\n"
			 "  DNS backup: 0x%08X\n"
			 "  DNS fb    : 0x%08X\n"
			 "  DNS count : %d",
			 st.connected ? "yes" : "no",
			 st.fast_connect ? "yes" : "no",
			 st.bssid[0], st.bssid[1], st.bssid[2],
			 st.bssid[3], st.bssid[4], st.bssid[5],
			 st.channel,
			 (int)rssi_dbm,
			 (unsigned)st.ip,
			 (unsigned)st.gw,
			 (unsigned)st.dns_main,
			 (unsigned)st.dns_backup,
			 (unsigned)st.dns_fallback,
			 st.dns_count);

	const char *broker_uri = mqtt_push_get_broker_uri();
	const char *mqtt_topic = mqtt_push_get_topic();

	ESP_LOGI(TAG,
			 "Sensor / MQTT payload:\n"
			 "  ESP32 temp    : %.2f °C\n"
			 "  BME280 temp   : %.2f °C\n"
			 "  BME280 press  : %.2f hPa\n"
			 "  BME280 humid  : %.2f %%RH\n"
			 "  SHT4X  temp   : %.2f °C\n"
			 "  SHT4X  humid  : %.2f %%RH\n"
			 "  MQTT broker   : %s\n"
			 "  MQTT topic    : %s",
			 esp32_temp,
			 bme280_temp,
			 bme280_pressure,
			 bme280_humidity,
			 sht4x_temp,
			 sht4x_humidity,
			 broker_uri ? broker_uri : "<null>",
			 mqtt_topic ? mqtt_topic : "<null>");

    /* One-shot MQTT: connect -> publish -> disconnect */
    esp_err_t mqtt_err = mqtt_publish_once(
        rssi_dbm,
        esp32_temp,
        bme280_temp, bme280_pressure, bme280_humidity,
        sht4x_temp,  sht4x_humidity
    );

    if (mqtt_err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT publish failed: %s", esp_err_to_name(mqtt_err));
        network_success = false;
    } else {
        network_success = true;
    }

    env_rtc_record_cycle_result(network_attempted, network_success);

    /* Sleep until the cadence point (active time subtracted, no per-cycle jitter) */
    deep_sleep_until_next_period(cycle_start_us);
}
