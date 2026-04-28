#include "esp_temp.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"

static const char *TAG = "ESP_TEMP";

/* Single global handle for the built-in temperature sensor */
static temperature_sensor_handle_t s_tsens = NULL;

esp_err_t esp_temp_init(void)
{
    if (s_tsens) {
        // Already initialized
        return ESP_OK;
    }

    /*
     * Configure the expected measurement range.
     *
     * This is NOT ambient temperature, it's the silicon die temperature.
     * For a small ESP32-C3 board in your use-case, something like 10–50 °C
     * is a reasonable default. If you expect much colder / hotter conditions,
     * you can widen this range later.
     */
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    esp_err_t err = temperature_sensor_install(&cfg, &s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "temperature_sensor_install() failed: %s", esp_err_to_name(err));
        s_tsens = NULL;
        return err;
    }

    err = temperature_sensor_enable(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "temperature_sensor_enable() failed: %s", esp_err_to_name(err));
        (void)temperature_sensor_uninstall(s_tsens);
        s_tsens = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Internal temperature sensor init (driver-based, calibrated)");
    return ESP_OK;
}

esp_err_t esp_temp_read_c(float *out_temp)
{
    if (!out_temp) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_tsens) {
        /* Lazy init if caller forgot to call esp_temp_init() */
        esp_err_t err = esp_temp_init();
        if (err != ESP_OK) {
            return err;
        }
    }

    float temp_c = 0.0f;
    esp_err_t err = temperature_sensor_get_celsius(s_tsens, &temp_c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "temperature_sensor_get_celsius() failed: %s", esp_err_to_name(err));
        return err;
    }

    *out_temp = temp_c;
    ESP_LOGD(TAG, "Internal temperature: %.2f °C", *out_temp);

    return ESP_OK;
}
