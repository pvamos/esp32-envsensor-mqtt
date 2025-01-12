#include "httpd.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include <inttypes.h> // For PRI macros
#include <stdlib.h>

static const char *TAG = "HTTPD";

// HTTP GET handler for sensor data
static esp_err_t sensor_data_handler(httpd_req_t *req) {
    int32_t raw_temp_bme280, raw_pressure_bme280, raw_humidity_bme280;
    int16_t raw_temp_tmp117;
    int32_t raw_temp_aht20, raw_humidity_aht20;
    float compensated_temp_bme280, compensated_pressure_bme280, compensated_humidity_bme280;
    float compensated_temp_tmp117, compensated_temp_aht20, compensated_humidity_aht20;

    // Read raw data from BME280
    if (bme280_read_raw(&raw_temp_bme280, &raw_pressure_bme280, &raw_humidity_bme280) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data from BME280");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Calculate compensated data for BME280
    if (bme280_calculate_compensated(
            raw_temp_bme280, raw_pressure_bme280, raw_humidity_bme280,
            &compensated_temp_bme280, &compensated_pressure_bme280, &compensated_humidity_bme280) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate compensated data for BME280");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read raw data from TMP117
    if (tmp117_read_raw(&raw_temp_tmp117) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data from TMP117");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Calculate compensated data for TMP117
    if (tmp117_calculate_compensated(raw_temp_tmp117, &compensated_temp_tmp117) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate compensated data for TMP117");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read raw data from AHT20
    if (aht20_read_raw(&raw_temp_aht20, &raw_humidity_aht20) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data from AHT20");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Calculate compensated data for AHT20
    if (aht20_calculate(raw_temp_aht20, raw_humidity_aht20, &compensated_temp_aht20, &compensated_humidity_aht20) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate compensated data for AHT20");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Generate Prometheus metrics
    char response[2048];
    int len = snprintf(response, sizeof(response),
                       "bme280_raw_temperature %" PRId32 "\n"
                       "bme280_raw_pressure %" PRId32 "\n"
                       "bme280_raw_humidity %" PRId32 "\n"
                       "bme280_compensated_temperature %.2f\n"
                       "bme280_compensated_pressure %.4f\n"
                       "bme280_compensated_humidity %.3f\n"
                       "tmp117_raw_temperature %" PRId16 "\n"
                       "tmp117_compensated_temperature %.4f\n"
                       "aht20_raw_temperature %" PRId32 "\n"
                       "aht20_raw_humidity %" PRId32 "\n"
                       "aht20_compensated_temperature %.2f\n"
                       "aht20_compensated_humidity %.3f\n",
                       raw_temp_bme280,
                       raw_pressure_bme280,
                       raw_humidity_bme280,
                       compensated_temp_bme280,
                       compensated_pressure_bme280,
                       compensated_humidity_bme280,
                       raw_temp_tmp117,
                       compensated_temp_tmp117,
                       raw_temp_aht20,
                       raw_humidity_aht20,
                       compensated_temp_aht20,
                       compensated_humidity_aht20);

    if (len < 0 || len >= sizeof(response)) {
        ESP_LOGE(TAG, "Response buffer too small or formatting error occurred.");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain; version=0.0.4; charset=utf-8");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// HTTP request handler for invalid routes (does nothing)
static esp_err_t default_handler(httpd_req_t *req) {
    return ESP_OK; // Close the connection without sending a response
}

// Function to start the HTTP server
esp_err_t http_server_start() {
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.stack_size = 32768; // Increased stack size
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_uri_t sensor_data_uri = {
        .uri       = "/metrics",
        .method    = HTTP_GET,
        .handler   = sensor_data_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t catch_all_uri = {
        .uri       = "/*",
        .method    = HTTP_GET,
        .handler   = default_handler,
        .user_ctx  = NULL
    };

    ESP_ERROR_CHECK(httpd_start(&server, &config));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &sensor_data_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &catch_all_uri));

    ESP_LOGI(TAG, "HTTP server started successfully");

    return ESP_OK;
}
