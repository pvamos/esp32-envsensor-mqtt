#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <math.h>

// Wi-Fi and MQTT
#include "wifi_sta.h"
#include "mqtt_push.h"
#include "i2c.h"

// Sensor drivers
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include "sht41.h"

// Old "protocol.h" might still be used for building frames, but we will no longer
// rely on its old BFS-based ECC. We'll remove or ignore protocol_init_bch() etc.
#include "protocol.h"

// 1) Include the new BCH library for 2-bit ECC encoding
#include "bch.h"

static const char *TAG = "MAIN";

// Our 6-byte location ID from macro
static const uint8_t s_location_id[6] = LOCATION_ID;

/////////////////////////////////////////
// Helper to pack a 32-bit message counter into big-endian 4 bytes
/////////////////////////////////////////
static void pack_message_id(uint8_t out[4], uint32_t counter)
{
    // big-endian
    out[0] = (uint8_t)((counter >> 24) & 0xFF);
    out[1] = (uint8_t)((counter >> 16) & 0xFF);
    out[2] = (uint8_t)((counter >>  8) & 0xFF);
    out[3] = (uint8_t)( counter        & 0xFF);
}

// We'll keep a global pointer to our BCH(1023,1003) 2-bit ECC context
static struct bch_control *g_bch = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");

    // 1) Wi-Fi
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed");
        return;
    }

    // 2) MQTT
    mqtt_init();

    // 3) I2C
    i2c_master_bus_handle_t i2c_bus_handle;
    if (i2c_master_init(&i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return;
    }

    // 4) Initialize sensors
    if (bme280_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init BME280");
        return;
    }
    if (tmp117_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init TMP117");
        return;
    }
    if (aht20_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init AHT20");
        return;
    }
    if (sht41_init(i2c_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SHT41");
        return;
    }

    // 5) BCH environment: we no longer call protocol_init_bch() from protocol.c;
    //    Instead, we call bch_init_2bit() from our new bch.c library.
    g_bch = bch_init_2bit(false);  // pass true if you want each data/ECC byte bit-reversed
    if (!g_bch) {
        ESP_LOGE(TAG, "Failed to initialize BCH(1023,1003) 2-bit ECC context");
        return;
    }

    ///////////////////////////////////////////////
    // Read sensor serials once
    ///////////////////////////////////////////////
    uint8_t bme280_serial[8];
    uint8_t tmp117_serial[8];
    uint8_t aht20_serial[8];
    uint8_t sht41_serial[8];

    bme280_get_serial(bme280_serial);
    tmp117_get_serial(tmp117_serial);
    aht20_get_serial(aht20_serial);
    sht41_get_serial(sht41_serial);

    ESP_LOGI(TAG, "BME280 serial: %02X %02X %02X %02X %02X %02X %02X %02X",
             bme280_serial[0], bme280_serial[1], bme280_serial[2], bme280_serial[3],
             bme280_serial[4], bme280_serial[5], bme280_serial[6], bme280_serial[7]);

    ESP_LOGI(TAG, "TMP117 serial: %02X %02X %02X %02X %02X %02X %02X %02X",
             tmp117_serial[0], tmp117_serial[1], tmp117_serial[2], tmp117_serial[3],
             tmp117_serial[4], tmp117_serial[5], tmp117_serial[6], tmp117_serial[7]);

    ESP_LOGI(TAG, "AHT20 serial : %02X %02X %02X %02X %02X %02X %02X %02X",
             aht20_serial[0], aht20_serial[1], aht20_serial[2], aht20_serial[3],
             aht20_serial[4], aht20_serial[5], aht20_serial[6], aht20_serial[7]);

    ESP_LOGI(TAG, "SHT41 serial: %02X %02X %02X %02X %02X %02X %02X %02X",
             sht41_serial[0], sht41_serial[1], sht41_serial[2], sht41_serial[3],
             sht41_serial[4], sht41_serial[5], sht41_serial[6], sht41_serial[7]);

    ///////////////////////////////////////////////
    // Build 12-byte MCU serial from 6-byte MAC
    ///////////////////////////////////////////////
    uint8_t raw_mac[6];
    esp_read_mac(raw_mac, ESP_MAC_WIFI_STA);

    uint8_t mcu_serial[12];
    memset(mcu_serial, 0, 12);
    memcpy(&mcu_serial[6], raw_mac, 6);

    ESP_LOGI(TAG, "MCU 12-byte serial => zero left + MAC =>");
    ESP_LOG_BUFFER_HEXDUMP(TAG, mcu_serial, 12, ESP_LOG_INFO);

    ///////////////////////////////////////////////
    // Main loop
    ///////////////////////////////////////////////
    uint32_t counter = 0;

    float bme_temp= NAN, bme_press= NAN, bme_hum= NAN;
    float tmp117_temp= NAN;
    float aht_temp= NAN, aht_hum= NAN;
    float sht_temp= NAN, sht_hum= NAN;

    while (true) {
        counter++;

        // read sensors
        if (bme280_read(&bme_temp, &bme_press, &bme_hum) != ESP_OK) {
            ESP_LOGW(TAG, "BME280 read failed");
            bme_temp=bme_press=bme_hum= NAN;
        }
        if (tmp117_read(&tmp117_temp) != ESP_OK) {
            ESP_LOGW(TAG, "TMP117 read failed");
            tmp117_temp= NAN;
        }
        if (aht20_read(&aht_temp, &aht_hum) != ESP_OK) {
            ESP_LOGW(TAG, "AHT20 read failed");
            aht_temp=aht_hum= NAN;
        }
        if (sht41_read(&sht_temp, &sht_hum) != ESP_OK) {
            ESP_LOGW(TAG, "SHT41 read failed");
            sht_temp=sht_hum= NAN;
        }

        // build 4-byte msg ID
        uint8_t message_id[4];
        pack_message_id(message_id, counter);

        bool is_full = ((counter % 1000) == 0);

        // Build raw frame data: call minimal or full, ignoring its old BFS-based ECC
        // (We keep them for the sensor data layout, but we won't call protocol_add_ecc())
        uint8_t frame_buf[128];
        size_t  frame_len=0;

        if (!is_full) {
            // minimal
            protocol_build_minimal_frame(
                frame_buf, &frame_len,
                message_id,
                s_location_id,
                bme_temp, bme_press, bme_hum,
                tmp117_temp,
                aht_temp, aht_hum,
                sht_temp, sht_hum
            );
        } else {
            // full
            protocol_build_full_frame(
                frame_buf, &frame_len,
                message_id,
                s_location_id,
                MCU_TYPE,
                mcu_serial,
                FW_MAJOR,
                FW_MINOR,
                bme280_serial, bme_temp, bme_press, bme_hum,
                tmp117_serial, tmp117_temp,
                aht20_serial, aht_temp, aht_hum,
                sht41_serial, sht_temp, sht_hum
            );
        }

        // Now pad to 125 bytes if needed
        while (frame_len < 125) {
            frame_buf[frame_len++] = 0x00;
        }
        // We do not call protocol_add_ecc() any more, because that code uses BFS single-bit approach.
        // Instead we do bch_encode() from our new library.

        // 3 bytes of ECC
        uint8_t ecc[3];
        memset(ecc, 0, sizeof(ecc)); // must be zeroed on first usage
        // call bch_encode => processes the 125 bytes
        bch_encode(g_bch, frame_buf, 125, ecc);

        // Now append the 3 ECC bytes to the frame
        memcpy(&frame_buf[frame_len], ecc, 3);
        frame_len += 3;

        // publish via MQTT
        mqtt_publish_binary(frame_buf, frame_len);

        ESP_LOGI(TAG, "#%u => %s frame, len=%u", (unsigned)counter,
                 is_full ? "FULL" : "MINIMAL", (unsigned)frame_len);

        // Wait 2s
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // If we ever exit => bch_free(g_bch);
}
