#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h" // for esp_read_mac()
#include <string.h>
#include <math.h>       // for NAN

#include "wifi_sta.h"
#include "mqtt_push.h"
#include "i2c.h"

// sensor drivers
#include "bme280.h"
#include "tmp117.h"
#include "aht20.h"
#include "sht41.h"

// binary protocol
#include "protocol.h"

static const char *TAG = "MAIN";

// Use definitions from main.h
static const uint8_t s_location_id[5] = LOCATION_ID;

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

    // 5) Initialize BCH environment for protocol
    protocol_init_bch();

    // 6) Read sensor serials once
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

    ESP_LOGI(TAG, "AHT20 serial:  %02X %02X %02X %02X %02X %02X %02X %02X",
             aht20_serial[0], aht20_serial[1], aht20_serial[2], aht20_serial[3],
             aht20_serial[4], aht20_serial[5], aht20_serial[6], aht20_serial[7]);

    ESP_LOGI(TAG, "SHT41 serial:  %02X %02X %02X %02X %02X %02X %02X %02X",
             sht41_serial[0], sht41_serial[1], sht41_serial[2], sht41_serial[3],
             sht41_serial[4], sht41_serial[5], sht41_serial[6], sht41_serial[7]);

    // 7) Read ESP32 MAC => produce 12-byte serial
    // left-pad with 6 bytes of 0, then copy the 6 mac bytes
    uint8_t raw_mac[6];
    esp_read_mac(raw_mac, ESP_MAC_WIFI_STA);

    uint8_t mcu_serial[12];
    memset(mcu_serial, 0, 12);  // fill with zeros

    // place 6 mac bytes in the last half
    memcpy(&mcu_serial[6], raw_mac, 6);

    ESP_LOGI(TAG, "MCU 12-byte serial = [0..5=0], [6..11=MAC]");
    ESP_LOG_BUFFER_HEXDUMP(TAG, mcu_serial, 12, ESP_LOG_INFO);

    // sensor data
    float bme280_temp = NAN, bme280_press = NAN, bme280_hum = NAN;
    float tmp117_temp = NAN;
    float aht20_temp  = NAN, aht20_hum = NAN;
    float sht41_temp  = NAN, sht41_hum = NAN;

    uint32_t message_count = 0;

    // main loop
    while (true) {
        message_count++;

        // read sensors
        if (bme280_read(&bme280_temp, &bme280_press, &bme280_hum) != ESP_OK) {
            ESP_LOGW(TAG, "BME280 read failed");
            bme280_temp = bme280_press = bme280_hum = NAN;
        }
        if (tmp117_read(&tmp117_temp) != ESP_OK) {
            ESP_LOGW(TAG, "TMP117 read failed");
            tmp117_temp = NAN;
        }
        if (aht20_read(&aht20_temp, &aht20_hum) != ESP_OK) {
            ESP_LOGW(TAG, "AHT20 read failed");
            aht20_temp = aht20_hum = NAN;
        }
        if (sht41_read(&sht41_temp, &sht41_hum) != ESP_OK) {
            ESP_LOGW(TAG, "SHT41 read failed");
            sht41_temp = sht41_hum = NAN;
        }

        bool full_frame = ((message_count % 1000) == 0);

        // build frame
        uint8_t frame_buf[128];
        size_t  frame_len = 0;

        if (!full_frame) {
            // minimal
            protocol_build_minimal_frame(
                frame_buf, &frame_len,
                (uint16_t)(message_count & 0xFFFF),
                s_location_id,
                bme280_temp, bme280_press, bme280_hum,
                tmp117_temp
            );
        } else {
            // full frame
            protocol_build_full_frame(
                frame_buf, &frame_len,
                (uint16_t)(message_count & 0xFFFF),
                s_location_id,
                MCU_TYPE,
                mcu_serial, 
                FW_MAJOR,
                FW_MINOR,
                bme280_temp, bme280_press, bme280_hum,
                aht20_temp, aht20_hum
            );
        }

        // add ECC
        protocol_add_ecc(frame_buf, &frame_len);

        // publish
        mqtt_publish_binary(frame_buf, frame_len);

        ESP_LOGI(TAG, "#%u %s frame, length=%u", 
                 (unsigned)message_count,
                 full_frame ? "FULL" : "MINIMAL",
                 (unsigned)frame_len);

        // Wait 2s
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
