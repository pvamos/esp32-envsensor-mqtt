#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// Define protocol constants
#define LOCATION_ID  { 1, 2, 3, 4, 5 }   // 5 bytes
#define MCU_TYPE     1                  // e.g. 1=ESP32
#define FW_MAJOR     1
#define FW_MINOR     2

/**
 * @brief ESP-IDF entry point.
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
