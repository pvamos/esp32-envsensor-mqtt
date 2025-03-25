#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// We define some protocol-related constants for the app
// location ID is 6 bytes
#define LOCATION_ID { 10, 20, 30, 40, 50, 60 }

// MCU type, firmware version
#define MCU_TYPE 1
#define FW_MAJOR 1
#define FW_MINOR 2

/**
 * @brief The ESP-IDF entry point
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
