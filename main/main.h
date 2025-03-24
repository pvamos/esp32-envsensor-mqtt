#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// 2 magic bytes: 'S'(0x53), 'N'(0x4E)
#define MAGIC_BYTE1 0x53
#define MAGIC_BYTE2 0x4E

// 6-byte location ID
#define LOCATION_ID  { 10, 20, 30, 40, 50, 60 }

// We'll define a simple MCU_TYPE, FW_MAJOR, FW_MINOR if needed
#define MCU_TYPE  1
#define FW_MAJOR  1
#define FW_MINOR  2

/**
 * @brief The main application entry point.
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
