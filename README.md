# esp32-envsensor-mqtt

## ESP32 Multi-Sensor Telemetry with digital message format, BCH ECC and MQTT

This document describes a project that integrates multiple I2C sensors on an ESP32, encodes their sensor readings in a custom binary format with 2‑bit BCH ECC, and publishes the data via MQTT. 
Temperature, pressure and humidity from Bosch BME280, SHT41, AHT20 and TI TMP117 sensors are read.
It also includes Wi-Fi (station mode) for network connectivity.

---
## 📁 Project Directory Structure
```
esp32-sensor-mqtt/
├── CMakeLists.txt
├── sdkconfig
├── sdkconfig.defaults
├── main/
│   ├── main.c
│   └── main.h
├── components/
│   ├── wifi_sta/
│   │   ├── wifi_sta.c
│   │   └── wifi_sta.h
│   ├── mqtt_push/
│   │   ├── mqtt_push.c
│   │   └── mqtt_push.h
│   ├── i2c/
│   │   ├── i2c.c
│   │   └── i2c.h
│   ├── aht20/
│   │   ├── aht20.c
│   │   └── aht20.h
│   ├── bme280/
│   │   ├── bme280.c
│   │   └── bme280.h
│   ├── tmp117/
│   │   ├── tmp117.c
│   │   └── tmp117.h
│   ├── sht41/
│   │   ├── sht41.c
│   │   └── sht41.h
│   ├── protocol/
│   │   ├── protocol.c
│   │   └── protocol.h
│   └── bch/
│       ├── bch.c
│       └── bch.h
...
```


## Overall Architecture

1. **Main Application** (`main/main.c`)

   - **Wi-Fi Initialization**  
     Configures the ESP32 in station mode and connects to your network.
   
   - **MQTT Client Setup**  
     Creates an MQTT client to publish data to a broker (e.g., `mqtt://192.168.1.x:1883`).
   
   - **I2C & Sensor Initialization**  
     Sets up the I2C bus and attaches each sensor (BME280, TMP117, AHT20, SHT41).

   - **Telemetry Loop**  
     Repeatedly (every 2 seconds):
     1. Reads temperature/humidity/pressure from the sensors.
     2. Builds a binary data “frame” in either a _minimal_ or _full_ format.
     3. Encodes that frame with a 2-bit BCH(1023,1003) ECC (adds 3 ECC bytes).
     4. Publishes the final 128-byte payload (`125 + 3 ECC`) to MQTT on topic `test/binary`.

2. **Component Libraries**

   - **`wifi_sta`**  
     Responsible for connecting the ESP32 to your Wi-Fi network in station mode (with a hardcoded SSID/password).

   - **`mqtt_push`**  
     Initializes the MQTT client and provides a function to publish raw binary data to a given topic.

   - **`i2c`**  
     A wrapper around ESP-IDF’s I2C driver. Allows adding devices by address and reading/writing easily.

   - **Sensor Drivers**  
     Each sensor driver configures the sensor, reads measurements, and provides a function to retrieve an 8-byte ID:
     - **`aht20`**: Humidity/temperature (no real serial → returns zeros).  
     - **`bme280`**: Temperature/pressure/humidity. Fakes an 8-byte ID by XORing calibration data.  
     - **`tmp117`**: Temperature-only device. Reads EEPROM for partial ID.  
     - **`sht41`**: Humidity/temperature. Retrieves a 4-byte hardware ID from command `0x89` (embedded in an 8-byte output).

   - **`bch`**  
     Implements a custom 2-bit BCH(1023,1003) encoder, producing 20 bits of ECC (3 bytes) for data up to 125 bytes.  

   - **`protocol`**  
     Builds either a “minimal” frame (no sensor serials, minimal metadata) or a “full” frame (includes MCU details and sensor serials).  
     The frame layout is standardized here, but **ECC is now handled by `bch_encode`** instead of older BFS code.

---

## Key Components and Files

### 1. `main/main.c`
**Purpose**  
- Entry point (`app_main`).
- Connects to Wi-Fi, starts MQTT, initializes I2C, initializes all sensors, then runs a 2-second loop to:
  1. Read sensor data  
  2. Build a binary frame (using `protocol_build_minimal_frame` or `protocol_build_full_frame`)  
  3. Use `bch_encode` to produce 3 bytes of ECC  
  4. Append ECC and publish via MQTT  

**Notable Details**  
- Uses a global BCH context (`g_bch`) from `bch_init_2bit(false)`.  
- Reads each sensor’s 8-byte “serial” once at startup and includes it in the “full” frame every 1000th iteration.  
- Has a 6-byte `LOCATION_ID` plus a 4-byte message counter in big-endian.  
- Minimal frames are used most of the time; “full” frames are sent periodically for extended metadata.

### 2. `wifi_sta` Component
- **Files**: `wifi_sta.[ch]`  
- **Function**:  
  - Sets up the ESP32 in station mode with `ssid`/`password` hard-coded in `wifi_sta.c`.  
  - Calls `nvs_flash_init`, configures `esp_wifi`, and executes `esp_wifi_connect()`.

### 3. `mqtt_push` Component
- **Files**: `mqtt_push.[ch]`  
- **Function**:  
  - Configures the MQTT client with a given URI (also hardcoded).  
  - Exposes `mqtt_publish_binary` to push binary frames to `test/binary`.

### 4. `i2c` Component
- **Files**: `i2c.[ch]`  
- **Function**:  
  - Provides `i2c_master_init(...)` to initialize the bus at pins 21 (SDA) and 22 (SCL) at 100kHz.  
  - `i2c_add_device(bus, dev_addr, ...)` to attach a device to the bus.  
  - `i2c_write(...)` and `i2c_read(...)` for data transfer.

### 5. Sensor Drivers

Each driver has:

- `init(...)`: Adds the device to the bus, sets up configuration.
- `read(...)`: Fetches data (temperature, humidity, etc.).
- `get_serial(...)`: Returns an 8-byte array with either real or placeholder ID data.

**AHT20**  
- I2C address: `0x38`.  
- `aht20_read`: writes a trigger command, waits, then reads 6 bytes → temperature & RH.

**BME280**  
- I2C address: `0x77`.  
- Reads calibration registers, does standard compensation for T/P/H.  
- `bme280_get_serial`: XORs calibration data for a pseudo-ID.

**TMP117**  
- I2C address: `0x48`.  
- Continuous conversion mode, reading 2 bytes for temperature.  
- Serial is derived from EEPROM registers `0x05`, `0x06`, `0x08`.

**SHT41**  
- I2C address: `0x44`.  
- High-precision command `0xFD`, read 6 bytes (temp, hum, CRC).  
- Serial is read with command `0x89` → merges 4 ID bytes into the last half of an 8-byte array.

### 6. `protocol` Component
- **Files**: `protocol.[ch]`  
- **Function**:  
  - Arranges sensor data into a “minimal” or “full” frame.  
  - Minimal has 4 sensor blocks (BME280, TMP117, AHT20, SHT41) and no extra fields.  
  - Full adds MCU type, 12-byte MCU serial, firmware version, and includes each sensor’s 8-byte serial in the sensor blocks.  
  - ECC is no longer generated here; you simply call `bch_encode` after building the 125-byte portion.

### 7. `bch` Component
- **Files**: `bch.[ch]`  
- **Function**:  
  - `bch_init_2bit(bool swap_bits)` → create a context.  
  - `bch_encode(context, data, data_len, ecc)` → for up to 125 bytes, updates a 3-byte ECC buffer.  
  - Freed via `bch_free(context)`.  
- Uses polynomial `0x10031` for a 20-bit remainder.

### 8. Transition from Older BFS Code
- `bch.c` is the new approach for 2-bit ECC.  
- `protocol.c` had older BFS code for single-bit ECC; now replaced with calls to `bch_encode`.

---

## Typical Data Flow

1. **Initialization**  
   - `app_main` calls `wifi_init()`, `mqtt_init()`, sets up I2C, and initializes all sensors.

2. **Loop**  
   - Reads BME280, TMP117, AHT20, SHT41 data.
   - Builds either a minimal or full frame (size up to 125 bytes).  
   - Zeroes a 3-byte ECC array and calls `bch_encode`.
   - Appends ECC → final 128 bytes.
   - Publishes to MQTT (`test/binary`).
   - Delays 2s, repeats.

---

## Summary

- **Purpose**: Showcases an ESP32 reading multiple I2C sensors, encoding frames with 2-bit BCH ECC, and publishing via MQTT.  
- **Architecture**: Divided into multiple components: Wi-Fi, MQTT, I2C, sensor drivers, protocol, BCH.  
- **Key Points**:  
  1. Minimal vs. full frames (with/without sensor serials, MCU info).  
  2. Always produce 125 data bytes + 3 ECC bytes = 128 total.  
  3. `bch_encode` handles the 2-bit ECC generation.  
  4. Publishes raw binary frames to an MQTT broker.

This design demonstrates a robust approach to sensor data telemetry on the ESP32, combining reliability (ECC) with flexible data layout (minimal/full frames) and standard connectivity (MQTT).

