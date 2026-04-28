# ---- Sensor build switches (set ON/OFF per firmware image) ----
option(ENVSENSOR_USE_BME280 "Enable BME280 sensor code" ON)
option(ENVSENSOR_USE_SHT4X  "Enable SHT4X sensor code"  ON)

# Convert ON/OFF to numeric 1/0 for the C header
set(ENVSENSOR_USE_BME280_VALUE 0)
if(ENVSENSOR_USE_BME280)
    set(ENVSENSOR_USE_BME280_VALUE 1)
endif()

set(ENVSENSOR_USE_SHT4X_VALUE 0)
if(ENVSENSOR_USE_SHT4X)
    set(ENVSENSOR_USE_SHT4X_VALUE 1)
endif()

# ---- Generate sensors_enable.h from the options ----
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/sensors_enable.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/sensors_enable.h
    @ONLY
)

# ---- Base deps ----
set(PRIV_REQS
    wifi_sta
    mqtt_push
        esp_temp
)

# Only require i2c + sensor components if enabled
if(ENVSENSOR_USE_BME280 OR ENVSENSOR_USE_SHT4X)
    list(APPEND PRIV_REQS i2c)
endif()

if(ENVSENSOR_USE_BME280)
    list(APPEND PRIV_REQS bme280)
endif()
if(ENVSENSOR_USE_SHT4X)
    list(APPEND PRIV_REQS sht4x)
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
    PRIV_REQUIRES ${PRIV_REQS}
)
