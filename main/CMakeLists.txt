idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES wifi_sta mqtt_push i2c bme280 tmp117 aht20 sht41
)

