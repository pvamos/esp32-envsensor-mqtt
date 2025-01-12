idf_component_register(
    SRCS "httpd.c"
    INCLUDE_DIRS "."
    REQUIRES bme280 tmp117 aht20 esp_http_server
)
