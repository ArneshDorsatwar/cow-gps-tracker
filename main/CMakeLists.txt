idf_component_register(SRCS "main.c" "nmea_parser.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_event esp_wifi esp_netif esp_http_client nvs_flash
                       PRIV_REQUIRES esp_driver_uart esp_driver_i2c esp_driver_spi esp_driver_gpio)
