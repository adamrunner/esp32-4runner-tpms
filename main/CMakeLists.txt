idf_component_register(SRCS "main.c" "cc1101.c" "tpms_decode.c" "tx_test.c" "tires.c" "feedback.c" "web.c" "wifi.c" "logbuf.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "web/index.html" "web/log.html" "web/wifi.html"
                       PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_usb_serial_jtag esp_timer nvs_flash
                                     esp_wifi esp_netif esp_event esp_http_server)
