idf_component_register(SRCS "main.c" "cc1101.c" "tpms_decode.c" "tx_test.c" "tires.c" "feedback.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_usb_serial_jtag esp_timer nvs_flash)
