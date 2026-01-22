idf_component_register(SRCS "esp32_8bitdo_gamepad.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_hid bt esp_common esp_event)