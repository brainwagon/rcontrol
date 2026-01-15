idf_component_register(SRCS "main.c" "motors.c" "lights.c" "sensors.c" "gamepad.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_hid bt esp_driver_gpio esp_driver_ledc esp_timer esp_partition esp_event)