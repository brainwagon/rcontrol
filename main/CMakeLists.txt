idf_component_register(SRCS "main.c" "motors.c" "lights.c" "sensors.c" "gamepad.c" "wifi_connect.c" "web_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_hid bt esp_driver_gpio esp_driver_ledc esp_timer esp_partition esp_event esp_wifi esp_http_server lwip cjson)