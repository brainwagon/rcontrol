#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "motors.h"
#include "lights.h"
#include "sensors.h"
#include "gamepad.h"
#include "wifi_connect.h"
#include "web_server.h"
#include "config.h"

static const char *TAG = "MAIN";

// Custom log vprintf to redirect logs to Web Server
int web_log_vprintf(const char *fmt, va_list args) {
    // Send to web server
    web_server_broadcast_log(fmt, args);
    // Also print to standard UART
    return vprintf(fmt, args);
}

// Mixing logic for Arcade Drive (Skid Steering)
void calculate_skid_steering(int joy_x, int joy_y, int *left_out, int *right_out) {
    // joy_x, joy_y are -127 to 127
    // Scale to -100 to 100
    float throttle = (float)joy_y / 1.27f; // -100 to 100
    float steering = (float)joy_x / 1.27f; // -100 to 100

    // Invert throttle if necessary (Gamepads often send -127 for UP)
    // Assuming here: Up is negative (standard HID), so invert.
    throttle = -throttle; 

    float left = throttle + steering;
    float right = throttle - steering;

    // Clamp
    if (left > 100) left = 100;
    if (left < -100) left = -100;
    if (right > 100) right = 100;
    if (right < -100) right = -100;

    *left_out = (int)left;
    *right_out = (int)right;
}

void app_main(void) {
    // 1. Initialize NVS (Required for Bluetooth & WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Logs Redirection
    esp_log_set_vprintf(web_log_vprintf);

    // 3. Initialize Hardware
    ESP_LOGI(TAG, "Initializing Hardware...");
    motors_init();
    lights_init();
    sensors_init();
    
    // 4. Initialize Wi-Fi & Web Server
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    wifi_init_sta();
    web_server_init();

    // 5. Initialize Gamepad (Bluetooth)
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    gamepad_init();

    ESP_LOGI(TAG, "System Ready. Waiting for controller...");

    int left_speed = 0;
    int right_speed = 0;

    while (1) {
        bool hit_fl = gpio_get_level(BUMPER_FRONT_LEFT) == 0;
        bool hit_fr = gpio_get_level(BUMPER_FRONT_RIGHT) == 0;
        bool hit_rl = gpio_get_level(BUMPER_REAR_LEFT) == 0;
        bool hit_rr = gpio_get_level(BUMPER_REAR_RIGHT) == 0;
        bool any_bumper = hit_fl || hit_fr || hit_rl || hit_rr;

        // Safety: Check bumpers
        if (any_bumper) {
            motors_stop();
            lights_set_left(true);
            lights_set_right(true);
            
            // Update Web Interface (Crash status)
            web_server_update_status(0, 0, hit_fl, hit_fr, hit_rl, hit_rr, true, true, gamepad_get_state().connected);

            vTaskDelay(pdMS_TO_TICKS(100));
            lights_set_left(false);
            lights_set_right(false);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; 
        }

        gamepad_state_t state = gamepad_get_state();

        if (state.connected) {
            calculate_skid_steering(state.joy_x, state.joy_y, &left_speed, &right_speed);
            
            if (abs(left_speed) < 10) left_speed = 0;
            if (abs(right_speed) < 10) right_speed = 0;

            motors_set_speed(left_speed, right_speed);

            // Turn signals
            bool led_l = false;
            bool led_r = false;
            if (state.joy_x < -50) { 
                 lights_toggle_left();
                 lights_set_right(false);
                 led_l = gpio_get_level(LED_LEFT_TURN);
            } else if (state.joy_x > 50) { 
                 lights_toggle_right();
                 lights_set_left(false);
                 led_r = gpio_get_level(LED_RIGHT_TURN);
            } else {
                 lights_set_left(false);
                 lights_set_right(false);
            }
            
            // Web Update
            web_server_update_status(left_speed, right_speed, hit_fl, hit_fr, hit_rl, hit_rr, led_l, led_r, true);

        } else {
            motors_stop();
            // Blink slowly to indicate waiting
            bool blink = (xTaskGetTickCount() % 100) < 50;
            lights_set_left(blink);
            
            web_server_update_status(0, 0, hit_fl, hit_fr, hit_rl, hit_rr, blink, false, false);
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}