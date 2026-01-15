#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "motors.h"
#include "lights.h"
#include "sensors.h"
#include "gamepad.h"

static const char *TAG = "MAIN";

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
    // 1. Initialize NVS (Required for Bluetooth)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Hardware
    ESP_LOGI(TAG, "Initializing Hardware...");
    motors_init();
    lights_init();
    sensors_init();
    
    // 3. Initialize Gamepad (Bluetooth)
    // Note: This starts the scan/connect process in background
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    gamepad_init();

    ESP_LOGI(TAG, "System Ready. Waiting for controller...");

    int left_speed = 0;
    int right_speed = 0;

    while (1) {
        // Safety: Check bumpers
        if (sensors_is_bumper_hit()) {
            // Collision! Stop and back up slightly?
            // For now, just stop and flash lights.
            motors_stop();
            lights_set_left(true);
            lights_set_right(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            lights_set_left(false);
            lights_set_right(false);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; // Skip the rest of the loop
        }

        gamepad_state_t state = gamepad_get_state();

        if (state.connected) {
            calculate_skid_steering(state.joy_x, state.joy_y, &left_speed, &right_speed);
            
            // Deadzone check (already in gamepad.c, but good to be safe)
            if (abs(left_speed) < 10) left_speed = 0;
            if (abs(right_speed) < 10) right_speed = 0;

            motors_set_speed(left_speed, right_speed);

            // Turn signals based on steering
            if (state.joy_x < -50) { // Turning Left
                 lights_toggle_left();
                 lights_set_right(false);
            } else if (state.joy_x > 50) { // Turning Right
                 lights_toggle_right();
                 lights_set_left(false);
            } else {
                 lights_set_left(false);
                 lights_set_right(false);
            }
            
            // Example: Use buttons for other features
            // if (state.buttons & 0x01) { ... }

        } else {
            // Not connected: Stop
            motors_stop();
            // Blink slowly to indicate waiting
            if ((xTaskGetTickCount() % 100) < 5) {
                lights_set_left(true);
            } else {
                lights_set_left(false);
            }
        }

        // Ultrasonic Debug (Optional, print every second)
        // static int64_t last_print = 0;
        // if (esp_timer_get_time() - last_print > 1000000) {
        //     float dist = sensors_get_distance_cm();
        //     ESP_LOGI(TAG, "Distance: %.2f cm", dist);
        //     last_print = esp_timer_get_time();
        // }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}
