#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t buttons;
    float left_stick_x;  // -1.0 to 1.0
    float left_stick_y;  // -1.0 to 1.0
    float right_stick_x; // -1.0 to 1.0
    float right_stick_y; // -1.0 to 1.0
    float left_trigger;  // 0.0 to 1.0
    float right_trigger; // 0.0 to 1.0
} gamepad_state_t;

typedef enum {
    GAMEPAD_STATUS_SCANNING,
    GAMEPAD_STATUS_CONNECTING,
    GAMEPAD_STATUS_CONNECTED,
    GAMEPAD_STATUS_DISCONNECTED
} gamepad_status_t;

typedef void (*gamepad_input_callback_t)(const gamepad_state_t *state);
typedef void (*gamepad_connection_callback_t)(gamepad_status_t status);

/**
 * @brief Initialize the Gamepad Bluetooth/HID subsystem.
 * 
 * This starts the BT stack, scanning, and handling of connections.
 * 
 * @return esp_err_t 
 */
esp_err_t gamepad_init(void);

/**
 * @brief Register a callback for input events.
 * 
 * @param cb Callback function
 */
void gamepad_set_input_callback(gamepad_input_callback_t cb);

/**
 * @brief Register a callback for connection status events.
 * 
 * @param cb Callback function
 */
void gamepad_set_connection_callback(gamepad_connection_callback_t cb);

#ifdef __cplusplus
}
#endif
