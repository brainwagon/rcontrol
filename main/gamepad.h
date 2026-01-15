#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int joy_x;       // -127 to 127
    int joy_y;       // -127 to 127
    uint32_t buttons; // Bitmask of buttons
    bool connected;
} gamepad_state_t;

/**
 * @brief Initialize Bluetooth HID Host.
 */
void gamepad_init(void);

/**
 * @brief Get the current gamepad state.
 * @return Copy of the state.
 */
gamepad_state_t gamepad_get_state(void);

#endif // GAMEPAD_H
