#include "lights.h"
#include "config.h"
#include "driver/gpio.h"

void lights_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_LEFT_TURN) | (1ULL << LED_RIGHT_TURN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Start off
    gpio_set_level(LED_LEFT_TURN, 0);
    gpio_set_level(LED_RIGHT_TURN, 0);
}

void lights_set_left(bool on) {
    gpio_set_level(LED_LEFT_TURN, on ? 1 : 0);
}

void lights_set_right(bool on) {
    gpio_set_level(LED_RIGHT_TURN, on ? 1 : 0);
}

void lights_toggle_left(void) {
    gpio_set_level(LED_LEFT_TURN, !gpio_get_level(LED_LEFT_TURN));
}

void lights_toggle_right(void) {
    gpio_set_level(LED_RIGHT_TURN, !gpio_get_level(LED_RIGHT_TURN));
}
