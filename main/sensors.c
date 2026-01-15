#include "sensors.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

void sensors_init(void) {
    // Bumpers
    gpio_config_t bumper_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUMPER_FRONT_LEFT) | (1ULL << BUMPER_FRONT_RIGHT) |
                        (1ULL << BUMPER_REAR_LEFT) | (1ULL << BUMPER_REAR_RIGHT),
        .pull_down_en = 0,
        .pull_up_en = 1 // Attempt internal pull-up. If input-only pin, this is ignored and external needed.
    };
    gpio_config(&bumper_conf);

    // Ultrasonic Trigger (Output)
    gpio_config_t trig_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << US_TRIGGER_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&trig_conf);
    gpio_set_level(US_TRIGGER_PIN, 0);

    // Ultrasonic Echo (Input)
    gpio_config_t echo_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << US_ECHO_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&echo_conf);
}

float sensors_get_distance_cm(void) {
    // 1. Send Trigger
    gpio_set_level(US_TRIGGER_PIN, 1);
    ets_delay_us(10);
    gpio_set_level(US_TRIGGER_PIN, 0);

    // 2. Wait for Echo High
    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_wait > 20000) return -1; // Timeout waiting for start
    }

    // 3. Measure High Time
    int64_t start_pulse = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - start_pulse > 25000) return -1; // Timeout waiting for end (approx 400cm)
    }
    int64_t end_pulse = esp_timer_get_time();

    // 4. Calculate Distance
    // Speed of sound ~= 343m/s = 0.0343 cm/us
    // Distance = (Time * Speed) / 2
    float distance = ((end_pulse - start_pulse) * 0.0343) / 2.0;
    return distance;
}

bool sensors_is_bumper_hit(void) {
    // Assuming Active Low (pressed connects to ground)
    // Returns true if ANY bumper is LOW (0)
    if (gpio_get_level(BUMPER_FRONT_LEFT) == 0) return true;
    if (gpio_get_level(BUMPER_FRONT_RIGHT) == 0) return true;
    if (gpio_get_level(BUMPER_REAR_LEFT) == 0) return true;
    if (gpio_get_level(BUMPER_REAR_RIGHT) == 0) return true;
    return false;
}
