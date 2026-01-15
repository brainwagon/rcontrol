#include "motors.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

// PWM Configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_LEFT     MOTOR_LEFT_ENA_PIN
#define LEDC_OUTPUT_IO_RIGHT    MOTOR_RIGHT_ENB_PIN
#define LEDC_CHANNEL_LEFT       LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT      LEDC_CHANNEL_1
#define LEDC_DUTY_RES           MOTOR_PWM_RES_BITS
#define LEDC_FREQUENCY          MOTOR_PWM_FREQ_HZ

void motors_init(void) {
    // Configure Direction Pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_LEFT_IN1_PIN) | (1ULL << MOTOR_LEFT_IN2_PIN) |
                        (1ULL << MOTOR_RIGHT_IN3_PIN) | (1ULL << MOTOR_RIGHT_IN4_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Configure PWM Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM Channels
    ledc_channel_config_t ledc_channel_left = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_LEFT,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_left);

    ledc_channel_config_t ledc_channel_right = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_RIGHT,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_right);
}

static uint32_t speed_to_duty(int speed) {
    if (speed < 0) speed = -speed;
    if (speed > 100) speed = 100;
    
    // Map 0-100 to 0-1023 (10 bit resolution)
    return (uint32_t)((speed / 100.0) * ((1 << MOTOR_PWM_RES_BITS) - 1));
}

void motors_set_speed(int left_speed, int right_speed) {
    // Left Motor Logic
    if (left_speed > 0) {
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 1);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
    } else if (left_speed < 0) {
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 0);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 1);
    } else {
        gpio_set_level(MOTOR_LEFT_IN1_PIN, 0);
        gpio_set_level(MOTOR_LEFT_IN2_PIN, 0);
    }

    // Right Motor Logic
    if (right_speed > 0) {
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 1);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);
    } else if (right_speed < 0) {
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 1);
    } else {
        gpio_set_level(MOTOR_RIGHT_IN3_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_IN4_PIN, 0);
    }

    // Update PWM
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, speed_to_duty(left_speed));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, speed_to_duty(right_speed));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
}

void motors_stop(void) {
    motors_set_speed(0, 0);
}
