#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gpio_in1;
    int gpio_in2;
    int gpio_ena; // PWM
    int gpio_in3;
    int gpio_in4;
    int gpio_enb; // PWM
    int pwm_freq_hz;
} motor_driver_config_t;

/**
 * @brief Initialize the L298 Motor Driver
 * 
 * @param config Configuration structure
 * @return esp_err_t 
 */
esp_err_t motor_driver_init(const motor_driver_config_t *config);

/**
 * @brief Set the speed and direction of the motors.
 * 
 * @param left_speed -1.0 (Full Reverse) to 1.0 (Full Forward)
 * @param right_speed -1.0 (Full Reverse) to 1.0 (Full Forward)
 */
void motor_driver_set_speed(float left_speed, float right_speed);

#ifdef __cplusplus
}
#endif
