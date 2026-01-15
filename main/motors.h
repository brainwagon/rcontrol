#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

/**
 * @brief Initialize motor control GPIOs and PWM.
 */
void motors_init(void);

/**
 * @brief Set the speed and direction of the motors.
 * 
 * @param left_speed Speed for left motors (-100 to 100). Negative is reverse.
 * @param right_speed Speed for right motors (-100 to 100). Negative is reverse.
 */
void motors_set_speed(int left_speed, int right_speed);

/**
 * @brief Stop all motors immediately.
 */
void motors_stop(void);

#endif // MOTORS_H
