#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize sensor GPIOs (Bumpers + Ultrasonic).
 */
void sensors_init(void);

/**
 * @brief Get the distance in centimeters from the ultrasonic sensor.
 * @return Distance in cm, or -1 if timeout/error.
 */
float sensors_get_distance_cm(void);

/**
 * @brief Check if any bumper is pressed.
 * @return true if collision detected.
 */
bool sensors_is_bumper_hit(void);

#endif // SENSORS_H
