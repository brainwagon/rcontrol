#ifndef CONFIG_H
#define CONFIG_H

// Motor Control Pins (L298N)
#define MOTOR_LEFT_ENA_PIN  14
#define MOTOR_LEFT_IN1_PIN  27
#define MOTOR_LEFT_IN2_PIN  26

#define MOTOR_RIGHT_ENB_PIN 12
#define MOTOR_RIGHT_IN3_PIN 25
#define MOTOR_RIGHT_IN4_PIN 33

// Motor PWM Configuration
#define MOTOR_PWM_FREQ_HZ   1000
#define MOTOR_PWM_RES_BITS  LEDC_TIMER_10_BIT

// Ultrasonic Sensor (HC-SR04)
#define US_TRIGGER_PIN      18
#define US_ECHO_PIN         19
#define US_MAX_DIST_CM      400

// Bumper Microswitches (Active Low usually)
#define BUMPER_FRONT_LEFT   32
#define BUMPER_FRONT_RIGHT  35
#define BUMPER_REAR_LEFT    34
#define BUMPER_REAR_RIGHT   39

// Lights (LEDs)
#define LED_LEFT_TURN       2
#define LED_RIGHT_TURN      4

#endif // CONFIG_H
