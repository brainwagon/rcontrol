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
// Note: Changed to pins with internal Pull-Ups to avoid floating inputs
// Moved Rear bumpers to 16/17 to free up 21/22 for I2C
#define BUMPER_FRONT_LEFT   32
#define BUMPER_FRONT_RIGHT  13

// I2C Configuration
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         100000

// Lights (LEDs)
#define LED_LEFT_TURN       5
#define LED_RIGHT_TURN      4
#define LED_HEARTBEAT_PIN   2

#endif // CONFIG_H
