#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

typedef void* mpu6050_handle_t;

typedef struct {
    int16_t acce_x;
    int16_t acce_y;
    int16_t acce_z;
} mpu6050_acce_value_t;

typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_gyro_value_t;

typedef struct {
    float temp;
} mpu6050_temp_value_t;

mpu6050_handle_t mpu6050_create(i2c_port_t port, uint16_t dev_addr);
void mpu6050_wake_up(mpu6050_handle_t handle);
void mpu6050_get_acce(mpu6050_handle_t handle, mpu6050_acce_value_t *acce);
void mpu6050_get_gyro(mpu6050_handle_t handle, mpu6050_gyro_value_t *gyro);
void mpu6050_get_temp(mpu6050_handle_t handle, mpu6050_temp_value_t *temp);

#endif
