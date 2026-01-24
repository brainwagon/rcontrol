#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_data_t;

/**
 * @brief Initialize the MPU6050 sensor
 * 
 * @param i2c_num I2C port number
 * @param i2c_addr I2C address of the device (0x68 or 0x69)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num, uint8_t i2c_addr);

/**
 * @brief Read all sensor data (Accel, Gyro, Temp)
 * 
 * @param data Pointer to mpu6050_data_t struct to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Wake up the MPU6050 (exit sleep mode)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_wake(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
