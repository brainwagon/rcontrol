#include "mpu6050.h"
#include "esp_log.h"
#include <stdlib.h>

#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_TEMP_XOUT_H         0x41
#define MPU6050_GYRO_XOUT_H         0x43

typedef struct {
    i2c_port_t port;
    uint16_t addr;
} mpu6050_dev_t;

mpu6050_handle_t mpu6050_create(i2c_port_t port, uint16_t dev_addr) {
    mpu6050_dev_t *dev = malloc(sizeof(mpu6050_dev_t));
    dev->port = port;
    dev->addr = dev_addr;
    return (mpu6050_handle_t)dev;
}

static esp_err_t mpu6050_read(mpu6050_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    if (!dev) return ESP_FAIL;
    return i2c_master_write_read_device(dev->port, dev->addr, &reg, 1, data, len, 100 / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_write_byte(mpu6050_handle_t handle, uint8_t reg, uint8_t data) {
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    if (!dev) return ESP_FAIL;
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(dev->port, dev->addr, write_buf, sizeof(write_buf), 100 / portTICK_PERIOD_MS);
}

void mpu6050_wake_up(mpu6050_handle_t handle) {
    mpu6050_write_byte(handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0);
}

void mpu6050_get_acce(mpu6050_handle_t handle, mpu6050_acce_value_t *acce) {
    uint8_t data[6];
    mpu6050_read(handle, MPU6050_ACCEL_XOUT_H, data, 6);
    acce->acce_x = (int16_t)((data[0] << 8) | data[1]);
    acce->acce_y = (int16_t)((data[2] << 8) | data[3]);
    acce->acce_z = (int16_t)((data[4] << 8) | data[5]);
}

void mpu6050_get_gyro(mpu6050_handle_t handle, mpu6050_gyro_value_t *gyro) {
    uint8_t data[6];
    mpu6050_read(handle, MPU6050_GYRO_XOUT_H, data, 6);
    gyro->gyro_x = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyro_y = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyro_z = (int16_t)((data[4] << 8) | data[5]);
}

void mpu6050_get_temp(mpu6050_handle_t handle, mpu6050_temp_value_t *temp) {
    uint8_t data[2];
    mpu6050_read(handle, MPU6050_TEMP_XOUT_H, data, 2);
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    temp->temp = 36.53f + ((float)raw / 340.0f);
}
