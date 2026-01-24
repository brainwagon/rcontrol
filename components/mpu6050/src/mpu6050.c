#include "mpu6050.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MPU6050";

// Registers
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

static i2c_port_t g_i2c_num;
static uint8_t g_i2c_addr;

esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (g_i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_wake(void) {
    // Clear sleep bit in PWR_MGMT_1 (bit 6)
    // Writing 0x00 wakes it up and uses internal 8MHz oscillator
    ESP_LOGI(TAG, "Waking up MPU6050...");
    return mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00);
}

esp_err_t mpu6050_init(i2c_port_t i2c_num, uint8_t i2c_addr) {
    g_i2c_num = i2c_num;
    g_i2c_addr = i2c_addr;

    esp_err_t ret = mpu6050_wake();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050");
        return ret;
    }
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data) {
    uint8_t raw_data[14];
    // Read 14 bytes starting from ACCEL_XOUT_H
    // ACCEL_XOUT_H (0x3B) -> ACCEL_XOUT_L (0x3C) -> ... -> GYRO_ZOUT_L (0x48)
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) return ret;

    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    data->temp    = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->gyro_x  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y  = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z  = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return ESP_OK;
}
