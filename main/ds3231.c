#include "ds3231.h"
#include "esp_log.h"
#include <stdlib.h>

#define DS3231_ADDR 0x68
#define DS3231_REG_TIME 0x00
#define DS3231_REG_TEMP 0x11

typedef struct {
    i2c_port_t port;
    uint16_t addr;
} ds3231_dev_t;

ds3231_handle_t ds3231_create(i2c_port_t port, uint16_t dev_addr) {
    ds3231_dev_t *dev = malloc(sizeof(ds3231_dev_t));
    dev->port = port;
    dev->addr = dev_addr;
    return (ds3231_handle_t)dev;
}

static uint8_t bcd2dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

static uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) + (val % 10);
}

void ds3231_get_time(ds3231_handle_t handle, ds3231_time_t *time) {
    ds3231_dev_t *dev = (ds3231_dev_t *)handle;
    uint8_t data[7];
    uint8_t reg = DS3231_REG_TIME;
    
    // Read 7 bytes starting from register 0x00
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, &reg, 1, data, 7, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return;
    }

    time->seconds = bcd2dec(data[0]);
    time->minutes = bcd2dec(data[1]);
    // Handle 12/24 hour mode? For simplicity assume 24h or just mask raw
    time->hours = bcd2dec(data[2] & 0x3F); // Mask bits to handle 12/24h flag if present, usually bit 6
    time->day_of_week = bcd2dec(data[3]);
    time->day = bcd2dec(data[4]);
    time->month = bcd2dec(data[5] & 0x1F); // Century bit in bit 7
    time->year = bcd2dec(data[6]);
}

void ds3231_set_time(ds3231_handle_t handle, const ds3231_time_t *time) {
    ds3231_dev_t *dev = (ds3231_dev_t *)handle;
    uint8_t data[8]; // 1 byte reg addr + 7 bytes data
    
    data[0] = DS3231_REG_TIME;
    data[1] = dec2bcd(time->seconds);
    data[2] = dec2bcd(time->minutes);
    data[3] = dec2bcd(time->hours); // Assumes 24h mode, bit 6=0
    data[4] = dec2bcd(time->day_of_week);
    data[5] = dec2bcd(time->day);
    data[6] = dec2bcd(time->month); // Century bit 0
    data[7] = dec2bcd(time->year);
    
    i2c_master_write_to_device(dev->port, dev->addr, data, 8, 100 / portTICK_PERIOD_MS);
}

float ds3231_get_temp(ds3231_handle_t handle) {
    ds3231_dev_t *dev = (ds3231_dev_t *)handle;
    uint8_t data[2];
    uint8_t reg = DS3231_REG_TEMP;
    
    esp_err_t ret = i2c_master_write_read_device(dev->port, dev->addr, &reg, 1, data, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return 0.0f;
    }

    int16_t temp_raw = (data[0] << 8) | data[1];
    return (float)temp_raw / 256.0f; // Registers are integer part + fractional part (upper 2 bits of LSB)
    // Actually DS3231 format: Upper byte is integer, Lower byte top 2 bits are .25 increments
    // So (MSB) + (LSB >> 6) * 0.25
    // Which is equivalent to ((MSB << 8) | LSB) / 256.0 if we treat it as 8.8 fixed point but only top 10 bits valid?
    // Let's stick to the datasheet:
    // Temp = integer part (addr 11h) + (fractional part (addr 12h) >> 6) * 0.25
    
    return (float)(int8_t)data[0] + ((data[1] >> 6) * 0.25f);
}
