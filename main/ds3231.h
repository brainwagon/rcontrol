#ifndef DS3231_H
#define DS3231_H

#include "driver/i2c.h"

typedef void *ds3231_handle_t;

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} ds3231_time_t;

ds3231_handle_t ds3231_create(i2c_port_t port, uint16_t dev_addr);
void ds3231_get_time(ds3231_handle_t handle, ds3231_time_t *time);
void ds3231_set_time(ds3231_handle_t handle, const ds3231_time_t *time);
float ds3231_get_temp(ds3231_handle_t handle);

#endif