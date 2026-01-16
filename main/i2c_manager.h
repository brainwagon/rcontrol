#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "cJSON.h"

void i2c_manager_init(void);
void i2c_manager_get_json(cJSON *root);
void i2c_manager_update_display(const char *ip_addr);

#endif
