#ifndef SSD1306_H
#define SSD1306_H

#include "driver/i2c.h"

typedef void* ssd1306_handle_t;

ssd1306_handle_t ssd1306_create(i2c_port_t port, uint8_t addr);
void ssd1306_init(ssd1306_handle_t handle);
void ssd1306_clear(ssd1306_handle_t handle);
void ssd1306_draw_string(ssd1306_handle_t handle, int x, int y, const char *text);
void ssd1306_refresh(ssd1306_handle_t handle);

#endif
