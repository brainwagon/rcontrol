#pragma once

#include <stdint.h>
#include "esp_err.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

/**
 * @brief Initialize the SSD1306 display on the specified I2C pins.
 * 
 * @param sda_pin GPIO number for SDA
 * @param scl_pin GPIO number for SCL
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ssd1306_init(int sda_pin, int scl_pin);

/**
 * @brief Clear the internal frame buffer.
 */
void ssd1306_clear(void);

/**
 * @brief Draw a string at the specified coordinates.
 * 
 * @param x X coordinate (0-127) - automatically aligned to 8-pixel boundaries for text if needed, but pixel precision supported by buffer.
 * @param y Y coordinate (0-63)
 * @param text Null-terminated string
 */
void ssd1306_draw_string(int x, int y, const char *text);

/**
 * @brief Update the display with the content of the frame buffer.
 */
void ssd1306_update(void);
