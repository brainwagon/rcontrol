#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160

// RGB565 Colors
#define ST7735_BLACK   0x0000
#define ST7735_WHITE   0xFFFF
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_BLUE    0x001F
#define ST7735_YELLOW  0xFFE0
#define ST7735_MAGENTA 0xF81F
#define ST7735_CYAN    0x07FF

typedef struct st7735_dev_t *st7735_handle_t;

typedef struct {
    int sclk_pin;
    int mosi_pin;
    int dc_pin;
    int rst_pin;
    int cs_pin;
    spi_host_device_t host;
} st7735_config_t;

/**
 * @brief Initialize the ST7735 display.
 * 
 * @param cfg Configuration structure
 * @param out_handle Returned handle for the display instance
 * @return esp_err_t 
 */
esp_err_t st7735_init(const st7735_config_t *cfg, st7735_handle_t *out_handle);

/**
 * @brief Clear the display with a background color.
 */
void st7735_clear(st7735_handle_t dev, uint16_t color);

/**
 * @brief Draw a single pixel.
 */
void st7735_draw_pixel(st7735_handle_t dev, uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw a string at coordinates (x, y).
 * Uses a basic 8x8 built-in font.
 */
void st7735_draw_string(st7735_handle_t dev, uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg_color);

/**
 * @brief Fill a rectangle.
 */
void st7735_fill_rect(st7735_handle_t dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

#ifdef __cplusplus
}
#endif
