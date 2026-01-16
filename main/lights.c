#include "lights.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HEARTBEAT_LEDC_TIMER       LEDC_TIMER_1
#define HEARTBEAT_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define HEARTBEAT_LEDC_OUTPUT_IO   LED_HEARTBEAT_PIN
#define HEARTBEAT_LEDC_CHANNEL     LEDC_CHANNEL_2
#define HEARTBEAT_LEDC_DUTY_RES    LEDC_TIMER_13_BIT
#define HEARTBEAT_LEDC_FREQUENCY   5000 // Frequency in Hertz. Set frequency at 5 kHz

static void heartbeat_task(void *arg) {
    while (1) {
        // Fade up to 4000 (approx 50% duty cycle for 13-bit) over 1000ms
        ledc_set_fade_with_time(HEARTBEAT_LEDC_MODE, HEARTBEAT_LEDC_CHANNEL, 4000, 1000);
        ledc_fade_start(HEARTBEAT_LEDC_MODE, HEARTBEAT_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Fade down to 0 over 1000ms
        ledc_set_fade_with_time(HEARTBEAT_LEDC_MODE, HEARTBEAT_LEDC_CHANNEL, 0, 1000);
        ledc_fade_start(HEARTBEAT_LEDC_MODE, HEARTBEAT_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void lights_init(void) {
    // 1. Configure Turn Signals (GPIO)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_LEFT_TURN) | (1ULL << LED_RIGHT_TURN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Start off
    gpio_set_level(LED_LEFT_TURN, 0);
    gpio_set_level(LED_RIGHT_TURN, 0);

    // 2. Configure Heartbeat LED (LEDC PWM)
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = HEARTBEAT_LEDC_DUTY_RES, // resolution of PWM duty
        .freq_hz = HEARTBEAT_LEDC_FREQUENCY,                      // frequency of PWM signal
        .speed_mode = HEARTBEAT_LEDC_MODE,           // timer mode
        .timer_num = HEARTBEAT_LEDC_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = HEARTBEAT_LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = HEARTBEAT_LEDC_OUTPUT_IO,
        .speed_mode = HEARTBEAT_LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = HEARTBEAT_LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    ledc_fade_func_install(0);

    // Start heartbeat task
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 5, NULL);
}


void lights_set_left(bool on) {
    gpio_set_level(LED_LEFT_TURN, on ? 1 : 0);
}

void lights_set_right(bool on) {
    gpio_set_level(LED_RIGHT_TURN, on ? 1 : 0);
}

void lights_toggle_left(void) {
    gpio_set_level(LED_LEFT_TURN, !gpio_get_level(LED_LEFT_TURN));
}

void lights_toggle_right(void) {
    gpio_set_level(LED_RIGHT_TURN, !gpio_get_level(LED_RIGHT_TURN));
}
