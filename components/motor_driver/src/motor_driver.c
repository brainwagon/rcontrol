#include <math.h>
#include "motor_driver.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_DRIVER";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_LEFT     (0) // Placeholder, overwritten by config
#define LEDC_OUTPUT_IO_RIGHT    (0) // Placeholder
#define LEDC_CHANNEL_LEFT       LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT      LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Set duty resolution to 12 bits
#define LEDC_DUTY_MAX           (4095) // 2^12 - 1

static motor_driver_config_t s_config;

esp_err_t motor_driver_init(const motor_driver_config_t *config) {
    s_config = *config;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << config->gpio_in1) | (1ULL << config->gpio_in2) |
                        (1ULL << config->gpio_in3) | (1ULL << config->gpio_in4),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = config->pwm_freq_hz,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_left = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = config->gpio_ena,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left));

    ledc_channel_config_t ledc_channel_right = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = config->gpio_enb,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right));

    ESP_LOGI(TAG, "Motor Driver Initialized");
    return ESP_OK;
}

static void set_motor_state(int in1, int in2, int channel, float speed) {
    uint32_t duty = (uint32_t)(fabs(speed) * LEDC_DUTY_MAX);
    if (duty > LEDC_DUTY_MAX) duty = LEDC_DUTY_MAX;

    if (speed > 0.05f) { // Deadzone
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (speed < -0.05f) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        duty = 0;
    }

    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void motor_driver_set_speed(float left_speed, float right_speed) {
    set_motor_state(s_config.gpio_in1, s_config.gpio_in2, LEDC_CHANNEL_LEFT, left_speed);
    set_motor_state(s_config.gpio_in3, s_config.gpio_in4, LEDC_CHANNEL_RIGHT, right_speed);
}
