#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "gamepad.h"
#include "motor_driver.h"
#if CONFIG_ENABLE_SSD1306
#include "ssd1306.h"
#endif
#if CONFIG_ENABLE_MPU6050
#include "mpu6050.h"
#endif
#include "st7735.h"


static const char *TAG = "ROBOT_MAIN";
static volatile float g_speed_left = 0.0f;
static volatile float g_speed_right = 0.0f;
static volatile gamepad_status_t g_gamepad_status = GAMEPAD_STATUS_SCANNING;
static volatile bool g_just_connected = false;

#if CONFIG_ENABLE_ST7735
static st7735_handle_t g_st7735_devs[CONFIG_ST7735_NUM_DEVS];
#endif

#define I2C_SDA 21

#define I2C_SCL 22



#define I2C_MASTER_NUM              0

#define I2C_MASTER_FREQ_HZ          400000

#define I2C_MASTER_TX_BUF_DISABLE   0

#define I2C_MASTER_RX_BUF_DISABLE   0



// Pin Configuration
// Avoid strapping pins (0, 2, 5, 12, 15) if possible or ensure correct pull during boot.
// We use:
// Left Motor: ENA=32, IN1=33, IN2=25
// Right Motor: ENB=26, IN3=27, IN4=14 (14 is safe for output)
#define MOTOR_L_ENA 32
#define MOTOR_L_IN1 33
#define MOTOR_L_IN2 25

#define MOTOR_R_ENB 26
#define MOTOR_R_IN3 27
#define MOTOR_R_IN4 14

#define PWM_FREQ_HZ 10000 // 10kHz

void connection_callback(gamepad_status_t status) {
    g_gamepad_status = status;
    if (status == GAMEPAD_STATUS_CONNECTED) {
        g_just_connected = true;
    }
}

void input_callback(const gamepad_state_t *state) {
    // Tank Drive Mixing
    // Just update global state. The control task will apply it.
    g_speed_left = state->left_stick_y;
    g_speed_right = state->right_stick_y;
}

void motor_control_task(void *arg) {
    float last_log_l = 0.0f;
    float last_log_r = 0.0f;
    bool was_moving = false;
    int loop_count = 0;
    int paired_ticks = 0;

    while (1) {
        float cur_l = g_speed_left;
        float cur_r = g_speed_right;
        bool is_moving = (fabs(cur_l) > 0.05f || fabs(cur_r) > 0.05f);

        // Apply speed safely in this task (approx 50Hz)
        motor_driver_set_speed(cur_l, cur_r);

        // Handle connection transition
        if (g_just_connected) {
            paired_ticks = 30; // Show PAIRED for ~3 seconds (30 * 100ms)
            g_just_connected = false;
        }

        // Logging Logic
        if (++loop_count >= 5) { // Every 100ms
            loop_count = 0;

            // Update Display
#if CONFIG_ENABLE_SSD1306
            ssd1306_clear();
#endif
#if CONFIG_ENABLE_ST7735
            for (int i = 0; i < CONFIG_ST7735_NUM_DEVS; i++) {
                st7735_clear(g_st7735_devs[i], ST7735_BLACK);
            }
#endif

            if (g_gamepad_status == GAMEPAD_STATUS_CONNECTED) {
                if (paired_ticks > 0) {
                    paired_ticks--;
#if CONFIG_ENABLE_SSD1306
                    // Center "PAIRED" roughly
                    // Font width ~8px? "PAIRED" is 6 chars -> 48px. Screen 128. (128-48)/2 = 40.
                    ssd1306_draw_string(40, 24, "PAIRED");
#endif
#if CONFIG_ENABLE_ST7735
                    for (int i = 0; i < CONFIG_ST7735_NUM_DEVS; i++) {
                        st7735_draw_string(g_st7735_devs[i], 40, 70, "PAIRED", ST7735_GREEN, ST7735_BLACK);
                    }
#endif
                } else {
                    // Normal Telemetry
                    char buf[20];

#if CONFIG_ENABLE_MPU6050
                    mpu6050_data_t imu;
                    if (mpu6050_read(&imu) == ESP_OK) {
                        // Log IMU data periodically (every 500ms approx since loop_count is reset every 100ms)
                        // Actually loop_count resets every 5 ticks (100ms).
                        // Let's log it here.
                        ESP_LOGI(TAG, "IMU: Ax:%d Ay:%d Az:%d | Gx:%d Gy:%d Gz:%d | T:%.1f",
                                 imu.accel_x, imu.accel_y, imu.accel_z,
                                 imu.gyro_x, imu.gyro_y, imu.gyro_z,
                                 (imu.temp / 340.0f) + 36.53f);
                    }
#endif

#if CONFIG_ENABLE_SSD1306
                    ssd1306_draw_string(0, 0, "Robot Status");
                    
                    snprintf(buf, sizeof(buf), "L: %+.2f", cur_l);
                    ssd1306_draw_string(0, 16, buf);
                    
                    snprintf(buf, sizeof(buf), "R: %+.2f", cur_r);
                    ssd1306_draw_string(0, 24, buf);

                    if (is_moving) {
                        ssd1306_draw_string(0, 40, "MOVING");
                    } else {
                        ssd1306_draw_string(0, 40, "STOPPED");
                    }
#endif

#if CONFIG_ENABLE_ST7735
                    for (int i = 0; i < CONFIG_ST7735_NUM_DEVS; i++) {
                        st7735_draw_string(g_st7735_devs[i], 10, 10, "Robot Status", ST7735_YELLOW, ST7735_BLACK);
                        snprintf(buf, sizeof(buf), "L: %+.2f", cur_l);
                        st7735_draw_string(g_st7735_devs[i], 10, 30, buf, ST7735_WHITE, ST7735_BLACK);
                        snprintf(buf, sizeof(buf), "R: %+.2f", cur_r);
                        st7735_draw_string(g_st7735_devs[i], 10, 40, buf, ST7735_WHITE, ST7735_BLACK);
                        if (is_moving) st7735_draw_string(g_st7735_devs[i], 10, 60, "MOVING", ST7735_RED, ST7735_BLACK);
                    }
#endif
                }
            } else if (g_gamepad_status == GAMEPAD_STATUS_CONNECTING) {
#if CONFIG_ENABLE_SSD1306
                // Center "PAIRING" (7 chars -> 56px) -> 36
                ssd1306_draw_string(36, 24, "PAIRING");
#endif
#if CONFIG_ENABLE_ST7735
                for (int i = 0; i < CONFIG_ST7735_NUM_DEVS; i++) {
                    st7735_draw_string(g_st7735_devs[i], 36, 70, "PAIRING", ST7735_YELLOW, ST7735_BLACK);
                }
#endif
            } else {
#if CONFIG_ENABLE_SSD1306
                // SCANNING or DISCONNECTED or BOOT -> Show "BOOT" per request
                // Center "BOOT" (4 chars -> 32px) -> 48
                ssd1306_draw_string(48, 24, "BOOT");
#endif
#if CONFIG_ENABLE_ST7735
                for (int i = 0; i < CONFIG_ST7735_NUM_DEVS; i++) {
                    st7735_draw_string(g_st7735_devs[i], 48, 70, "BOOT", ST7735_WHITE, ST7735_BLACK);
                }
#endif
            }
            
#if CONFIG_ENABLE_SSD1306
            ssd1306_update();
#endif



            if (is_moving) {

                if (cur_l != last_log_l || cur_r != last_log_r) {
                    ESP_LOGI(TAG, "Motor Speed - L: %.2f | R: %.2f", cur_l, cur_r);
                    last_log_l = cur_l;
                    last_log_r = cur_r;
                }
                was_moving = true;
            } else if (was_moving) {
                // Just stopped
                ESP_LOGI(TAG, "Motor Stopped");
                last_log_l = cur_l;
                last_log_r = cur_r;
                was_moving = false;
            } else {
                // Stayed stopped, update last values to avoid noise
                last_log_l = cur_l;
                last_log_r = cur_r;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Robot Controller...");

    // Initialize Motors
    motor_driver_config_t motor_cfg = {
        .gpio_ena = MOTOR_L_ENA,
        .gpio_in1 = MOTOR_L_IN1,
        .gpio_in2 = MOTOR_L_IN2,
        .gpio_enb = MOTOR_R_ENB,
        .gpio_in3 = MOTOR_R_IN3,
        .gpio_in4 = MOTOR_R_IN4,
        .pwm_freq_hz = PWM_FREQ_HZ
    };
    ESP_ERROR_CHECK(motor_driver_init(&motor_cfg));

    // Initialize I2C Bus
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

#if CONFIG_ENABLE_MPU6050
    // Initialize MPU6050
    ESP_LOGI(TAG, "Initializing MPU6050...");
    if (mpu6050_init(I2C_MASTER_NUM, CONFIG_MPU6050_I2C_ADDR) == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 Init Success");
    } else {
        ESP_LOGE(TAG, "MPU6050 Init Failed");
    }
#endif

#if CONFIG_ENABLE_SSD1306
    // Initialize Display
    if (ssd1306_init() == ESP_OK) {
        ssd1306_draw_string(48, 24, "BOOT");
        ssd1306_update();
    } else {
        ESP_LOGE(TAG, "SSD1306 Init Failed");
    }
#endif


#if CONFIG_ENABLE_ST7735
    ESP_LOGI(TAG, "Initializing ST7735 SPI Displays...");
    spi_bus_config_t buscfg = {
        .miso_io_num = -1, // No MISO for now
        .mosi_io_num = CONFIG_ST7735_PIN_MOSI,
        .sclk_io_num = CONFIG_ST7735_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7735_WIDTH * ST7735_HEIGHT * 2,
    };
    esp_err_t ret_spi = spi_bus_initialize((spi_host_device_t)(CONFIG_ST7735_SPI_HOST - 1), &buscfg, SPI_DMA_CH_AUTO);
    if (ret_spi == ESP_OK) {
        st7735_config_t cfg1 = {
            .sclk_pin = CONFIG_ST7735_PIN_SCLK,
            .mosi_pin = CONFIG_ST7735_PIN_MOSI,
            .dc_pin = CONFIG_ST7735_PIN_DC,
            .rst_pin = CONFIG_ST7735_PIN_RST,
            .cs_pin = CONFIG_ST7735_PIN_CS1,
            .host = (spi_host_device_t)(CONFIG_ST7735_SPI_HOST - 1),
        };
        st7735_init(&cfg1, &g_st7735_devs[0]);
        st7735_clear(g_st7735_devs[0], ST7735_BLACK);
        st7735_draw_string(g_st7735_devs[0], 48, 70, "BOOT", ST7735_WHITE, ST7735_BLACK);

#if CONFIG_ST7735_COUNT_2
        st7735_config_t cfg2 = cfg1;
        cfg2.cs_pin = CONFIG_ST7735_PIN_CS2;
        st7735_init(&cfg2, &g_st7735_devs[1]);
        st7735_clear(g_st7735_devs[1], ST7735_BLACK);
        st7735_draw_string(g_st7735_devs[1], 48, 70, "BOOT", ST7735_WHITE, ST7735_BLACK);
#endif
    } else {
        ESP_LOGE(TAG, "SPI Bus Init Failed: %d", ret_spi);
    }
#endif

    // Initialize Gamepad

    gamepad_set_input_callback(input_callback);
    gamepad_set_connection_callback(connection_callback);
    esp_err_t ret = gamepad_init();
    if (ret != ESP_OK) {

        ESP_LOGE(TAG, "Gamepad Init Failed: %s", esp_err_to_name(ret));
        return;
    }
    
    xTaskCreate(motor_control_task, "motor_ctrl", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System Ready. Waiting for controller...");
}
