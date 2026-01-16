#include "i2c_manager.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "ds3231.h"

static const char *TAG = "I2C_MAN";

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_PORT_NUM I2C_NUM_0

// Detected devices flags
static bool s_mpu6050_detected = false;
static bool s_ina219_detected = false;
static bool s_ssd1306_detected = false;
static bool s_ds3231_detected = false;

static mpu6050_handle_t s_mpu_handle = NULL;
static ssd1306_handle_t s_ssd_handle = NULL;
static ds3231_handle_t s_ds3231_handle = NULL;

void i2c_manager_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    ESP_LOGI(TAG, "I2C Initialized on SDA:%d SCL:%d", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Scan Bus
    printf("I2C Scan:\n");
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Found device at: 0x%02x\n", i);
            if (i == 0x68 || i == 0x69) {
                // Check if it is MPU6050 by reading WHO_AM_I (0x75)
                uint8_t who_am_i = 0;
                uint8_t reg = 0x75;
                esp_err_t err = i2c_master_write_read_device(I2C_PORT_NUM, i, &reg, 1, &who_am_i, 1, 100 / portTICK_PERIOD_MS);
                
                if (err == ESP_OK && who_am_i == 0x68) {
                    s_mpu6050_detected = true;
                    ESP_LOGI(TAG, "MPU6050 Detected at 0x%02x", i);
                    // Init MPU
                    s_mpu_handle = mpu6050_create(I2C_PORT_NUM, i);
                    mpu6050_wake_up(s_mpu_handle);
                } else if (i == 0x68) {
                    // Assume DS3231 if at 0x68 and not MPU6050
                    s_ds3231_detected = true;
                    ESP_LOGI(TAG, "DS3231 Detected at 0x%02x", i);
                    s_ds3231_handle = ds3231_create(I2C_PORT_NUM, i);
                }
            } else if (i == 0x40 || i == 0x41 || i == 0x44 || i == 0x45) {
                 // INA219 addresses
                 s_ina219_detected = true;
                 ESP_LOGI(TAG, "INA219 Detected at 0x%02x", i);
            } else if (i == 0x3C || i == 0x3D) {
                 s_ssd1306_detected = true;
                 ESP_LOGI(TAG, "SSD1306 Detected at 0x%02x", i);
                 s_ssd_handle = ssd1306_create(I2C_PORT_NUM, i);
                 ssd1306_init(s_ssd_handle);
                 ssd1306_draw_string(s_ssd_handle, 0, 0, "RControl Init...");
                 ssd1306_refresh(s_ssd_handle);
            }
        }
    }
}

void i2c_manager_get_json(cJSON *root) {
    cJSON *i2c_obj = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "i2c", i2c_obj);

    if (s_mpu6050_detected && s_mpu_handle) {
        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        mpu6050_temp_value_t temp;
        
        mpu6050_get_acce(s_mpu_handle, &acce);
        mpu6050_get_gyro(s_mpu_handle, &gyro);
        mpu6050_get_temp(s_mpu_handle, &temp);

        cJSON *mpu = cJSON_CreateObject();
        cJSON_AddNumberToObject(mpu, "ax", acce.acce_x);
        cJSON_AddNumberToObject(mpu, "ay", acce.acce_y);
        cJSON_AddNumberToObject(mpu, "az", acce.acce_z);
        cJSON_AddNumberToObject(mpu, "temp", temp.temp);
        cJSON_AddItemToObject(i2c_obj, "mpu6050", mpu);
    }

    if (s_ina219_detected) {
        // Placeholder for manual INA219 read
        cJSON *ina = cJSON_CreateObject();
        cJSON_AddStringToObject(ina, "status", "detected");
        cJSON_AddItemToObject(i2c_obj, "ina219", ina);
    }
    
    if (s_ds3231_detected && s_ds3231_handle) {
        ds3231_time_t time;
        ds3231_get_time(s_ds3231_handle, &time);
        float temp = ds3231_get_temp(s_ds3231_handle);
        
        cJSON *rtc = cJSON_CreateObject();
        char time_str[32];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", time.hours, time.minutes, time.seconds);
        cJSON_AddStringToObject(rtc, "time", time_str);
        cJSON_AddNumberToObject(rtc, "temp", temp);
        cJSON_AddItemToObject(i2c_obj, "ds3231", rtc);
    }

    if (s_ssd1306_detected) {
        cJSON_AddStringToObject(i2c_obj, "ssd1306", "active");
    }
}

void i2c_manager_update_display(const char *ip_addr) {
    if (s_ssd1306_detected && s_ssd_handle) {
        ssd1306_clear(s_ssd_handle);
        ssd1306_draw_string(s_ssd_handle, 0, 0, CONFIG_MDNS_HOSTNAME);
        ssd1306_draw_string(s_ssd_handle, 0, 8, ".local");
        
        char buf[32];
        snprintf(buf, sizeof(buf), "IP: %s", ip_addr);
        ssd1306_draw_string(s_ssd_handle, 0, 24, buf);
        
        ssd1306_refresh(s_ssd_handle);
    }
}

#include <time.h>
#include <sys/time.h>

void i2c_manager_sync_rtc(void) {
    if (!s_ds3231_detected || !s_ds3231_handle) {
        ESP_LOGE(TAG, "Cannot sync RTC: No DS3231 detected");
        return;
    }

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Is time set? If year is < 2020 (120 + 1900), likely not synced.
    // However, user might force sync even if system time is wrong, but typically we want SNTP time.
    // Let's assume the caller ensures SNTP is ready or we just write whatever system time we have.
    // The requirement says "try to synchronize... using NTP".
    // We assume SNTP runs in background and updates system time.
    
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGW(TAG, "System time not yet set by SNTP? Year is %d", timeinfo.tm_year + 1900);
        // We proceed anyway? Or maybe the user wants to trigger SNTP update?
        // Usually SNTP is automatic. If we are here, we trust system time.
    }

    ds3231_time_t rtc_time;
    rtc_time.seconds = timeinfo.tm_sec;
    rtc_time.minutes = timeinfo.tm_min;
    rtc_time.hours = timeinfo.tm_hour;
    rtc_time.day_of_week = timeinfo.tm_wday + 1; // tm_wday is 0-6 (Sun-Sat), DS3231 is 1-7
    rtc_time.day = timeinfo.tm_mday;
    rtc_time.month = timeinfo.tm_mon + 1; // tm_mon is 0-11
    rtc_time.year = timeinfo.tm_year % 100; // 2 digit year

    ds3231_set_time(s_ds3231_handle, &rtc_time);
    ESP_LOGI(TAG, "RTC Synchronized with System Time: %s", asctime(&timeinfo));
}
