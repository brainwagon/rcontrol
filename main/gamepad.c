#include "gamepad.h"
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GAMEPAD";

static gamepad_state_t current_state = {0};
static bool is_scanning = false;

// Forward declaration
void start_scan(void);

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Device Found
        esp_bd_addr_t *bda = &param->disc_res.bda;
        uint32_t cod = 0;
        
        // Find COD in properties
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD) {
                cod = *(uint32_t *)(param->disc_res.prop[i].val);
                break;
            }
        }

        ESP_LOGI(TAG, "Device found: %02x:%02x:%02x:%02x:%02x:%02x (COD: 0x%06lx)",
                 (*bda)[0], (*bda)[1], (*bda)[2], (*bda)[3], (*bda)[4], (*bda)[5], cod);

        // Major Device Class (bits 8-12). 0x05 = Peripheral
        uint32_t major_class = (cod >> 8) & 0x1F;

        if (major_class == 0x05) { 
             ESP_LOGI(TAG, "Found Peripheral! Connecting...");
             esp_bt_gap_cancel_discovery();
             esp_hidh_dev_open(*bda, ESP_HID_TRANSPORT_BT, 0);
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            is_scanning = false;
            if (!current_state.connected) {
                // If we stopped scanning and are not connected, restart scan (maybe after delay)
                // For simplicity, we restart immediately here if purely seeking.
                // In production, avoid spamming.
                ESP_LOGI(TAG, "Scan stopped. Restarting...");
                start_scan();
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            is_scanning = true;
            ESP_LOGI(TAG, "Scan started.");
        }
        break;
    default:
        break;
    }
}

void start_scan(void) {
    if (!is_scanning && !current_state.connected) {
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    }
}

void gamepad_hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT:
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, "Connected to %02x:%02x:%02x:%02x:%02x:%02x",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            current_state.connected = true;
        } else {
            ESP_LOGE(TAG, "Open failed, status: %d", param->open.status);
            start_scan();
        }
        break;
    case ESP_HIDH_CLOSE_EVENT:
        ESP_LOGI(TAG, "Disconnected");
        current_state.connected = false;
        // Restart scan
        start_scan();
        break;
    case ESP_HIDH_INPUT_EVENT:
        if (param->input.length >= 2) {
            uint8_t *data = param->input.data;
            
            // Map 0-255 to -127 to 127
            // Y is often inverted (0 is top)
            // Note: Mapping depends heavily on specific controller.
            // This is a common mapping for generic HID gamepads.
            current_state.joy_x = (int)data[0] - 128;
            current_state.joy_y = (int)data[1] - 128; 

            // Simple deadzone
            if (current_state.joy_x > -10 && current_state.joy_x < 10) current_state.joy_x = 0;
            if (current_state.joy_y > -10 && current_state.joy_y < 10) current_state.joy_y = 0;

            if (param->input.length > 5) {
                current_state.buttons = data[5]; 
            }
        }
        break;
    default:
        break;
    }
}

void gamepad_init(void) {
    esp_err_t ret;

    // NVS init is assumed done in main.c

    // Initialize Bluetooth Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT Controller Init Failed");
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT Controller Enable Failed");
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid Init Failed");
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid Enable Failed");
        return;
    }

    // Register GAP Callback for scanning
    esp_bt_gap_register_callback(gap_callback);

    // Initialize HID Host
    esp_hidh_config_t hidh_config = {
        .callback = gamepad_hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    esp_hidh_init(&hidh_config);

    // Set Scan Mode to allow others to find us (optional, mostly we are scanning them)
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Start Scanning
    ESP_LOGI(TAG, "Starting BT Scan...");
    start_scan();
}

gamepad_state_t gamepad_get_state(void) {
    return current_state;
}