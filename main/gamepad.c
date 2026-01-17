#include "gamepad.h"
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GAMEPAD";

static gamepad_state_t current_state = {0};
static bool is_scanning = false;
static bool is_connecting = false;

// Forward declaration
void start_scan(void);

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Device Found
        esp_bd_addr_t *bda = &param->disc_res.bda;
        uint32_t cod = 0;
        
        char name[32] = {0};
        bool name_found = false;
        
        // Find properties
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD) {
                cod = *(uint32_t *)(param->disc_res.prop[i].val);
            } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                size_t len = param->disc_res.prop[i].len;
                if (len > 31) len = 31;
                memcpy(name, param->disc_res.prop[i].val, len);
                name[len] = '\0';
                name_found = true;
            }
        }

        if (name_found) {
            // Filter: Connect if name matches common controller patterns
            bool name_match = (strstr(name, "GameSir") != NULL) || 
                              (strstr(name, "Xbox") != NULL) || 
                              (strstr(name, "Pro Controller") != NULL) || 
                              (strstr(name, "Wireless Controller") != NULL);

            if (name_match) { 
                 if (is_connecting || current_state.connected) break;

                 ESP_LOGI(TAG, "Found Controller (%s)! Connecting... (Heap: %lu)", name, esp_get_free_heap_size());
                 ESP_LOGI(TAG, "Target BDA: %02x:%02x:%02x:%02x:%02x:%02x", 
                          (*bda)[0], (*bda)[1], (*bda)[2], (*bda)[3], (*bda)[4], (*bda)[5]);
                 
                 // Stop scanning and visibility to focus on connection
                 if (is_scanning) {
                     esp_bt_gap_cancel_discovery();
                 }
                 esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                 
                 is_connecting = true;

                 void *dev = esp_hidh_dev_open(*bda, ESP_HID_TRANSPORT_BT, 0);
                 if (dev == NULL) {
                     ESP_LOGE(TAG, "Failed to initiate connection to %s (Classic BT)", name);
                     is_connecting = false;
                 } else {
                     ESP_LOGI(TAG, "Initiated connection via Classic BT");
                 }
            }
        } else {
            // Name not in inquiry response, request it explicitly
            esp_bt_gap_read_remote_name(*bda);
        }
        break;
    }
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
             char *name = (char *)param->read_rmt_name.rmt_name;
             
             bool name_match = (strstr(name, "GameSir") != NULL) || 
                               (strstr(name, "Xbox") != NULL) || 
                               (strstr(name, "Pro Controller") != NULL) || 
                               (strstr(name, "Wireless Controller") != NULL);
            
             if (name_match) {
                 if (is_connecting || current_state.connected) break;

                 ESP_LOGI(TAG, "Found Controller (%s)! Connecting... (Heap: %lu)", name, esp_get_free_heap_size());
                 ESP_LOGI(TAG, "Target BDA: %02x:%02x:%02x:%02x:%02x:%02x", 
                          param->read_rmt_name.bda[0], param->read_rmt_name.bda[1], 
                          param->read_rmt_name.bda[2], param->read_rmt_name.bda[3], 
                          param->read_rmt_name.bda[4], param->read_rmt_name.bda[5]);
                 
                 // Stop scanning and visibility to focus on connection
                 if (is_scanning) {
                     esp_bt_gap_cancel_discovery();
                 }
                 esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

                 is_connecting = true;
                 
                 // Cancel discovery not always strictly needed here but good practice before connecting
                 void *dev = esp_hidh_dev_open(param->read_rmt_name.bda, ESP_HID_TRANSPORT_BT, 0);
                 if (dev == NULL) {
                     ESP_LOGE(TAG, "Failed to initiate connection to %s (Classic BT)", name);
                     is_connecting = false;
                 } else {
                     ESP_LOGI(TAG, "Initiated connection via Classic BT");
                 }
             }
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
        is_connecting = false; // Finished connecting attempt
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
        is_connecting = false;
        // Restart scan
        start_scan();
        break;
    default:
        ESP_LOGI(TAG, "HIDH Event: %d", event);
        break;
    }
}

void gamepad_init(void) {
    ESP_LOGI(TAG, "Initializing Gamepad Module...");
    esp_err_t ret;

    // NVS init is assumed done in main.c

    // Initialize Bluetooth Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT Controller Init Failed");
        return;
    }
    ESP_LOGI(TAG, "BT Controller Initialized");

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT Controller Enable Failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }
    ESP_LOGI(TAG, "BT Controller Enabled (Classic)");

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid Init Failed");
        return;
    }
    ESP_LOGI(TAG, "Bluedroid Initialized");

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid Enable Failed");
        return;
    }
    ESP_LOGI(TAG, "Bluedroid Enabled");

    // Set default parameters for Secure Simple Pairing (Classic BT)
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    ESP_LOGI(TAG, "Security Parameters Set (Classic)");

    // Register GAP Callback for scanning
    ret = esp_bt_gap_register_callback(gap_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP Callback Register Failed");
        return;
    }
    ESP_LOGI(TAG, "GAP Callback Registered");

    // Initialize HID Host
    esp_hidh_config_t hidh_config = {
        .callback = gamepad_hidh_callback,
        .event_stack_size = 6144,
        .callback_arg = NULL,
    };
    ESP_LOGI(TAG, "Calling esp_hidh_init... (Waiting 2s)");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ret = esp_hidh_init(&hidh_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID Host Init Failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }
    ESP_LOGI(TAG, "HID Host Initialized");

    // Set Scan Mode to allow others to find us (optional, mostly we are scanning them)
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Start Scanning
    ESP_LOGI(TAG, "Starting BT Scan...");
    start_scan();
}

gamepad_state_t gamepad_get_state(void) {
    return current_state;
}