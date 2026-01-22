#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"

static const char *TAG = "8BITDO_DEMO";

// Extern the internal handler from esp_hid component
extern void esp_hidh_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static bool is_connected = false;

typedef struct {
    esp_bd_addr_t bda;
    esp_hid_transport_t transport;
    esp_ble_addr_type_t ble_addr_type;
} connect_req_t;

static QueueHandle_t s_connect_queue = NULL;

// Scan parameters
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

void rumble_test_task(void *pvParameters)
{
    esp_hidh_dev_t *dev = (esp_hidh_dev_t *)pvParameters;
    
    ESP_LOGI(TAG, "Starting Rumble Test...");

    // Packet: [Enable, TrgL, TrgR, Heavy, Light, Dur, 0, 0]
    
    // 1. Strong Rumble (Heavy Motor)
    ESP_LOGI(TAG, "Rumble: STRONG");
    uint8_t rumble_strong[] = {0x08, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00};
    esp_hidh_dev_output_set(dev, 0, 1, rumble_strong, sizeof(rumble_strong));
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. Weak Rumble (Light Motor)
    ESP_LOGI(TAG, "Rumble: WEAK");
    uint8_t rumble_weak[] = {0x08, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
    esp_hidh_dev_output_set(dev, 0, 1, rumble_weak, sizeof(rumble_weak));
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3. Off
    ESP_LOGI(TAG, "Rumble: OFF");
    uint8_t rumble_off[] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    esp_hidh_dev_output_set(dev, 0, 1, rumble_off, sizeof(rumble_off));

    vTaskDelete(NULL);
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, "ESP_HIDH_OPEN_EVENT: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
            ESP_LOGI(TAG, "Connected to %s", esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
            is_connected = true;
            
            // Start Rumble Test
            xTaskCreate(rumble_test_task, "rumble_test", 2048, param->open.dev, 5, NULL);
            
        } else {
            ESP_LOGE(TAG, "ESP_HIDH_OPEN_EVENT failed: %d", param->open.status);
            is_connected = false;
            // Restart scanning if connection failed
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            esp_ble_gap_start_scanning(0);
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT:
        ESP_LOGI(TAG, "ESP_HIDH_BATTERY_EVENT: %d%%", param->battery.level);
        break;
    case ESP_HIDH_INPUT_EVENT: {
        static uint8_t last_data[64] = {0};
        static size_t last_len = 0;
        
        // Only print if data has changed to prevent serial flooding and WDT triggers
        if (param->input.length != last_len || memcmp(param->input.data, last_data, param->input.length) != 0) {
            printf("Input Report (ID %d, Len %d): ", param->input.report_id, param->input.length);
            for (int i = 0; i < param->input.length; i++) {
                printf("%02X ", param->input.data[i]);
            }
            printf("\n");
            
            // Update last data
            if (param->input.length <= sizeof(last_data)) {
                memcpy(last_data, param->input.data, param->input.length);
                last_len = param->input.length;
            }
        }
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, "ESP_HIDH_CLOSE_EVENT: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
        is_connected = false;
        // Restart scanning
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        esp_ble_gap_start_scanning(0);
        break;
    }
    default:
        ESP_LOGI(TAG, "ESP_HIDH event: %d", event);
        break;
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        if (is_connected) break;
        char device_name[64] = {0};
        bool name_found = false;
        int rssi = -127;

        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
                 uint8_t *eir = (uint8_t*)param->disc_res.prop[i].val;
                 int offset = 0;
                 while (offset < param->disc_res.prop[i].len) {
                     uint8_t len = eir[offset];
                     if (len == 0) break;
                     uint8_t type = eir[offset + 1];
                     if (type == 0x09 || type == 0x08) {
                         int name_len = len - 1;
                         if (name_len > sizeof(device_name) - 1) name_len = sizeof(device_name) - 1;
                         memcpy(device_name, &eir[offset + 2], name_len);
                         device_name[name_len] = 0;
                         name_found = true;
                     }
                     offset += len + 1;
                 }
            } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                int len = param->disc_res.prop[i].len;
                if (len > sizeof(device_name) - 1) len = sizeof(device_name) - 1;
                memcpy(device_name, param->disc_res.prop[i].val, len);
                device_name[len] = 0;
                name_found = true;
            } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_RSSI) {
                rssi = *((int8_t*)param->disc_res.prop[i].val);
            }
        }

        if (name_found) {
            ESP_LOGI(TAG, "Classic Found: %s [" ESP_BD_ADDR_STR "] RSSI:%d", device_name, ESP_BD_ADDR_HEX(param->disc_res.bda), rssi);
            if (strstr(device_name, "8BitDo") || strstr(device_name, "Wireless Controller") || strstr(device_name, "Pro Controller")) {
                ESP_LOGI(TAG, "Classic Target found! Queuing connection...");
                esp_bt_gap_cancel_discovery();
                esp_ble_gap_stop_scanning();
                
                connect_req_t req;
                memcpy(req.bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
                req.transport = ESP_HID_TRANSPORT_BT;
                req.ble_addr_type = BLE_ADDR_TYPE_PUBLIC; // Not used for BT
                xQueueSend(s_connect_queue, &req, 0);
            }
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            if (!is_connected) {
                ESP_LOGI(TAG, "Classic Discovery stopped, restarting...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
        }
        break;
    default:
        break;
    }
}

static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (is_connected) break;
            
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (!adv_name) {
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
            }

            if (adv_name) {
                char name[64] = {0};
                memcpy(name, adv_name, adv_name_len);
                ESP_LOGI(TAG, "BLE Found: %s [" ESP_BD_ADDR_STR "] RSSI:%d", name, ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), scan_result->scan_rst.rssi);
                
                if (strstr(name, "8BitDo") || strstr(name, "Wireless Controller") || strstr(name, "Pro Controller")) {
                    ESP_LOGI(TAG, "BLE Target found! Queuing connection...");
                    esp_bt_gap_cancel_discovery();
                    esp_ble_gap_stop_scanning();
                    
                    connect_req_t req;
                    memcpy(req.bda, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    req.transport = ESP_HID_TRANSPORT_BLE;
                    req.ble_addr_type = scan_result->scan_rst.ble_addr_type;
                    xQueueSend(s_connect_queue, &req, 0);
                }
            }
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "BLE Scan start failed: %d", param->scan_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "BLE Scan started.");
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "BLE Security Request");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "BLE Authentication Success");
        } else {
            ESP_LOGE(TAG, "BLE Authentication Failed, reason 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Create Connection Queue
    s_connect_queue = xQueueCreate(1, sizeof(connect_req_t));

    // ... NVS Init ...
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BT Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Enabling Controller in Dual Mode...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Initializing Bluedroid...");
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Enabling Bluedroid...");
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register GATTC callback for esp_hid
    esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);

    // Initialize HID Host
    ESP_LOGI(TAG, "Initializing HID Host...");
    esp_hidh_config_t hidh_config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ret = esp_hidh_init(&hidh_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID Host init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "HID Host Initialized.");

    // Set BLE Security Parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Register GAP Callbacks
    esp_bt_gap_register_callback(bt_gap_cb);
    esp_ble_gap_register_callback(ble_gap_cb);

    // Set discoverable/connectable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Start Discovery (Dual Mode)
    ESP_LOGI(TAG, "Starting discovery...");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    
    esp_ble_gap_set_scan_params(&ble_scan_params);
    esp_ble_gap_start_scanning(0);

    // Main Loop: Handle Connection Requests
    connect_req_t req;
    while (1) {
        if (xQueueReceive(s_connect_queue, &req, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Processing connection request for: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(req.bda));
            esp_hidh_dev_open(req.bda, req.transport, req.ble_addr_type);
        }
    }
}