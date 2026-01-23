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
#include "gamepad.h"

static const char *TAG = "GAMEPAD";

static bool is_connected = false;
static esp_hidh_dev_t *s_connected_dev = NULL;
static gamepad_input_callback_t s_input_callback = NULL;

typedef struct {
    esp_bd_addr_t bda;
    esp_hid_transport_t transport;
    esp_ble_addr_type_t ble_addr_type;
} connect_req_t;

static QueueHandle_t s_connect_queue = NULL;

extern void esp_hidh_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static void parse_report(uint8_t *data, size_t length) {
    if (!s_input_callback) return;

    gamepad_state_t state = {0};
    
    // Default centers (8-bit)
    uint8_t raw_lx = 128, raw_ly = 128, raw_rx = 128, raw_ry = 128;

    // Mapping based on User Insight: Byte 0 is D-Pad.
    if (length >= 5) {
        raw_lx = data[1];
        raw_ly = data[2];
        raw_rx = data[3];
        raw_ry = data[4];
    }
    
    // Normalize 8-bit (0-255) to -1.0 .. 1.0
    float lx = ((float)raw_lx - 128.0f) / 128.0f;
    float ly = ((float)raw_ly - 128.0f) / 128.0f;
    float rx = ((float)raw_rx - 128.0f) / 128.0f;
    float ry = ((float)raw_ry - 128.0f) / 128.0f;

    // Invert Y axes (Standard: 0=Up/Neg, 255=Down/Pos)
    state.left_stick_x = lx;
    state.left_stick_y = -ly;
    state.right_stick_x = rx;
    state.right_stick_y = -ry;
    
    s_input_callback(&state);
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT:
        if (param->open.status == ESP_OK) {
            ESP_LOGI(TAG, "Connected to %s", esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_set_protocol(param->open.dev, ESP_HID_PROTOCOL_MODE_REPORT);
            is_connected = true;
            s_connected_dev = param->open.dev;
        } else {
            ESP_LOGE(TAG, "Open failed: %d", param->open.status);
            is_connected = false;
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            esp_ble_gap_start_scanning(0);
        }
        break;
    case ESP_HIDH_INPUT_EVENT:
        // Parse input
        parse_report(param->input.data, param->input.length);
        break;
    case ESP_HIDH_CLOSE_EVENT:
        ESP_LOGI(TAG, "Disconnected");
        is_connected = false;
        s_connected_dev = NULL;
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        esp_ble_gap_start_scanning(0);
        break;
    default:
        break;
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
        if (is_connected) return;
        char device_name[64] = {0};
        bool name_found = false;

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
            }
        }

        if (name_found) {
            if (strstr(device_name, "8BitDo") || strstr(device_name, "Wireless Controller") || strstr(device_name, "Pro Controller")) {
                ESP_LOGI(TAG, "Found: %s. Connecting...", device_name);
                esp_bt_gap_cancel_discovery();
                esp_ble_gap_stop_scanning();
                connect_req_t req;
                memcpy(req.bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
                req.transport = ESP_HID_TRANSPORT_BT;
                req.ble_addr_type = BLE_ADDR_TYPE_PUBLIC;
                xQueueSend(s_connect_queue, &req, 0);
            }
        }
    } else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED && !is_connected) {
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        }
    }
}

static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_SCAN_RESULT_EVT) {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (is_connected) return;
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (!adv_name) adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);

            if (adv_name) {
                char name[64] = {0};
                memcpy(name, adv_name, adv_name_len);
                if (strstr(name, "8BitDo") || strstr(name, "Wireless Controller") || strstr(name, "Pro Controller")) {
                    ESP_LOGI(TAG, "Found BLE: %s. Connecting...", name);
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
    }
}

static void connection_task(void *pvParameters) {
    connect_req_t req;
    while (1) {
        if (xQueueReceive(s_connect_queue, &req, portMAX_DELAY)) {
            esp_hidh_dev_open(req.bda, req.transport, req.ble_addr_type);
        }
    }
}

void gamepad_set_input_callback(gamepad_input_callback_t cb) {
    s_input_callback = cb;
}

esp_err_t gamepad_init(void) {
    esp_err_t ret;
    s_connect_queue = xQueueCreate(1, sizeof(connect_req_t));

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize BT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) return ret;
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) return ret;
    ret = esp_bluedroid_init();
    if (ret) return ret;
    ret = esp_bluedroid_enable();
    if (ret) return ret;

    esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);

    esp_hidh_config_t hidh_config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ret = esp_hidh_init(&hidh_config);
    if (ret) return ret;

    esp_bt_gap_register_callback(bt_gap_cb);
    esp_ble_gap_register_callback(ble_gap_cb);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    esp_ble_gap_start_scanning(0);

    xTaskCreate(connection_task, "bt_conn", 4096, NULL, 5, NULL);

    return ESP_OK;
}
