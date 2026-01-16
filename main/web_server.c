#include "web_server.h"
#include "index_html.h"
#include "config.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "cJSON.h"
#include <stdio.h>

static const char *TAG = "WEB";
static httpd_handle_t server = NULL;

// WebSocket file descriptor
static int ws_fd = -1; 

// Handler for HTML
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler for WebSocket
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // We don't really expect data FROM client, but good to handle ping/pong
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;
    
    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }
        // ESP_LOGI(TAG, "WS Received: %s", ws_pkt.payload);
        free(buf);
    }
    return ESP_OK;
}

// Handler for /help
static esp_err_t help_get_handler(httpd_req_t *req)
{
    // 1. Gather Data & Build JSON
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "project", "RControl ESP32 Firmware");
    
    char ip_str[32] = "unknown";
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(netif, &ip_info);
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    }
    cJSON_AddStringToObject(root, "ip", ip_str);

    cJSON *pins = cJSON_CreateObject();
    #define ADD_PIN(p) cJSON_AddNumberToObject(pins, #p, p)
    ADD_PIN(MOTOR_LEFT_ENA_PIN);
    ADD_PIN(MOTOR_LEFT_IN1_PIN);
    ADD_PIN(MOTOR_LEFT_IN2_PIN);
    ADD_PIN(MOTOR_RIGHT_ENB_PIN);
    ADD_PIN(MOTOR_RIGHT_IN3_PIN);
    ADD_PIN(MOTOR_RIGHT_IN4_PIN);
    ADD_PIN(US_TRIGGER_PIN);
    ADD_PIN(US_ECHO_PIN);
    ADD_PIN(BUMPER_FRONT_LEFT);
    ADD_PIN(BUMPER_FRONT_RIGHT);
    ADD_PIN(BUMPER_REAR_LEFT);
    ADD_PIN(BUMPER_REAR_RIGHT);
    ADD_PIN(LED_LEFT_TURN);
    ADD_PIN(LED_RIGHT_TURN);
    #undef ADD_PIN

    cJSON_AddItemToObject(root, "pins", pins);

    char *json_str = cJSON_Print(root); // Pretty print

    // 2. Start HTML Response
    httpd_resp_set_type(req, "text/html");
    
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><head><title>RControl Help</title>"
                                  "<style>body{font-family:sans-serif;} table{border-collapse:collapse;} "
                                  "th,td{border:1px solid #ddd;padding:8px;} th{background-color:#f2f2f2;}</style>"
                                  "</head><body>");
    
    char buf[128];
    snprintf(buf, sizeof(buf), "<h1>RControl ESP32 Firmware</h1><p><strong>IP Address:</strong> %s</p>", ip_str);
    httpd_resp_sendstr_chunk(req, buf);

    httpd_resp_sendstr_chunk(req, "<h2>Pin Assignments</h2><table><tr><th>Name</th><th>Pin</th></tr>");

    #define SEND_ROW(name, pin) \
        snprintf(buf, sizeof(buf), "<tr><td>" #name "</td><td>%d</td></tr>", pin); \
        httpd_resp_sendstr_chunk(req, buf)

    SEND_ROW(MOTOR_LEFT_ENA_PIN, MOTOR_LEFT_ENA_PIN);
    SEND_ROW(MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN1_PIN);
    SEND_ROW(MOTOR_LEFT_IN2_PIN, MOTOR_LEFT_IN2_PIN);
    SEND_ROW(MOTOR_RIGHT_ENB_PIN, MOTOR_RIGHT_ENB_PIN);
    SEND_ROW(MOTOR_RIGHT_IN3_PIN, MOTOR_RIGHT_IN3_PIN);
    SEND_ROW(MOTOR_RIGHT_IN4_PIN, MOTOR_RIGHT_IN4_PIN);
    SEND_ROW(US_TRIGGER_PIN, US_TRIGGER_PIN);
    SEND_ROW(US_ECHO_PIN, US_ECHO_PIN);
    SEND_ROW(BUMPER_FRONT_LEFT, BUMPER_FRONT_LEFT);
    SEND_ROW(BUMPER_FRONT_RIGHT, BUMPER_FRONT_RIGHT);
    SEND_ROW(BUMPER_REAR_LEFT, BUMPER_REAR_LEFT);
    SEND_ROW(BUMPER_REAR_RIGHT, BUMPER_REAR_RIGHT);
    SEND_ROW(LED_LEFT_TURN, LED_LEFT_TURN);
    SEND_ROW(LED_RIGHT_TURN, LED_RIGHT_TURN);
    
    #undef SEND_ROW

    httpd_resp_sendstr_chunk(req, "</table><h2>JSON Data</h2><pre>");
    if (json_str) {
        httpd_resp_sendstr_chunk(req, json_str);
        free(json_str);
    }
    httpd_resp_sendstr_chunk(req, "</pre></body></html>");
    
    httpd_resp_send_chunk(req, NULL, 0);

    cJSON_Delete(root);
    return ESP_OK;
}

void web_server_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7; // Increase if needed

    httpd_uri_t root_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t ws_uri = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };

    httpd_uri_t help_uri = {
        .uri        = "/help",
        .method     = HTTP_GET,
        .handler    = help_get_handler,
        .user_ctx   = NULL
    };

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &ws_uri);
        httpd_register_uri_handler(server, &help_uri);
    } else {
        ESP_LOGE(TAG, "Error starting server!");
    }
}

#include "i2c_manager.h"

// Helper to push data to all connected WS clients
// Note: ESP-IDF's simple WS implementation in examples usually iterates FDs.
// httpd_queue_work is safer for broadcasting from other tasks.

struct async_ctx {
    char *data;
};

static void ws_async_send(void *arg)
{
    char *data = (char *)arg;
    
    // We want to send to all clients. HTTPD API makes this a bit tricky without keeping track.
    // However, we can use httpd_get_client_list if available or just assume single client for this prototype 
    // or improve it later. 
    // Actually, httpd_ws_send_frame_async requires a req handle or fd.
    // For simplicity in this demo: We will only support broadcasting if we track FDs or use specific API.
    // A simpler way for a robot dashboard often accessed by one device is fine.
    
    // BETTER APPROACH: Use httpd_queue_work combined with a function that iterates open sessions.
    // But httpd_get_client_list is only available in newer versions.
    
    // Let's use a broadcast-like approach by "guessing" or just sending to the last known FD? 
    // No, that's brittle.
    // Correct way in ESP-IDF v4.x/5.x: 
    
    size_t fds = 7;
    int client_fds[7];
    if (httpd_get_client_list(server, &fds, client_fds) == ESP_OK) {
        for (int i = 0; i < fds; i++) {
            // Check if this FD is a websocket
            // (There isn't a direct API to check if FD is WS, but sending WS frame to non-WS might fail gracefully or close it)
            // Ideally we should track which FDs are WS in the handshake handler.
            // For now, we try sending.
            
            httpd_ws_frame_t ws_pkt;
            memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
            ws_pkt.payload = (uint8_t*)data;
            ws_pkt.len = strlen(data);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            
            httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
        }
    }
    
    free(data);
}

void web_server_broadcast_msg(const char *json_data) {
    if (!server) return;
    char *data_copy = strdup(json_data);
    if (data_copy) {
        httpd_queue_work(server, ws_async_send, data_copy);
    }
}

void web_server_broadcast_log(const char *fmt, va_list args) {
    char buf[256];
    char json[300];
    vsnprintf(buf, sizeof(buf), fmt, args);
    
    // Escape standard JSON chars if needed, but for logs, basic text is usually fine.
    // For robust JSON, we should escape quotes/newlines in 'buf'.
    // cJSON is safer.
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "log");
    cJSON_AddStringToObject(root, "data", buf);
    const char *json_str = cJSON_PrintUnformatted(root);
    
    web_server_broadcast_msg(json_str);
    
    free((void*)json_str); // cJSON_Print allocates
    cJSON_Delete(root);
}

void web_server_update_status(int ml, int mr, bool b_fl, bool b_fr, bool b_rl, bool b_rr, bool ll, bool lr, bool bt_connected) {
    // Send status ~5-10 times a second max to save bandwidth
    static int64_t last_send = 0;
    if (esp_timer_get_time() - last_send < 100000) return; // 100ms
    last_send = esp_timer_get_time();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "status");
    cJSON_AddNumberToObject(root, "ml", ml);
    cJSON_AddNumberToObject(root, "mr", mr);
    
    cJSON *bumpers = cJSON_CreateArray();
    cJSON_AddItemToArray(bumpers, cJSON_CreateBool(b_fl));
    cJSON_AddItemToArray(bumpers, cJSON_CreateBool(b_fr));
    cJSON_AddItemToArray(bumpers, cJSON_CreateBool(b_rl));
    cJSON_AddItemToArray(bumpers, cJSON_CreateBool(b_rr));
    cJSON_AddItemToObject(root, "b", bumpers);

    cJSON_AddBoolToObject(root, "ll", ll);
    cJSON_AddBoolToObject(root, "lr", lr);
    
    cJSON_AddBoolToObject(root, "bt", bt_connected);

    // Add I2C Data
    i2c_manager_get_json(root);

    const char *json_str = cJSON_PrintUnformatted(root);
    web_server_broadcast_msg(json_str);
    free((void*)json_str);
    cJSON_Delete(root);
}
