// main/main.cpp
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "lcd.h"
#include "gamepad.hpp"

static const char *TAG = "pyController";

// --- Global State Variables ---
static volatile bool is_paired = false;
static volatile bool has_car = false;

static uint8_t peer_mac[6] = {0};
static const uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static char distance_str[32] = "0.00 cm";
static bool line_follower_state = false;
static bool sync_state = false;

// --- ESP-NOW Receive Callback ---
void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data == NULL || data_len <= 0) return;

    // Check if it's the pairing acknowledgement from PyCar
    if (data_len == 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        if (!is_paired) {
            memcpy((void*)peer_mac, esp_now_info->src_addr, 6);
            is_paired = true;
        }
        has_car = true;
        return;
    }
    
    // Check if it's a telemetry update from PyCar (e.g., "D:12.34,L:1,X:0")
    if (data_len >= 2 && data[0] == 'D' && data[1] == ':') {
        has_car = true;
        char buf[64] = {0};
        memcpy(buf, data, data_len < 63 ? data_len : 63);

        char *d_ptr = strstr(buf, "D:");
        if (d_ptr) {
            float dist = 0.0f;
            if (sscanf(d_ptr, "D:%f", &dist) == 1) {
                snprintf(distance_str, sizeof(distance_str), "%.2f cm", dist);
            }
        }

        char *l_ptr = strstr(buf, "L:");
        if (l_ptr) {
            int lf = 0;
            if (sscanf(l_ptr, "L:%d", &lf) == 1) {
                line_follower_state = (lf == 1);
            }
        }

        char *x_ptr = strstr(buf, "X:");
        if (x_ptr) {
            int sync = 0;
            if (sscanf(x_ptr, "X:%d", &sync) == 1) {
                sync_state = (sync == 1);
            }
        }
    }
}

// --- Application Entry Point ---
extern "C" void app_main(void) {
    // 1. Initialize Non-Volatile Storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Hardware Peripherals
    LCD lcd;
    lcd.init();
    
    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_string(10, 10, "Booting...", COLOR_BLACK, COLOR_WHITE, 2);

    Gamepad gamepad;
    gamepad.init();

    // 3. Initialize File System (SPIFFS)
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (spiffs_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(spiffs_ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS Mounted Successfully");
    }

    // 4. Initialize Wi-Fi and ESP-NOW
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    // Register Broadcast Peer
    esp_now_peer_info_t peer_info = {};
    peer_info.channel = 1;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, broadcast_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    // 5. Pairing Loop
    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_string(10, 100, "Searching for", COLOR_BLACK, COLOR_WHITE, 2);
    lcd.draw_string(10, 130, "pyCar...", COLOR_BLACK, COLOR_WHITE, 2);
    ESP_LOGI(TAG, "Searching for pyCar...");

    while (!is_paired) {
        esp_now_send(broadcast_mac, (const uint8_t*)"pyCAR_DISCOVER", 14);
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to allow receiving the ACK
    }

    ESP_LOGI(TAG, "Connected to Peer!");
    memcpy(peer_info.peer_addr, peer_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    // --- Initial UI Render --- (Draw the pyCar background image from flash)
    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_jpg("/spiffs/picture/Car.jpg", 0, 0); // Will render over the white canvas
    lcd.draw_string(10, 10, "Connected!", COLOR_GREEN, COLOR_WHITE, 2);
    lcd.draw_string(10, 160, "Ultrasonic:", COLOR_BLACK, COLOR_WHITE, 2);

    // --- State Timers & Caches ---
    TickType_t last_lcd_update = xTaskGetTickCount();
    TickType_t last_tx_update = xTaskGetTickCount();
    char last_dist_str_on_screen[32] = "";
    bool last_lf_state = false;
    bool last_sync_state = false;

    // 6. Main Control Loop
    while (true) {
        TickType_t now = xTaskGetTickCount();

        // A. RATE-LIMITED LCD UPDATE (200ms interval)
        if (pdTICKS_TO_MS(now - last_lcd_update) >= 200) {
            
            if (has_car) {
                // Distance Text Update
                if (strcmp(distance_str, last_dist_str_on_screen) != 0) {
                    char padded_text[32];
                    snprintf(padded_text, sizeof(padded_text), "%-12s", distance_str);
                    lcd.draw_string(10, 190, padded_text, COLOR_BLACK, COLOR_WHITE, 2);
                    strcpy(last_dist_str_on_screen, distance_str);
                }

                // Line Follower Icon Update
                if (line_follower_state != last_lf_state) {
                    const char* icon = line_follower_state ? "[L]" : "   ";
                    uint16_t fg_color = line_follower_state ? COLOR_BLACK : COLOR_WHITE;
                    lcd.draw_string(210, 10, icon, fg_color, COLOR_WHITE, 2);
                    last_lf_state = line_follower_state;
                }

                // Sync State Icon Update
                if (sync_state != last_sync_state) {
                    const char* icon = sync_state ? "[X]" : "   ";
                    uint16_t fg_color = sync_state ? COLOR_RED : COLOR_WHITE;
                    lcd.draw_string(180, 10, icon, fg_color, COLOR_WHITE, 2);
                    last_sync_state = sync_state;
                }
            } 
            last_lcd_update = now;
        }

        // B. RATE-LIMITED GAMEPAD TX (50ms interval / ~20Hz)
        if (pdTICKS_TO_MS(now - last_tx_update) >= 50) {
            GamepadState state = gamepad.read();
            
            uint8_t lx = (uint8_t)(state.left_x + 128);
            uint8_t ly = (uint8_t)(state.left_y + 128);
            uint8_t rx = (uint8_t)(state.right_x + 128);
            uint8_t ry = (uint8_t)(state.right_y + 128);
            
            uint8_t btns = 8; 
            if (state.up && state.right)         btns = 1;
            else if (state.right && state.down)  btns = 3;
            else if (state.left && state.down)   btns = 5;
            else if (state.left && state.up)     btns = 7;
            else if (state.up)                   btns = 0;
            else if (state.right)                btns = 2;
            else if (state.down)                 btns = 4;
            else if (state.left)                 btns = 6;
            
            if (state.x) btns |= (1 << 7);
            if (state.a) btns |= (1 << 6);
            if (state.b) btns |= (1 << 5);
            if (state.y) btns |= (1 << 4);

            uint8_t payload[6] = {67, lx, ly, rx, ry, btns};
            esp_now_send(peer_mac, payload, sizeof(payload));

            last_tx_update = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}