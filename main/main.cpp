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

    if (data_len == 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        if (!is_paired) {
            memcpy((void*)peer_mac, esp_now_info->src_addr, 6);
            is_paired = true;
        }
        has_car = true;
        return;
    }
    
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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    LCD lcd;
    lcd.init();
    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_string(10, 10, "Booting...", COLOR_BLACK, COLOR_WHITE, 2);

    Gamepad gamepad;
    gamepad.init();

    // VFS mount point requires a prefix. 
    // This connects SPIFFS to "/spiffs" behind the scenes, allowing lcd.cpp to format the string for you!
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (spiffs_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(spiffs_ret));
    }

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

    esp_now_peer_info_t peer_info = {};
    peer_info.channel = 1;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, broadcast_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_string(10, 100, "Searching for", COLOR_BLACK, COLOR_WHITE, 2);
    lcd.draw_string(10, 130, "pyCar...", COLOR_BLACK, COLOR_WHITE, 2);

    while (!is_paired) {
        esp_now_send(broadcast_mac, (const uint8_t*)"pyCAR_DISCOVER", 14);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    memcpy(peer_info.peer_addr, peer_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    lcd.fill_screen(COLOR_WHITE);
    
    // Now you can easily use standard directory strings, no spiffs text required!
    lcd.draw_jpg("/picture/Car.jpg", 0, 0); 
    
    lcd.draw_string(10, 10, "Connected!", COLOR_GREEN, COLOR_WHITE, 2);
    lcd.draw_string(10, 160, "Ultrasonic:", COLOR_BLACK, COLOR_WHITE, 2);

    TickType_t last_lcd_update = xTaskGetTickCount();
    TickType_t last_tx_update = xTaskGetTickCount();
    char last_dist_str_on_screen[32] = "";
    bool last_lf_state = false;
    bool last_sync_state = false;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // A. RATE-LIMITED LCD UPDATE
        if (pdTICKS_TO_MS(now - last_lcd_update) >= 200) {
            if (has_car) {
                if (strcmp(distance_str, last_dist_str_on_screen) != 0) {
                    char padded_text[32];
                    snprintf(padded_text, sizeof(padded_text), "%-12s", distance_str);
                    lcd.draw_string(10, 190, padded_text, COLOR_BLACK, COLOR_WHITE, 2);
                    strcpy(last_dist_str_on_screen, distance_str);
                }
                if (line_follower_state != last_lf_state) {
                    const char* icon = line_follower_state ? "[L]" : "   ";
                    uint16_t fg_color = line_follower_state ? COLOR_BLACK : COLOR_WHITE;
                    lcd.draw_string(210, 10, icon, fg_color, COLOR_WHITE, 2);
                    last_lf_state = line_follower_state;
                }
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
            
            // Advanced mapping: Safely guarantees a 128 perfectly dead-center 
            // when letting go, completely solving rogue wheel movement.
            auto map_axis = [](uint16_t raw) -> uint8_t {
                // Wide deadband forces exact center to lock the car.py math at 0.
                if (raw > 1800 && raw < 2300) return 128; 
                
                if (raw <= 1800) {
                    return (uint8_t)((raw * 127) / 1800);
                } else {
                    int val = 128 + ((raw - 2300) * 127) / (4095 - 2300);
                    return (uint8_t)(val > 255 ? 255 : val);
                }
            };

            uint8_t lx = map_axis(state.left_x);
            uint8_t ly = map_axis(state.left_y);
            uint8_t rx = map_axis(state.right_x);
            uint8_t ry = map_axis(state.right_y);
            
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