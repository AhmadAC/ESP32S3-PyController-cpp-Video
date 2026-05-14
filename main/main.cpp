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
#include "esp_heap_caps.h"

#include "lcd.h"
#include "gamepad.hpp"

static const char *TAG = "pyController";

// --- Global State Variables ---
static volatile bool has_car = false;
static volatile bool has_cam = false;

static uint8_t peer_mac[6] = {0}; // Car MAC
static uint8_t cam_mac[6]  = {0}; // Cam MAC
static const uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static char distance_str[32] = "0.00 cm";
static bool line_follower_state = false;
static bool sync_state = false;

// --- Image Reception Variables ---
static uint8_t* img_buf = nullptr;
static size_t img_len = 0;
static int img_chunks_received = 0;
static int img_total_chunks = 0;
static volatile bool img_ready = false;

// --- ESP-NOW Receive Callback ---
void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data == NULL || data_len <= 0) return;

    if (data_len == 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        if (!has_car) {
            memcpy((void*)peer_mac, esp_now_info->src_addr, 6);
            has_car = true;
            esp_now_peer_info_t peer_info = {};
            peer_info.channel = 1;
            peer_info.encrypt = false;
            memcpy(peer_info.peer_addr, peer_mac, 6);
            if (!esp_now_is_peer_exist(peer_mac)) {
                esp_now_add_peer(&peer_info);
            }
        }
        return;
    }
    
    if (data_len == 9 && memcmp(data, "pyCAM_ACK", 9) == 0) {
        if (!has_cam) {
            memcpy((void*)cam_mac, esp_now_info->src_addr, 6);
            has_cam = true;
            esp_now_peer_info_t peer_info = {};
            peer_info.channel = 1;
            peer_info.encrypt = false;
            memcpy(peer_info.peer_addr, cam_mac, 6);
            if (!esp_now_is_peer_exist(cam_mac)) {
                esp_now_add_peer(&peer_info);
            }
        }
        return;
    }

    // Reassemble incoming image chunks into active memory
    if (data_len >= 8 && data[0] == 'C' && data[1] == 'I') {
        uint16_t chunk_idx; memcpy(&chunk_idx, &data[2], 2);
        uint16_t total_chunks; memcpy(&total_chunks, &data[4], 2);
        uint16_t len; memcpy(&len, &data[6], 2);
        
        if (chunk_idx == 0) {
            if (total_chunks > 400) return; // Ignore wildly unsafe sizes

            if (img_buf) {
                heap_caps_free(img_buf);
                img_buf = nullptr;
            }
            img_buf = (uint8_t*)heap_caps_malloc(total_chunks * 240, MALLOC_CAP_8BIT); 
            img_chunks_received = 0;
            img_total_chunks = total_chunks;
            img_len = 0;
            img_ready = false;
        }
        
        if (img_buf && chunk_idx == img_chunks_received && total_chunks == img_total_chunks) {
            memcpy(img_buf + img_len, &data[8], len);
            img_len += len;
            img_chunks_received++;
            
            if (img_chunks_received == img_total_chunks) {
                img_ready = true;
            }
        }
        return;
    }
    
    // Existing pyCar state logic telemetry
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

    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false 
    };
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (spiffs_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s). Background images will be skipped.", esp_err_to_name(spiffs_ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted successfully!");
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
    lcd.draw_string(10, 130, "pyCar/pyCam...", COLOR_BLACK, COLOR_WHITE, 2);

    // Initial sequence discovery boot blocker. Boot sequence breaks free after finding at least one node!
    while (!has_car && !has_cam) {
        esp_now_send(broadcast_mac, (const uint8_t*)"pyCAR_DISCOVER", 14);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lcd.fill_screen(COLOR_WHITE);
    lcd.draw_jpg("/Car.jpg", 0, 0); 

    TickType_t last_lcd_update = xTaskGetTickCount();
    TickType_t last_tx_update = xTaskGetTickCount();
    TickType_t last_discover = xTaskGetTickCount();

    char last_dist_str_on_screen[32] = "";
    bool last_lf_state = false;
    bool last_sync_state = false;
    bool last_x_state = false;
    bool last_cam_state = false;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // Dynamically continue hunting globally if one device hasn't booted up yet
        if (!has_car || !has_cam) {
            if (pdTICKS_TO_MS(now - last_discover) >= 3000) {
                esp_now_send(broadcast_mac, (const uint8_t*)"pyCAR_DISCOVER", 14);
                last_discover = now;
            }
        }

        // Draw image asynchronously exactly when buffer clears ready state!
        if (img_ready) {
            // Draw QVGA (320x240) shifted -40x centering perfectly onto the 240x240 hardware boundaries
            lcd.draw_jpg_mem(img_buf, img_len, -40, 0);
            img_ready = false;
            
            // Deliberately reset UI cache variables so HUD overlays write cleanly on top of image next pass
            strcpy(last_dist_str_on_screen, "");
            last_lf_state = !line_follower_state;
            last_sync_state = !sync_state;
            last_cam_state = !has_cam; 
            
            lcd.draw_string(10, 220, "          ", COLOR_WHITE, COLOR_WHITE, 1); // clears req text
        }

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
            if (has_cam != last_cam_state) {
                const char* icon = has_cam ? "[C]" : "   ";
                lcd.draw_string(150, 10, icon, COLOR_BLUE, COLOR_WHITE, 2);
                last_cam_state = has_cam;
            }
            last_lcd_update = now;
        }

        // B. RATE-LIMITED GAMEPAD TX (50ms interval / ~20Hz)
        if (pdTICKS_TO_MS(now - last_tx_update) >= 50) {
            GamepadState state = gamepad.read();
            
            // X Button Pulse Event Handler
            if (state.x && !last_x_state) {
                if (has_cam) {
                    esp_now_send(cam_mac, (const uint8_t*)"pyCAM_REQ", 9);
                    lcd.draw_string(10, 220, "Req Cam...", COLOR_RED, COLOR_WHITE, 1);
                }
            }
            last_x_state = state.x;

            uint8_t lx = (state.left_x / 14) > 255 ? 255 : (state.left_x / 14);
            uint8_t ly = (state.left_y / 14) > 255 ? 255 : (state.left_y / 14);
            uint8_t rx = (state.right_x / 14) > 255 ? 255 : (state.right_x / 14);
            uint8_t ry = (state.right_y / 14) > 255 ? 255 : (state.right_y / 14);
            
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
            
            if (has_car) {
                esp_now_send(peer_mac, payload, sizeof(payload));
            }

            last_tx_update = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}