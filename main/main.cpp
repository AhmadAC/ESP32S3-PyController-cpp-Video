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
static uint8_t my_mac[6]   = {0}; // PyController MAC
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

// Helper to draw clean circles without background white rectangles
void fill_circle(LCD& lcd, int x0, int y0, int r, uint16_t color) {
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                lcd.draw_pixel(x0 + x, y0 + y, color);
            }
        }
    }
}

// --- RAW Wi-Fi Promiscuous Callback for High-Speed MJPEG ---
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) return;
    
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint8_t *payload = pkt->payload;
    uint16_t pkt_len = pkt->rx_ctrl.sig_len;

    if (pkt_len < 33) return; 

    uint16_t fc;
    memcpy(&fc, payload, 2);
    if (fc != 0x0008) return; 

    if (memcmp(payload + 4, my_mac, 6) != 0 && memcmp(payload + 4, broadcast_mac, 6) != 0) return;

    uint8_t *custom = payload + 24;
    if (custom[0] == 'C' && custom[1] == 'A' && custom[2] == 'M') {
        if (img_ready) return; 

        uint16_t chunk_idx, total_chunks, len;
        memcpy(&chunk_idx, custom + 3, 2);
        memcpy(&total_chunks, custom + 5, 2);
        memcpy(&len, custom + 7, 2);
        
        if (chunk_idx == 0) {
            if (total_chunks > 100) return; 
            
            if (img_buf) {
                heap_caps_free(img_buf);
                img_buf = nullptr;
            }
            img_buf = (uint8_t*)heap_caps_malloc(total_chunks * 1400, MALLOC_CAP_8BIT);
            img_chunks_received = 0;
            img_total_chunks = total_chunks;
            img_len = 0;
        }

        if (img_buf && chunk_idx == img_chunks_received && total_chunks == img_total_chunks) {
            if (img_len + len <= total_chunks * 1400) {
                memcpy(img_buf + img_len, custom + 9, len);
                img_len += len;
                img_chunks_received++;
                
                if (img_chunks_received == img_total_chunks) {
                    img_ready = true;
                }
            }
        }
    }
}

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
            if (!esp_now_is_peer_exist(peer_mac)) esp_now_add_peer(&peer_info);
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
            if (!esp_now_is_peer_exist(cam_mac)) esp_now_add_peer(&peer_info);
        }
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
            if (sscanf(l_ptr, "L:%d", &lf) == 1) line_follower_state = (lf == 1);
        }

        char *x_ptr = strstr(buf, "X:");
        if (x_ptr) {
            int sync = 0;
            if (sscanf(x_ptr, "X:%d", &sync) == 1) sync_state = (sync == 1);
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
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s).", esp_err_to_name(spiffs_ret));
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, my_mac));

    wifi_promiscuous_filter_t filter = {};
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT;
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

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
    bool last_start_state = false;
    bool last_back_state = false;
    
    bool show_camera_feed = false;
    bool sonar_active = true;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        if (!has_car || !has_cam) {
            if (pdTICKS_TO_MS(now - last_discover) >= 3000) {
                esp_now_send(broadcast_mac, (const uint8_t*)"pyCAR_DISCOVER", 14);
                last_discover = now;
            }
        }

        // Draw image asynchronously to the screen whenever one is ready.
        // This will display both live stream frames AND manual 'X' button snapshots.
        if (img_ready) {
            lcd.draw_jpg_mem(img_buf, img_len, -40, 0);
            img_ready = false;
        }

        // A. RATE-LIMITED LCD UPDATE (HUD Mode)
        if (pdTICKS_TO_MS(now - last_lcd_update) >= 200) {
            // Only draw HUD elements if the camera stream is NOT active
            if (has_car && !show_camera_feed) {
                
                // Draw Sonar Distance Text
                if (sonar_active) {
                    if (strcmp(distance_str, last_dist_str_on_screen) != 0) {
                        char padded_text[32];
                        snprintf(padded_text, sizeof(padded_text), "%-12s", distance_str);
                        lcd.draw_string(10, 190, padded_text, COLOR_BLACK, COLOR_WHITE, 2);
                        strcpy(last_dist_str_on_screen, distance_str);
                    }
                }

                // Draw Top-Right Graphic Indicators
                if (line_follower_state != last_lf_state || sync_state != last_sync_state) {
                    
                    // If a circle turned OFF, seamlessly redraw the base image to erase it cleanly
                    if ((last_lf_state && !line_follower_state) || (last_sync_state && !sync_state)) {
                        lcd.draw_jpg("/Car.jpg", 0, 0);
                        strcpy(last_dist_str_on_screen, ""); // Force text redraw after background erase
                    }

                    // Redraw active state graphics
                    if (line_follower_state) fill_circle(lcd, 220, 20, 6, COLOR_BLACK);
                    if (sync_state) fill_circle(lcd, 195, 20, 6, COLOR_RED);

                    last_lf_state = line_follower_state;
                    last_sync_state = sync_state;
                }
            } 
            last_lcd_update = now;
        }

        // B. RATE-LIMITED GAMEPAD TX (50ms interval / ~20Hz)
        if (pdTICKS_TO_MS(now - last_tx_update) >= 50) {
            GamepadState state = gamepad.read();
            
            // X Button (Request Single High-Quality Photo)
            if (state.x && !last_x_state) {
                if (has_cam) esp_now_send(cam_mac, (const uint8_t*)"pyCAM_REQ", 9);
            }
            last_x_state = state.x;

            // START Button (Toggle Camera Live Stream to LCD)
            if (state.start && !last_start_state) {
                if (has_cam) {
                    show_camera_feed = !show_camera_feed;
                    if (show_camera_feed) {
                        esp_now_send(cam_mac, (const uint8_t*)"pyCAM_STR_1", 11);
                    } else {
                        esp_now_send(cam_mac, (const uint8_t*)"pyCAM_STR_0", 11);
                        
                        // When exiting camera, instantly restore PyCar backdrop and refresh UI variables
                        lcd.draw_jpg("/Car.jpg", 0, 0);
                        strcpy(last_dist_str_on_screen, "");
                        
                        if (line_follower_state) fill_circle(lcd, 220, 20, 6, COLOR_BLACK);
                        if (sync_state) fill_circle(lcd, 195, 20, 6, COLOR_RED);
                        
                        last_lf_state = line_follower_state;
                        last_sync_state = sync_state;
                    }
                }
            }
            last_start_state = state.start;

            // BACK Button (Toggle Car Hardware Forward Restriction Sonar)
            if (state.back && !last_back_state) {
                sonar_active = !sonar_active;
                if (sonar_active) {
                    if (has_car) esp_now_send(peer_mac, (const uint8_t*)"pyCAR_SONAR_1", 13);
                    strcpy(last_dist_str_on_screen, ""); // Force text to reappear
                } else {
                    if (has_car) esp_now_send(peer_mac, (const uint8_t*)"pyCAR_SONAR_0", 13);
                    
                    // Seamlessly erase the text by redrawing the backdrop if not in video mode
                    if (!show_camera_feed) {
                        lcd.draw_jpg("/Car.jpg", 0, 0);
                        if (line_follower_state) fill_circle(lcd, 220, 20, 6, COLOR_BLACK);
                        if (sync_state) fill_circle(lcd, 195, 20, 6, COLOR_RED);
                        strcpy(last_dist_str_on_screen, "");
                        
                        last_lf_state = line_follower_state;
                        last_sync_state = sync_state;
                    }
                }
            }
            last_back_state = state.back;

            // Hardware Stick Mapping & Output Transmission
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