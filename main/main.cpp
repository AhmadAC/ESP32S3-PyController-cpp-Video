// src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_camera.h"

// ----------------------------------------------------
// Seeed Studio XIAO ESP32S3 Sense OV2640 Pinout
// ----------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

uint8_t pyControllerMac[6];
volatile bool isConnected = false;
volatile bool requestImage = false;
bool lastBtnX = false;

// Packet structure to chunk the RGB565 Image
#pragma pack(push, 1)
struct ImageChunk {
    uint8_t magic;      // 0xCC (Identifier for image chunks)
    uint8_t type;       // 0x01
    uint16_t chunk_id;  // Identifier for assembly on the receiver
    uint8_t data[240];  // Half a row of pixels (120 pixels * 2 bytes = 240 bytes)
};
#pragma pack(pop)

// ----------------------------------------------------
// ESP-NOW Receive Callback (Listens for Handshake / Controls)
// ----------------------------------------------------
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (!isConnected && len >= 14 && strncmp((const char*)incomingData, "pyCAR_DISCOVER", 14) == 0) {
        Serial.println("Received 'pyCAR_DISCOVER' via ESP-NOW!");
        memcpy(pyControllerMac, mac, 6);
        
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, pyControllerMac, 6);
        peerInfo.channel = 1; 
        peerInfo.encrypt = false;
        
        if (!esp_now_is_peer_exist(pyControllerMac)) {
            esp_now_add_peer(&peerInfo);
        }

        const char* ackMsg = "pyCAM_ACK";
        esp_now_send(pyControllerMac, (uint8_t *)ackMsg, strlen(ackMsg));
        
        Serial.println("Sent 'pyCAM_ACK' via ESP-NOW. Connected!");
        isConnected = true;
    } 
    // Intercept Gamepad Control Packet (6 bytes, starts with 67)
    else if (isConnected && len == 6 && incomingData[0] == 67) {
        uint8_t btns = incomingData[5];
        bool btnX = (btns & (1 << 7)) != 0; // X button is stored at bit 7
        
        // Detect a rising edge (button was just pressed)
        if (btnX && !lastBtnX) {
            requestImage = true;
            Serial.println("X Button Pressed! Queuing image capture...");
        }
        lastBtnX = btnX;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Standard ESP-NOW Initialization
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(1);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("Waiting for PyController to broadcast 'pyCAR_DISCOVER'...");
    
    // --- Camera Initialization ---
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565; // Direct screen colors (no JPEG decoding needed)
    config.frame_size   = FRAMESIZE_QVGA;   // 320x240
    config.jpeg_quality = 12;               // Ignored for RGB565
    config.fb_count     = 1;                // Memory efficiency
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    Serial.println("Camera initialized! Waiting for X Button input.");
}

void loop() {
    if (isConnected && requestImage) {
        requestImage = false; // Reset flag to only send one image per press
        
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            if (fb->format == PIXFORMAT_RGB565 && fb->width >= 320 && fb->height >= 240) {
                Serial.println("Sending image chunks...");
                
                // Crop image horizontally (320px down to 240px to fit square controller screen)
                int start_x = (fb->width - 240) / 2;
                int start_y = (fb->height - 240) / 2; // Usually 0 since height is already 240
                
                uint16_t chunk_id = 0;
                ImageChunk packet;
                packet.magic = 0xCC;
                packet.type = 0x01;
                
                for (int y = 0; y < 240; y++) {
                    // Send one row of 240 pixels in two halves (120 pixels each)
                    for (int half = 0; half < 2; half++) {
                        packet.chunk_id = chunk_id++;
                        int x_offset = start_x + (half * 120);
                        
                        // Copy 120 pixels (240 bytes) to the packet
                        uint8_t *src = fb->buf + ((start_y + y) * fb->width + x_offset) * 2;
                        memcpy(packet.data, src, 240);
                        
                        // Robust ESP-NOW send with retry
                        esp_err_t err;
                        int retries = 0;
                        do {
                            err = esp_now_send(pyControllerMac, (uint8_t*)&packet, sizeof(packet));
                            if (err != ESP_OK) {
                                delay(1); // Backoff if wifi queue gets full
                                retries++;
                            }
                        } while (err != ESP_OK && retries < 10);
                        
                        delay(1); // Small artificial spacer between transmission chunks
                    }
                }
                Serial.println("Full Photo frame sent successfully.");
            } else {
                Serial.println("Incorrect frame format. Required: RGB565 / QVGA.");
            }
            esp_camera_fb_return(fb);
        } else {
            Serial.println("Failed to capture image.");
        }
    }
    
    // Yield time to the watchdog and network tasks
    delay(10);
}