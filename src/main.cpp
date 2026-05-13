#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Hardware Pins from Schematic ---
#define LCD_BL 33 // Backlight pin for 01Studio controller

// --- Global Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// Buffer for Video Frame
#define FRAME_BUFFER_SIZE 20000 
uint8_t frame_buffer[FRAME_BUFFER_SIZE]; 

volatile bool hasNewFrame = false;
volatile uint32_t latestFrameLength = 0;

enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
uint8_t carMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// JPEG Drawing Callback
int JPEGDraw(JPEGDRAW *pDraw) {
    if (pDraw->pPixels) {
        tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    }
    return 1;
}

// Video Packet Callback
void onVideoFrame(uint32_t length) {
    if (currentState != CONNECTED || hasNewFrame) return;
    if (length > 0 && length <= FRAME_BUFFER_SIZE) {
        latestFrameLength = length;
        hasNewFrame = true;
    }
}

// FIXED: Correct ESP-NOW Recv Callback signature for modern Arduino-ESP32 (v3.0+)
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, recv_info->src_addr, 6);
        currentState = CONNECTED;
        Serial.println("Paired with Car!");
    }
}

void setup() {
    Serial.begin(115200);
    
    // LCD Backlight logic
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    Serial.println("\n[1/4] Starting Display...");
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("pyController S3", 10, 10, 2);
    tft.drawString("Video Sync: Init...", 10, 30, 2);

    Serial.println("[2/4] Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Force specific channel for ESPNowCam compatibility
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Fail");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    Serial.println("[3/4] Setting up Radio...");
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.channel = 1;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
    }

    Serial.println("[4/4] Starting Video Engine...");
    radio.setRecvBuffer(frame_buffer);
    radio.setRecvCallback(onVideoFrame);
    
    // radio.init clears memory; ensure it happens after display is ready
    radio.init(160); 

    Serial.println("BOOT COMPLETE");
    tft.fillScreen(TFT_BLACK);
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastSearch = 0;

    if (hasNewFrame) {
        // Double check JPEG Start of Image (SOI) marker
        if (latestFrameLength > 4 && frame_buffer[0] == 0xFF && frame_buffer[1] == 0xD8) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();
            }
        }
        hasNewFrame = false;
    }

    // Discovery Logic
    if (currentState == SEARCHING && (now - lastSearch > 2000)) {
        esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
        lastSearch = now;
        Serial.println("Searching for pyCar...");
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString("SEARCHING FOR CAR...", 120, 110, 2);
    }
}