#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Pins ---
#define BTN_UP 10
#define BTN_DOWN 11
#define ADC_JOY_LX 4
#define ADC_JOY_LY 5

// --- Global Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// Small buffer for No-PSRAM stability
#define FRAME_BUFFER_SIZE 16384 
uint8_t frame_buffer[FRAME_BUFFER_SIZE]; 

volatile bool hasNewFrame = false;
volatile uint32_t latestFrameLength = 0;

enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
uint8_t carMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// JPEG Drawing Callback
int JPEGDraw(JPEGDRAW *pDraw) {
    if (pDraw->pPixels) tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
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

// Data Packet Callback
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        currentState = CONNECTED;
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000); 
    Serial.println("\n[1/4] Starting Display...");

    // Initialize Screen Only
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("pyController S3", 10, 10, 2);
    tft.drawString("Status: Hardware OK", 10, 30, 2);
    tft.drawString("Starting WiFi in 3s...", 10, 50, 2);

    // Wait to let memory settle
    delay(3000);
    Serial.println("[2/4] Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
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
    esp_now_add_peer(&peer);

    Serial.println("[4/4] Starting Video Engine...");
    radio.setRecvBuffer(frame_buffer);
    radio.setRecvCallback(onVideoFrame);
    
    // Low-res init for memory safety
    radio.init(160); 

    Serial.println("BOOT COMPLETE - NO CRASH");
    tft.fillScreen(TFT_BLACK);
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastSearch = 0;

    if (hasNewFrame) {
        // Valid JPEG header check
        if (latestFrameLength > 4 && frame_buffer[0] == 0xFF && frame_buffer[1] == 0xD8) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();
            }
        }
        hasNewFrame = false;
    }

    if (currentState == SEARCHING && now - lastSearch > 1000) {
        esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
        lastSearch = now;
        Serial.println("Searching for Car...");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("Searching for Car...", 10, 110, 2);
    }
}