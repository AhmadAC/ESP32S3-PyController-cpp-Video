#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Pin Definitions (Verified from Schematic) ---
#define BTN_UP    10
#define BTN_DOWN  11
#define BTN_LEFT  12
#define BTN_RIGHT 13
#define BTN_Y     15
#define BTN_X     14
#define BTN_B     21
#define BTN_A     16
#define ADC_JOY_LX 4 
#define ADC_JOY_LY 5 
#define ADC_JOY_RX 7 
#define ADC_JOY_RY 8 

// --- Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// --- Memory Buffer (Reduced for No-PSRAM Stability) ---
#define FRAME_BUFFER_SIZE 20480 
uint8_t frame_buffer[FRAME_BUFFER_SIZE]; 

volatile bool hasNewFrame = false;
volatile uint32_t latestFrameLength = 0;

enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
uint8_t carMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

float currentDist = 0.0;
unsigned long lastDiscoveryTime = 0;
unsigned long lastJoystickTime = 0;

// JPEG Drawing Callback (Standard for TFT_eSPI)
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

// Data Packet Callback (Pairing & Telemetry)
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (!data || len <= 0) return;

    // Detect Pairing Handshake
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        currentState = CONNECTED;
        
        // Add specific car to ESP-NOW peers
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, carMac, 6);
        peer.channel = 1;
        peer.encrypt = false;
        if (!esp_now_is_peer_exist(carMac)) esp_now_add_peer(&peer);
        
        tft.fillScreen(TFT_BLACK);
    }
    // Detect Telemetry
    else if (currentState == CONNECTED && len > 2 && data[0] == 'D' && data[1] == ':') {
        char msg[32];
        int cpyLen = (len < 31) ? len : 31;
        memcpy(msg, data, cpyLen);
        msg[cpyLen] = '\0';
        char* d_ptr = strstr(msg, "D:");
        if (d_ptr) currentDist = atof(d_ptr + 2);
    }
}

void setup() {
    // 1. Initial Stability Delay
    delay(2000); 
    Serial.begin(115200);
    Serial.println("pyController S3 Booting...");

    // 2. Setup Input Pins
    const int buttons[] = {BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_X, BTN_Y, BTN_A, BTN_B};
    for (int p : buttons) pinMode(p, INPUT_PULLUP);

    // 3. Setup Display
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Status: Searching...", 10, 110, 2);

    // 4. WiFi & Radio Setup (Clean order)
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    // Register callback for non-video data
    esp_now_register_recv_cb(onDataRecv);

    // 5. Initialize Broadcast Peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // 6. Initialize Video Stream Handler
    radio.setRecvBuffer(frame_buffer);
    radio.setRecvCallback(onVideoFrame);
    radio.init(160); // 160px width mode (Fastest for ESP-NOW)

    Serial.println("Init Complete.");
}

void loop() {
    unsigned long now = millis();

    // Handle Video Frame Processing
    if (hasNewFrame) {
        // Double check for valid JPEG SOI marker to prevent crash
        if (latestFrameLength > 4 && frame_buffer[0] == 0xFF && frame_buffer[1] == 0xD8) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();

                // Draw Telemetry Overlay
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.drawString("Dist: " + String(currentDist, 1) + "cm", 5, 220, 2);
            }
        }
        hasNewFrame = false; 
    }

    // State Tasks
    if (currentState == SEARCHING) {
        if (now - lastDiscoveryTime > 1000) {
            esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
            lastDiscoveryTime = now;
            Serial.println("Searching...");
        }
    }
    else if (currentState == CONNECTED) {
        if (now - lastJoystickTime > 50) {
            // Read JoySticks (scaled to 0-255)
            uint8_t lx = analogRead(ADC_JOY_LX) / 16;
            uint8_t ly = 255 - (analogRead(ADC_JOY_LY) / 16);
            uint8_t rx = analogRead(ADC_JOY_RX) / 16;
            uint8_t ry = 255 - (analogRead(ADC_JOY_RY) / 16);

            // Read Buttons
            uint8_t buttons = 8; // Neutral D-Pad
            if (!digitalRead(BTN_UP)) buttons = 0;
            else if (!digitalRead(BTN_DOWN)) buttons = 4;
            if (!digitalRead(BTN_Y)) buttons |= (1 << 4);
            if (!digitalRead(BTN_A)) buttons |= (1 << 6);

            uint8_t payload[6] = {67, lx, ly, rx, ry, buttons};
            esp_now_send(carMac, payload, 6);
            lastJoystickTime = now;
        }
    }
}
