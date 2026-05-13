// src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Gamepad Pinout (from your mpconfigboard.h) ---
#define BTN_START 0
#define BTN_BACK  1
#define BTN_A_OK  6
#define BTN_B_OK  9
#define BTN_UP    10
#define BTN_DOWN  11
#define BTN_LEFT  12
#define BTN_RIGHT 13
#define BTN_X     14
#define BTN_Y     15
#define BTN_A     16
#define BTN_B     21

// --- Analog Stick ADC Channels (from your mpconfigboard.h) ---
#define ADC_JOY_LX 3 // ADC1_CHANNEL_3 -> GPIO 4
#define ADC_JOY_LY 4 // ADC1_CHANNEL_4 -> GPIO 5
#define ADC_JOY_RX 6 // ADC1_CHANNEL_6 -> GPIO 7
#define ADC_JOY_RY 7 // ADC1_CHANNEL_7 -> GPIO 8

// --- Library Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// --- State Management ---
enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
uint8_t peerMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- Telemetry Data ---
float currentDist = 0.0;
bool lineFollowerActive = false;

// --- Timers ---
unsigned long lastDiscoveryTime = 0;
unsigned long lastJoystickTime = 0;

// === JPEG Drawing Callback (High-Speed RAM to Screen) ===
int JPEGDraw(JPEGDRAW *pDraw) {
    tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    return 1; // Continue drawing
}

// === Raw Wi-Fi Video Frame Received Callback ===
void onVideoFrame(size_t length) {
    if (currentState != CONNECTED) return;
    
    // FIXED: getBuffer() is the correct method for EspNowCam v0.2.1
    uint8_t *buffer = radio.getBuffer();
    
    if (jpeg.openRAM(buffer, length, JPEGDraw)) {
        jpeg.setPixelType(RGB565_BIG_ENDIAN); 
        jpeg.decode(0, 0, 0);                 
        jpeg.close();

        // --- Overlay Telemetry Text on top of the video feed ---
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(TL_DATUM); 
        tft.drawString("Dist: " + String(currentDist, 2) + " cm  ", 5, 220, 2);
        
        if (lineFollowerActive) {
            tft.fillCircle(220, 20, 10, TFT_BLACK);
        } else {
            // FIXED: readPixel is the correct method for TFT_eSPI
            tft.fillCircle(220, 20, 10, tft.readPixel(220, 20)); 
            tft.drawCircle(220, 20, 10, TFT_WHITE);
        }
    }
}

// === ESP-NOW Data Received Callback (Handshake & Telemetry) ===
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && strncmp((const char*)data, "pyCAR_ACK", 9) == 0) {
        memcpy(peerMac, mac, 6);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerMac, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        if(esp_now_is_peer_exist(peerMac)) esp_now_mod_peer(&peerInfo);
        else esp_now_add_peer(&peerInfo);
        currentState = CONNECTED;
        tft.fillScreen(TFT_BLACK);
        Serial.println("Paired to Car!");
    } 
    else if (len > 3 && data[0] == 'D' && data[1] == ':') {
        char msg[64];
        int cpyLen = len < 63 ? len : 63;
        memcpy(msg, data, cpyLen);
        msg[cpyLen] = '\0';
        char* d_ptr = strstr(msg, "D:");
        char* l_ptr = strstr(msg, "L:");
        if (d_ptr) currentDist = atof(d_ptr + 2);
        if (l_ptr) lineFollowerActive = (l_ptr[2] == '1');
    }
}

uint8_t getDPadAndButtons() {
    uint8_t keyByte5 = 8;
    bool up = !digitalRead(BTN_UP);
    bool down = !digitalRead(BTN_DOWN);
    bool left = !digitalRead(BTN_LEFT);
    bool right = !digitalRead(BTN_RIGHT);
    if (up) keyByte5 = (right) ? 1 : (left) ? 7 : 0;
    else if (down) keyByte5 = (right) ? 3 : (left) ? 5 : 4;
    else if (right) keyByte5 = 2;
    else if (left) keyByte5 = 6;
    if (!digitalRead(BTN_Y)) keyByte5 |= (1 << 4);
    if (!digitalRead(BTN_B)) keyByte5 |= (1 << 5);
    if (!digitalRead(BTN_A)) keyByte5 |= (1 << 6);
    if (!digitalRead(BTN_X)) keyByte5 |= (1 << 7);
    return keyByte5;
}

uint8_t getAnalog(int channel) {
    return analogRead(channel) / 16; 
}

void setup() {
    Serial.begin(115200);
    const int buttons[] = {BTN_START, BTN_BACK, BTN_A_OK, BTN_B_OK, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_X, BTN_Y, BTN_A, BTN_B};
    for (int pin : buttons) pinMode(pin, INPUT_PULLUP);
    tft.init();
    tft.setRotation(0); 
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Booting Controller...", 20, 110, 4);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) return;
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_now_register_recv_cb(onDataRecv);
    radio.init(512); 
    radio.setRecvCallback(onVideoFrame);
    tft.fillScreen(TFT_WHITE);
    tft.drawString("Searching for Car...", 25, 110, 4);
}

void loop() {
    unsigned long now = millis();
    if (currentState == SEARCHING) {
        if (now - lastDiscoveryTime > 250) {
            esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
            lastDiscoveryTime = now;
        }
    } 
    else if (currentState == CONNECTED) {
        if (now - lastJoystickTime > 50) {
            uint8_t payload[6] = {67, getAnalog(ADC_JOY_LX), (uint8_t)(255 - getAnalog(ADC_JOY_LY)), getAnalog(ADC_JOY_RX), (uint8_t)(255 - getAnalog(ADC_JOY_RY)), getDPadAndButtons()};
            esp_now_send(peerMac, payload, 6);
            lastJoystickTime = now;
        }
    }
}
