// src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Gamepad Pinout ---
#define BTN_START 0
#define BTN_BACK  1
#define BTN_UP    10
#define BTN_DOWN  11
#define BTN_LEFT  12
#define BTN_RIGHT 13
#define BTN_X     14
#define BTN_Y     15
#define BTN_A     16
#define BTN_B     21

// --- Analog Stick ADC Channels ---
#define ADC_JOY_LX 3
#define ADC_JOY_LY 4
#define ADC_JOY_RX 6
#define ADC_JOY_RY 7

// --- Library Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// --- Video Buffer & Flags ---
const int FRAME_BUFFER_SIZE = 20480; // 20KB
// Statically allocated to ensure it fits in internal RAM on N8 (No PSRAM) board
uint8_t frame_buffer[FRAME_BUFFER_SIZE]; 

volatile bool hasNewFrame = false;
volatile uint32_t latestFrameLength = 0;

// --- State Management ---
enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
volatile bool pairedFlag = false;
uint8_t carMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- Telemetry Data ---
float currentDist = 0.0;
bool lineFollowerActive = false;

// --- Timers ---
unsigned long lastDiscoveryTime = 0;
unsigned long lastJoystickTime = 0;

// === JPEG Drawing Callback ===
int JPEGDraw(JPEGDRAW *pDraw) {
    tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    return 1;
}

// === Video Callback (SIGNAL ONLY) ===
void onVideoFrame(uint32_t length) {
    if (currentState != CONNECTED || hasNewFrame) return;
    if (length > FRAME_BUFFER_SIZE) return;
    latestFrameLength = length;
    hasNewFrame = true;
}

// === ESP-NOW Data Callback (SIGNAL ONLY) ===
// FIXED: Reverted to the traditional signature as required by your framework
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && strncmp((const char*)data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        pairedFlag = true;
    }
    else if (currentState == CONNECTED && len > 3 && data[0] == 'D' && data[1] == ':') {
        char msg[64];
        int cpyLen = (len < 63) ? len : 63;
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
    delay(2000); // Important for S3 stability on boot
    Serial.begin(115200);

    const int buttons[] = {BTN_START, BTN_BACK, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_X, BTN_Y, BTN_A, BTN_B};
    for (int pin : buttons) pinMode(pin, INPUT_PULLUP);

    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Booting...", 10, 10, 2);

    jpeg.setMaxOutputSize(1); // Crucial for No-PSRAM boards

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    // Register the callback
    esp_now_register_recv_cb(onDataRecv);

    radio.setRecvBuffer(frame_buffer);
    radio.setRecvCallback(onVideoFrame);
    radio.init(160); // Match 160x120 resolution

    tft.drawString("Searching for Car...", 10, 110, 2);
}

void loop() {
    unsigned long now = millis();

    if (pairedFlag) {
        pairedFlag = false;
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, carMac, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        if (!esp_now_is_peer_exist(carMac)) {
            esp_now_add_peer(&peerInfo);
        }
        currentState = CONNECTED;
        tft.fillScreen(TFT_BLACK);
        Serial.println("Paired with Car!");
    }

    if (hasNewFrame) {
        hasNewFrame = false;
        if (currentState == CONNECTED) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();

                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.drawString("Dist: " + String(currentDist, 1) + "cm", 5, 220, 2);
            }
        }
    }

    if (currentState == SEARCHING) {
        if (now - lastDiscoveryTime > 500) {
            esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
            lastDiscoveryTime = now;
        }
    }
    else if (currentState == CONNECTED) {
        if (now - lastJoystickTime > 50) {
            uint8_t payload[6] = {
                67,
                getAnalog(ADC_JOY_LX),
                (uint8_t)(255 - getAnalog(ADC_JOY_LY)),
                getAnalog(ADC_JOY_RX),
                (uint8_t)(255 - getAnalog(ADC_JOY_RY)),
                getDPadAndButtons()
            };
            esp_now_send(carMac, payload, sizeof(payload));
            lastJoystickTime = now;
        }
    }
}
