#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Gamepad Pinout (Verified from Schematic) ---
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
#define ADC_JOY_LX 4  // GPIO 4
#define ADC_JOY_LY 5  // GPIO 5
#define ADC_JOY_RX 7  // GPIO 7
#define ADC_JOY_RY 8  // GPIO 8

// --- Objects ---
TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// --- Buffer Settings (Optimized for No-PSRAM N8 Board) ---
#define FRAME_BUFFER_SIZE 24576 
uint8_t frame_buffer[FRAME_BUFFER_SIZE]; 

volatile bool hasNewFrame = false;
volatile uint32_t latestFrameLength = 0;

enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
volatile bool pairedFlag = false;
uint8_t carMac[6];
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

float currentDist = 0.0;
unsigned long lastDiscoveryTime = 0;
unsigned long lastJoystickTime = 0;

// JPEG Drawing Callback
int JPEGDraw(JPEGDRAW *pDraw) {
    if (pDraw->pPixels) {
        tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    }
    return 1;
}

// Video Callback
void onVideoFrame(uint32_t length) {
    if (currentState != CONNECTED || hasNewFrame) return;
    if (length > 0 && length <= FRAME_BUFFER_SIZE) {
        latestFrameLength = length;
        hasNewFrame = true;
    }
}

// Data Callback: Handles Telemetry and Pairing
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (!data || len <= 0) return;

    // 1. Handle Handshake
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        pairedFlag = true;
    }
    // 2. Handle Telemetry (Distance)
    else if (currentState == CONNECTED && len > 2 && data[0] == 'D' && data[1] == ':') {
        char msg[32];
        int cpy = (len < 31) ? len : 31;
        memcpy(msg, data, cpy);
        msg[cpy] = '\0';
        char* d_ptr = strstr(msg, "D:");
        if (d_ptr) currentDist = atof(d_ptr + 2);
    }
}

uint8_t getDPadAndButtons() {
    uint8_t key = 8; // Neutral
    bool u = !digitalRead(BTN_UP), d = !digitalRead(BTN_DOWN);
    bool l = !digitalRead(BTN_LEFT), r = !digitalRead(BTN_RIGHT);

    if (u) key = r ? 1 : (l ? 7 : 0);
    else if (d) key = r ? 3 : (l ? 5 : 4);
    else if (r) key = 2;
    else if (l) key = 6;

    if (!digitalRead(BTN_Y)) key |= (1 << 4);
    if (!digitalRead(BTN_B)) key |= (1 << 5);
    if (!digitalRead(BTN_A)) key |= (1 << 6);
    if (!digitalRead(BTN_X)) key |= (1 << 7);
    return key;
}

void setup() {
    // 1. Stability Delay
    delay(2000); 
    Serial.begin(115200);
    Serial.println("pyController S3 Booting...");

    // 2. Initialize Hardware Pins
    const int btns[] = {BTN_START, BTN_BACK, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_X, BTN_Y, BTN_A, BTN_B};
    for (int p : btns) pinMode(p, INPUT_PULLUP);

    // 3. Initialize Display (TFT_eSPI handles GPIO 38-42)
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("pyController S3", 10, 10, 2);
    tft.drawString("Searching for pyCar...", 10, 40, 2);

    // 4. Initialize WiFi & ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    // 5. Register Data Callback (Must be done BEFORE radio.init)
    esp_now_register_recv_cb(onDataRecv);

    // 6. Initialize Video Radio
    radio.setRecvBuffer(frame_buffer);
    radio.setRecvCallback(onVideoFrame);
    radio.init(160); // 160px width mode

    // 7. Add Broadcast Peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

void loop() {
    unsigned long now = millis();

    // Handle Pairing
    if (pairedFlag) {
        pairedFlag = false;
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, carMac, 6);
        peer.channel = 1;
        peer.encrypt = false;
        if (!esp_now_is_peer_exist(carMac)) esp_now_add_peer(&peer);
        currentState = CONNECTED;
        tft.fillScreen(TFT_BLACK);
        Serial.println("Paired!");
    }

    // Handle Video Frames
    if (hasNewFrame) {
        // Only decode if we are connected and have a JPEG Start-of-Image marker
        if (currentState == CONNECTED && latestFrameLength > 4 && frame_buffer[0] == 0xFF && frame_buffer[1] == 0xD8) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();

                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.drawString("Dist: " + String(currentDist, 1) + "cm", 5, 220, 2);
            }
        }
        hasNewFrame = false; 
    }

    // Handshake Task
    if (currentState == SEARCHING) {
        if (now - lastDiscoveryTime > 1000) {
            esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
            lastDiscoveryTime = now;
        }
    }
    // Control Task
    else if (currentState == CONNECTED) {
        if (now - lastJoystickTime > 50) {
            uint8_t payload[6] = {
                67, // 'C' for Control
                (uint8_t)(analogRead(ADC_JOY_LX)/16),
                (uint8_t)(255 - (analogRead(ADC_JOY_LY)/16)),
                (uint8_t)(analogRead(ADC_JOY_RX)/16),
                (uint8_t)(255 - (analogRead(ADC_JOY_RY)/16)),
                getDPadAndButtons()
            };
            esp_now_send(carMac, payload, 6);
            lastJoystickTime = now;
        }
    }
}