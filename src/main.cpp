#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- pyController Hardware Pins ---
#define BTN_UP      10
#define BTN_DOWN    11
#define BTN_LEFT    12
#define BTN_RIGHT   13
#define BTN_A       16
#define BTN_B       21
#define BTN_X       14
#define BTN_Y       15
#define BTN_START   0
#define BTN_BACK    1

#define POT_LX      4
#define POT_LY      5
#define POT_RX      7
#define POT_RY      8

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

// ESP-NOW Data Callback (ESP32 Arduino Core v2.x signature)
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        currentState = CONNECTED;
        Serial.println("Paired with Car!");
    }
}

// Encode gamepad buttons into a single byte matching original pyController protocol
uint8_t encodeButtons() {
    bool up = !digitalRead(BTN_UP);
    bool down = !digitalRead(BTN_DOWN);
    bool left = !digitalRead(BTN_LEFT);
    bool right = !digitalRead(BTN_RIGHT);

    uint8_t dpad = 8; // Idle
    if (up && right) dpad = 1;
    else if (right && down) dpad = 3;
    else if (down && left) dpad = 5;
    else if (left && up) dpad = 7;
    else if (up) dpad = 0;
    else if (right) dpad = 2;
    else if (down) dpad = 4;
    else if (left) dpad = 6;

    uint8_t btns = dpad;
    
    // Action Buttons
    if (!digitalRead(BTN_A)) btns |= 0x10; // A toggles line follower
    if (!digitalRead(BTN_B)) btns |= 0x20; // B toggles headlights
    if (!digitalRead(BTN_X)) btns |= 0x40;
    if (!digitalRead(BTN_Y)) btns |= 0x80;

    return btns;
}

void setup() {
    Serial.begin(115200);

    // Initialize Input Pins
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);
    pinMode(BTN_X, INPUT_PULLUP);
    pinMode(BTN_Y, INPUT_PULLUP);
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);

    // Set analog resolution to match expected 0-4095 range for mapping
    analogReadResolution(12);

    Serial.println("\n[1/4] Starting Display...");
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // Use safe built-in font drawing
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("pyController S3");
    tft.println("Starting...");

    Serial.println("[2/4] Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Force ESP-NOW communication channel
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
    
    // Init video at 160px width mode
    radio.init(160); 

    Serial.println("BOOT COMPLETE");
    tft.fillScreen(TFT_BLACK);
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastSearch = 0;
    static unsigned long lastGamepad = 0;

    // 1. Process Received Video Frames
    if (hasNewFrame) {
        // Double check JPEG Start of Image (SOI) marker to prevent crash
        if (latestFrameLength > 4 && frame_buffer[0] == 0xFF && frame_buffer[1] == 0xD8) {
            if (jpeg.openRAM(frame_buffer, latestFrameLength, JPEGDraw)) {
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();
            }
        }
        hasNewFrame = false;
    }

    // 2. State-based Logic (Searching vs Connected)
    if (currentState == SEARCHING) {
        if (now - lastSearch > 1500) {
            esp_now_send(broadcastMac, (uint8_t*)"pyCAR_DISCOVER", 14);
            lastSearch = now;
            
            tft.setCursor(10, 110);
            tft.print("Searching for Car... ");
        }
    } else {
        // 3. Send Gamepad Data (~20Hz)
        if (now - lastGamepad > 50) {
            uint8_t lx = map(analogRead(POT_LX), 0, 4095, 0, 255);
            uint8_t ly = map(analogRead(POT_LY), 0, 4095, 0, 255);
            uint8_t rx = map(analogRead(POT_RX), 0, 4095, 0, 255);
            uint8_t ry = map(analogRead(POT_RY), 0, 4095, 0, 255);
            
            uint8_t buttons = encodeButtons();
            
            // Magic packet header 'C' (67)
            uint8_t pkt[6] = { 67, lx, ly, rx, ry, buttons };
            esp_now_send(carMac, pkt, 6);
            
            lastGamepad = now;
        }
    }
}
