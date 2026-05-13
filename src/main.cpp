#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- pyController Hardware Pins ---
#define LCD_BL      33
#define BTN_UP      10
#define BTN_DOWN    11
#define BTN_LEFT    12
#define BTN_RIGHT   13
#define BTN_A       16
#define BTN_B       21
#define POT_LX      4
#define POT_LY      5

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

// LEGACY SIGNATURE: Required for your current Arduino-ESP32 v2.x core
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (currentState == SEARCHING && len >= 9 && memcmp(data, "pyCAR_ACK", 9) == 0) {
        memcpy(carMac, mac, 6);
        currentState = CONNECTED;
        Serial.println("Paired with Car!");
    }
}

void setup() {
    Serial.begin(115200);
    
    // LCD Backlight setup
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    // Inputs setup
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);

    Serial.println("\n[1/4] Starting Display...");
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawCentreString("PYCONTROLLER S3", 120, 10, 2);
    tft.drawCentreString("Initializing WiFi...", 120, 40, 2);

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
    radio.init(160); // Init at 160px width for stability on No-PSRAM board

    Serial.println("BOOT COMPLETE");
    tft.fillScreen(TFT_BLACK);
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastSearch = 0;
    static unsigned long lastGamepad = 0;

    // 1. Process Received Video Frames
    if (hasNewFrame) {
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
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.drawCentreString("SEARCHING FOR CAR...", 120, 110, 2);
        }
    } else {
        // 3. Send Gamepad Data (~20Hz)
        if (now - lastGamepad > 50) {
            uint8_t lx = map(analogRead(POT_LX), 0, 4095, 0, 255);
            uint8_t ly = map(analogRead(POT_LY), 0, 4095, 0, 255);
            
            // Basic button bitmask (A=bit 4, B=bit 5)
            uint8_t buttons = 0;
            if (digitalRead(BTN_A) == LOW) buttons |= 0x10; 
            if (digitalRead(BTN_B) == LOW) buttons |= 0x20;
            
            // Standard controller packet
            uint8_t pkt[6] = { 67, lx, ly, 128, 128, buttons };
            esp_now_send(carMac, pkt, 6);
            lastGamepad = now;
        }
    }
}