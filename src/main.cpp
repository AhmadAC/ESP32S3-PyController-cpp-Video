#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// --- Gamepad Pinout (From your mpconfigboard.h) ---
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

// --- Analog Stick Pinout ---
#define PIN_JOY_LX 4 // ADC1_CH3
#define PIN_JOY_LY 5 // ADC1_CH4
#define PIN_JOY_RX 7 // ADC1_CH6
#define PIN_JOY_RY 8 // ADC1_CH7

TFT_eSPI tft = TFT_eSPI();
JPEGDEC jpeg;

WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// Connection State
enum State { SEARCHING, CONNECTED };
State currentState = SEARCHING;
uint8_t carMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Telemetry State
float currentDist = 0.0;
bool lineFollowerActive = false;

// Timers
unsigned long lastDiscoveryTime = 0;
unsigned long lastJoystickTime = 0;

// --- JPEG Drawing Callback ---
// Called by JPEGDEC when a block of pixels is ready to draw
int JPEGDraw(JPEGDRAW *pDraw) {
    tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    return 1;
}

// --- Video Frame Received Callback ---
// Called by ESPNowCam when a complete MJPEG chunk is fully assembled
void onVideoFrame(uint8_t *buffer, size_t length) {
    if (currentState != CONNECTED) return;
    
    // Decode and draw the frame directly to the TFT
    if (jpeg.openRAM(buffer, length, JPEGDraw)) {
        jpeg.setPixelType(RGB565_BIG_ENDIAN); // Match TFT_eSPI format
        jpeg.decode(0, 0, 0);                 // Draw starting at X=0, Y=0
        jpeg.close();

        // Overlay Telemetry Text on top of the video
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("Dist: " + String(currentDist) + " cm", 5, 215, 2);
        
        if (lineFollowerActive) {
            tft.fillCircle(220, 20, 10, TFT_BLACK);
        } else {
            tft.drawCircle(220, 20, 10, TFT_WHITE);
        }
    }
}

// --- ESP-NOW Telemetry & Handshake Callback ---
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len >= 9 && strncmp((const char*)data, "pyCAR_ACK", 9) == 0) {
        if (currentState == SEARCHING) {
            memcpy(carMac, mac, 6);
            
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, carMac, 6);
            peerInfo.channel = 1;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
            
            currentState = CONNECTED;
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawString("Connected!", 60, 110, 4);
            Serial.println("Paired to Car!");
            delay(1000); // Briefly show connected screen
        }
    } 
    else if (data[0] == 'D' && data[1] == ':') {
        // Parse "D:xx.xx,L:x,X:0" Telemetry string
        char msg[64];
        int cpyLen = len < 63 ? len : 63;
        memcpy(msg, data, cpyLen);
        msg[cpyLen] = '\0';
        
        char* d_ptr = strstr(msg, "D:");
        char* l_ptr = strstr(msg, "L:");
        if(d_ptr) currentDist = atof(d_ptr + 2);
        if(l_ptr) lineFollowerActive = (l_ptr[2] == '1');
    }
}

// --- Gamepad Mapping Logic ---
uint8_t getDPad() {
    bool up = !digitalRead(BTN_UP);
    bool down = !digitalRead(BTN_DOWN);
    bool left = !digitalRead(BTN_LEFT);
    bool right = !digitalRead(BTN_RIGHT);

    if (up && right) return 1;
    if (right && !down && !up) return 2;
    if (down && right) return 3;
    if (down && !left && !right) return 4;
    if (down && left) return 5;
    if (left && !up && !down) return 6;
    if (up && left) return 7;
    if (up && !left && !right) return 0;
    return 8; // None pressed
}

uint8_t getButtons() {
    uint8_t btns = getDPad();
    if (!digitalRead(BTN_Y)) btns |= (1 << 4);
    if (!digitalRead(BTN_B)) btns |= (1 << 5);
    if (!digitalRead(BTN_A)) btns |= (1 << 6);
    if (!digitalRead(BTN_X)) btns |= (1 << 7);
    return btns;
}

uint8_t getAnalog(int pin) {
    int val = analogRead(pin);
    int mapped = val / 16; // Map 12-bit (4095) down to 8-bit (255)
    return (mapped > 255) ? 255 : mapped;
}

void setup() {
    Serial.begin(115200);

    // Setup Buttons
    const int buttons[] = {BTN_START, BTN_BACK, BTN_A_OK, BTN_B_OK, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_X, BTN_Y, BTN_A, BTN_B};
    for (int pin : buttons) {
        pinMode(pin, INPUT_PULLUP);
    }
    
    // Setup ADC
    analogReadResolution(12);

    // Setup Screen
    tft.init();
    tft.setRotation(0); 
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString("Booting C++...", 60, 110, 4);

    // Setup Network
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    // Register Broadcast peer for discovery
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_register_recv_cb(onDataRecv);

    // Start Raw Wi-Fi Video Receiver
    radio.init(512); 
    radio.setReceiveCallback(onVideoFrame);

    tft.fillScreen(TFT_WHITE);
    tft.drawString("Searching for pyCar...", 30, 110, 4);
}

void loop() {
    unsigned long now = millis();

    if (currentState == SEARCHING) {
        // Send Discovery Ping every 250ms
        if (now - lastDiscoveryTime > 250) {
            const char* msg = "pyCAR_DISCOVER";
            esp_now_send(broadcastMac, (uint8_t*)msg, strlen(msg));
            lastDiscoveryTime = now;
        }
    } 
    else if (currentState == CONNECTED) {
        // Send Joystick Control Packets every 50ms (20Hz)
        if (now - lastJoystickTime > 50) {
            uint8_t lx = getAnalog(PIN_JOY_LX);
            uint8_t ly = getAnalog(PIN_JOY_LY);
            uint8_t rx = getAnalog(PIN_JOY_RX);
            uint8_t ry = getAnalog(PIN_JOY_RY);
            uint8_t btns = getButtons();

            // Structure matches your Python: struct.pack('<BBBBBB', 67, lx, ly, rx, ry, btns)
            uint8_t payload[6] = {67, lx, ly, rx, ry, btns};
            esp_now_send(carMac, payload, 6);
            
            lastJoystickTime = now;
        }
    }
}
