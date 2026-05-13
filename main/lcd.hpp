#pragma once

#include <stdint.h>
#include <string>

#define WHITE 0xFFFF
#define BLACK 0x0000
#define GREEN 0x07E0
#define RED   0xF800

class LCD {
public:
    void init();
    void fill(uint16_t color);
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t size = 2);
    void printStr(const std::string& str, uint16_t x, uint16_t y, uint16_t color, uint16_t bg_color, uint8_t size = 2);
    void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

private:
    void sendCmd(uint8_t cmd);
    void sendData(const uint8_t* data, int len);
    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
};
