#ifndef LCD_H
#define LCD_H

#include "driver/spi_master.h"
#include <stdint.h>

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0

class LCD {
public:
    LCD();
    void init();
    void fill_screen(uint16_t color);
    void draw_pixel(int16_t x, int16_t y, uint16_t color);
    
    // Text rendering functions
    void draw_char(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t scale = 1);
    void draw_string(int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg_color, uint8_t scale = 1);

    // Advanced block & JPG rendering
    void draw_bitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data);
    void draw_jpg(const char* filename, int x, int y);

private:
    void send_cmd(uint8_t cmd);
    void send_data(const uint8_t* data, int len);
    void set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void reset();

    spi_device_handle_t spi_handle_;
    
    static const int DISP_WIDTH = 240;
    static const int DISP_HEIGHT = 240;

    static const int X_OFFSET = 0;
    static const int Y_OFFSET = 0;
};

#endif