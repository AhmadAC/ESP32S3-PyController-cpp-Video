#include "lcd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

// Pin definitions from your mpconfigboard.h
#define PIN_NUM_MOSI 41
#define PIN_NUM_CLK  40
#define PIN_NUM_CS   39
#define PIN_NUM_DC   38
#define PIN_NUM_RST  42

LCD::LCD() : spi_handle_(nullptr) {}

void LCD::reset() {
    gpio_set_level((gpio_num_t)PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level((gpio_num_t)PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void LCD::send_cmd(uint8_t cmd) {
    gpio_set_level((gpio_num_t)PIN_NUM_DC, 0);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spi_handle_, &t);
}

void LCD::send_data(const uint8_t* data, int len) {
    if (len == 0) return;
    gpio_set_level((gpio_num_t)PIN_NUM_DC, 1);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi_handle_, &t);
}

void LCD::send_u16(uint16_t data) {
    gpio_set_level((gpio_num_t)PIN_NUM_DC, 1);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    // SPI is big-endian, so swap bytes
    uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    t.tx_buffer = bytes;
    spi_device_polling_transmit(spi_handle_, &t);
}


void LCD::init() {
    // 1. Configure GPIO pins
    gpio_reset_pin((gpio_num_t)PIN_NUM_DC);
    gpio_set_direction((gpio_num_t)PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)PIN_NUM_RST);
    gpio_set_direction((gpio_num_t)PIN_NUM_RST, GPIO_MODE_OUTPUT);

    // 2. Configure SPI Bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_WIDTH * DISP_HEIGHT * 2,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 40 * 1000 * 1000, // 40 MHz
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle_);

    // 3. Reset and send initialization commands
    reset();

    send_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x3A); // Interface Pixel Format
    send_data(new uint8_t[1]{0x05}, 1); // 16-bit/pixel

    send_cmd(0x36); // Memory Access Control
    send_data(new uint8_t[1]{0x00}, 1); // Row/Col addr, Top to Bottom

    send_cmd(0xB2); // Porch Setting
    send_data(new uint8_t[5]{0x0C, 0x0C, 0x00, 0x33, 0x33}, 5);

    send_cmd(0xB7); // Gate Control
    send_data(new uint8_t[1]{0x35}, 1);

    send_cmd(0xBB); // VCOM Setting
    send_data(new uint8_t[1]{0x32}, 1); // Vcom=1.35V

    send_cmd(0xC2); // VDV and VRH Command Enable
    send_data(new uint8_t[1]{0x01}, 1);

    send_cmd(0xC3); // VRH Set
    send_data(new uint8_t[1]{0x15}, 1); // GVDD=4.8V

    send_cmd(0xC4); // VDV Set
    send_data(new uint8_t[1]{0x20}, 1); // VDV=0v

    send_cmd(0xC6); // Frame Rate Control in Normal Mode
    send_data(new uint8_t[1]{0x0F}, 1); // 60Hz

    send_cmd(0xD0); // Power Control 1
    send_data(new uint8_t[2]{0xA4, 0xA1}, 2);

    // Positive Voltage Gamma Control
    send_cmd(0xE0);
    send_data(new uint8_t[14]{0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34}, 14);

    // Negative Voltage Gamma Control
    send_cmd(0xE1);
    send_data(new uint8_t[14]{0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34}, 14);

    send_cmd(0x21); // Display Inversion On
    send_cmd(0x29); // Display On
    vTaskDelay(pdMS_TO_TICKS(120));

    fill_screen(COLOR_BLACK);
}

void LCD::set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    // Apply the crucial offsets
    x1 += X_OFFSET;
    x2 += X_OFFSET;
    y1 += Y_OFFSET;
    y2 += Y_OFFSET;

    send_cmd(0x2A); // Column Address Set
    send_data(new uint8_t[4]{(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2}, 4);

    send_cmd(0x2B); // Row Address Set
    send_data(new uint8_t[4]{(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2}, 4);

    send_cmd(0x2C); // Memory Write
}

void LCD::fill_screen(uint16_t color) {
    set_address_window(0, 0, DISP_WIDTH - 1, DISP_HEIGHT - 1);
    
    const int buffer_size = 128; // Send in chunks
    uint8_t color_buffer[buffer_size * 2];
    for(int i = 0; i < buffer_size; ++i) {
        color_buffer[i*2] = color >> 8;
        color_buffer[i*2 + 1] = color & 0xFF;
    }
    
    int total_pixels = DISP_WIDTH * DISP_HEIGHT;
    int chunks = total_pixels / buffer_size;
    
    gpio_set_level((gpio_num_t)PIN_NUM_DC, 1);
    for(int i=0; i < chunks; ++i) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = buffer_size * 16;
        t.tx_buffer = color_buffer;
        spi_device_polling_transmit(spi_handle_, &t);
    }
    
    int remaining = total_pixels % buffer_size;
    if (remaining > 0) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = remaining * 16;
        t.tx_buffer = color_buffer;
        spi_device_polling_transmit(spi_handle_, &t);
    }
}

void LCD::draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= DISP_WIDTH || y >= DISP_HEIGHT) return;
    set_address_window(x, y, x, y);
    send_u16(color);
}