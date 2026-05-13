#include "lcd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <cstring>

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
    uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    t.tx_buffer = bytes;
    spi_device_polling_transmit(spi_handle_, &t);
}

void LCD::init() {
    gpio_reset_pin((gpio_num_t)PIN_NUM_DC);
    gpio_set_direction((gpio_num_t)PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)PIN_NUM_RST);
    gpio_set_direction((gpio_num_t)PIN_NUM_RST, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = DISP_WIDTH * DISP_HEIGHT * 2;
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 40 * 1000 * 1000;
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 7;
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle_));

    reset();

    send_cmd(0x11); 
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t d;
    d = 0x05; send_cmd(0x3A); send_data(&d, 1);
    d = 0xC0; send_cmd(0x36); send_data(&d, 1); // HW Orientation Fix

    uint8_t porch[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    send_cmd(0xB2); send_data(porch, 5);

    d = 0x35; send_cmd(0xB7); send_data(&d, 1);
    d = 0x32; send_cmd(0xBB); send_data(&d, 1);
    d = 0x01; send_cmd(0xC2); send_data(&d, 1);
    d = 0x15; send_cmd(0xC3); send_data(&d, 1);
    d = 0x20; send_cmd(0xC4); send_data(&d, 1);
    d = 0x0F; send_cmd(0xC6); send_data(&d, 1);

    uint8_t pwr[] = {0xA4, 0xA1};
    send_cmd(0xD0); send_data(pwr, 2);

    uint8_t gamma[] = {0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34};
    send_cmd(0xE0); send_data(gamma, 14);
    send_cmd(0xE1); send_data(gamma, 14);

    send_cmd(0x21); 
    send_cmd(0x29); 
    vTaskDelay(pdMS_TO_TICKS(120));

    fill_screen(COLOR_BLACK);
}

void LCD::set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t dx[] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
    send_cmd(0x2A); send_data(dx, 4);

    uint8_t dy[] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
    send_cmd(0x2B); send_data(dy, 4);

    send_cmd(0x2C); 
}

void LCD::fill_screen(uint16_t color) {
    set_address_window(0, 0, DISP_WIDTH - 1, DISP_HEIGHT - 1);
    uint16_t swap = (color << 8) | (color >> 8);
    uint16_t* line = (uint16_t*)heap_caps_malloc(DISP_WIDTH * 2, MALLOC_CAP_DMA);
    for(int i = 0; i < DISP_WIDTH; i++) line[i] = swap;

    gpio_set_level((gpio_num_t)PIN_NUM_DC, 1);
    for(int i = 0; i < DISP_HEIGHT; i++) {
        spi_transaction_t t = {};
        t.length = DISP_WIDTH * 16;
        t.tx_buffer = line;
        spi_device_polling_transmit(spi_handle_, &t);
    }
    free(line);
}

void LCD::draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= DISP_WIDTH || y >= DISP_HEIGHT) return;
    set_address_window(x, y, x, y);
    send_u16(color);
}