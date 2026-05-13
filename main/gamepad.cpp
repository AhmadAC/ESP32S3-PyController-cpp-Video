#include "gamepad.hpp"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t adc1_handle;

void Gamepad::init() {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL<<10) | (1ULL<<11) | (1ULL<<12) | (1ULL<<13) |
                           (1ULL<<14) | (1ULL<<15) | (1ULL<<16) | (1ULL<<21) |
                           (1ULL<<1)  | (1ULL<<0)  | (1ULL<<6)  | (1ULL<<9);
    gpio_config(&io_conf);

    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.atten = ADC_ATTEN_DB_12; 
    config.bitwidth = ADC_BITWIDTH_DEFAULT;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config)); // IO4
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config)); // IO5
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config)); // IO7
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config)); // IO8
}

GamepadState Gamepad::read() {
    GamepadState state = {};
    int val = 0;
    
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &val); // LX
    state.lx = (val * 255) / 4095;
    
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &val); // LY
    state.ly = (val * 255) / 4095;
    
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &val); // RX
    state.rx = (val * 255) / 4095;
    
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &val); // RY
    state.ry = (val * 255) / 4095;

    bool up = !gpio_get_level((gpio_num_t)10);
    bool down = !gpio_get_level((gpio_num_t)11);
    bool left = !gpio_get_level((gpio_num_t)12);
    bool right = !gpio_get_level((gpio_num_t)13);

    uint8_t dpad = 8;
    if (up) {
        if (right) dpad = 1;
        else if (left) dpad = 7;
        else dpad = 0;
    } else if (down) {
        if (right) dpad = 3;
        else if (left) dpad = 5;
        else dpad = 4;
    } else if (right) {
        dpad = 2;
    } else if (left) {
        dpad = 6;
    }

    state.buttons = dpad;

    if (!gpio_get_level((gpio_num_t)16)) state.buttons |= (1<<6); // A
    if (!gpio_get_level((gpio_num_t)21)) state.buttons |= (1<<5); // B
    if (!gpio_get_level((gpio_num_t)14)) state.buttons |= (1<<7); // X
    if (!gpio_get_level((gpio_num_t)15)) state.buttons |= (1<<4); // Y

    return state;
}
