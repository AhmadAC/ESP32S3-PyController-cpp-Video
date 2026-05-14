#include "gamepad.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include <cstdlib>

#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12

// Pin Mapping
#define PIN_LEFT_X ADC_CHANNEL_3  // GPIO 4
#define PIN_LEFT_Y ADC_CHANNEL_4  // GPIO 5
#define PIN_RIGHT_X ADC_CHANNEL_6 // GPIO 7
#define PIN_RIGHT_Y ADC_CHANNEL_7 // GPIO 8

#define PIN_L_PUSH GPIO_NUM_6
#define PIN_R_PUSH GPIO_NUM_9
#define PIN_UP     GPIO_NUM_10
#define PIN_DOWN   GPIO_NUM_11
#define PIN_LEFT   GPIO_NUM_12
#define PIN_RIGHT  GPIO_NUM_13
#define PIN_Y      GPIO_NUM_15
#define PIN_X      GPIO_NUM_14
#define PIN_B      GPIO_NUM_21
#define PIN_A      GPIO_NUM_16
#define PIN_START  GPIO_NUM_0
#define PIN_BACK   GPIO_NUM_1

Gamepad::Gamepad() : adc_handle_(nullptr) {}

void Gamepad::init() {
    const gpio_num_t buttons[] = {
        PIN_L_PUSH, PIN_R_PUSH, PIN_UP, PIN_DOWN, PIN_LEFT, PIN_RIGHT,
        PIN_Y, PIN_X, PIN_B, PIN_A, PIN_START, PIN_BACK
    };

    for(auto pin : buttons) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    }

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT;
    init_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT; 
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

    adc_oneshot_chan_cfg_t config = {};
    config.atten = ADC_ATTEN;
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, PIN_LEFT_X, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, PIN_LEFT_Y, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, PIN_RIGHT_X, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, PIN_RIGHT_Y, &config));
}

GamepadState Gamepad::read() {
    GamepadState s = {};
    s.up = !gpio_get_level(PIN_UP);
    s.down = !gpio_get_level(PIN_DOWN);
    s.left = !gpio_get_level(PIN_LEFT);
    s.right = !gpio_get_level(PIN_RIGHT);
    s.y = !gpio_get_level(PIN_Y);
    s.x = !gpio_get_level(PIN_X);
    s.b = !gpio_get_level(PIN_B);
    s.a = !gpio_get_level(PIN_A);
    s.start = !gpio_get_level(PIN_START);
    s.back = !gpio_get_level(PIN_BACK);
    s.left_stick_push = !gpio_get_level(PIN_L_PUSH);
    s.right_stick_push = !gpio_get_level(PIN_R_PUSH);

    int lx = 0, ly = 0, rx = 0, ry = 0;
    
    // 10x Oversampling drastically reduces jitter and physical drift.
    // Explicitly initializes fallback vars at neutral 2048 to prevent rogue memory pointers
    for (int i = 0; i < 10; i++) {
        int val_lx = 2048, val_ly = 2048, val_rx = 2048, val_ry = 2048;
        adc_oneshot_read(adc_handle_, PIN_LEFT_X, &val_lx);
        adc_oneshot_read(adc_handle_, PIN_LEFT_Y, &val_ly);
        adc_oneshot_read(adc_handle_, PIN_RIGHT_X, &val_rx);
        adc_oneshot_read(adc_handle_, PIN_RIGHT_Y, &val_ry);
        
        lx += val_lx;
        ly += val_ly;
        rx += val_rx;
        ry += val_ry;
    }

    s.left_x = lx / 10;
    s.left_y = ly / 10;
    s.right_x = rx / 10;
    s.right_y = ry / 10;

    return s;
}