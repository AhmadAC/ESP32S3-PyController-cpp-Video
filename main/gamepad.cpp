#include "gamepad.hpp"
#include "driver/gpio.h"
#include <cstdlib>

#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12

// Verified hardware mapping from schematic + MicroPython config
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

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
    adc_oneshot_new_unit(&init_config, &adc_handle_);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc_handle_, PIN_LEFT_X, &config);
    adc_oneshot_config_channel(adc_handle_, PIN_LEFT_Y, &config);
    adc_oneshot_config_channel(adc_handle_, PIN_RIGHT_X, &config);
    adc_oneshot_config_channel(adc_handle_, PIN_RIGHT_Y, &config);
}

GamepadState Gamepad::read() {
    GamepadState s;
    // Low = Pressed (Pull-up)
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

    int lx, ly, rx, ry;
    adc_oneshot_read(adc_handle_, PIN_LEFT_X, &lx);
    adc_oneshot_read(adc_handle_, PIN_LEFT_Y, &ly);
    adc_oneshot_read(adc_handle_, PIN_RIGHT_X, &rx);
    adc_oneshot_read(adc_handle_, PIN_RIGHT_Y, &ry);

    auto map_axis = [](int val) -> int8_t {
        int res = (val - 2048) / 16;
        if (std::abs(res) < 15) return 0; // Deadzone
        return (res > 127) ? 127 : (res < -128 ? -128 : res);
    };

    s.left_x = map_axis(lx);
    s.left_y = -map_axis(ly); // Invert Y
    s.right_x = map_axis(rx);
    s.right_y = -map_axis(ry); // Invert Y

    return s;
}