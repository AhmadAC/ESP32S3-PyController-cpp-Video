#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

struct GamepadState {
    // Upgraded to uint16_t to hold raw 0-4095 ADC values for the main loop to process
    uint16_t left_x, left_y;
    uint16_t right_x, right_y;
    bool up, down, left, right;
    bool y, x, b, a;
    bool start, back;
    bool left_stick_push, right_stick_push;
};

class Gamepad {
public:
    Gamepad();
    void init();
    GamepadState read();

private:
    adc_oneshot_unit_handle_t adc_handle_;
};

#endif