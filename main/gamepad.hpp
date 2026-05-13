#pragma once

#include <stdint.h>

struct GamepadState {
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t buttons;
};

class Gamepad {
public:
    void init();
    GamepadState read();
};
