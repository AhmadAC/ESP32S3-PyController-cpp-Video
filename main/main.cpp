#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lcd.h"
#include "gamepad.hpp"

extern "C" void app_main(void) {
    LCD lcd;
    lcd.init();
    
    Gamepad gamepad;
    gamepad.init();

    ESP_LOGI("MAIN", "PyController C++ Hardware Initialized.");

    while (1) {
        GamepadState state = gamepad.read();

        if (state.a) lcd.fill_screen(COLOR_GREEN);
        else if (state.b) lcd.fill_screen(COLOR_RED);
        else if (state.x) lcd.fill_screen(COLOR_BLUE);
        else if (state.y) lcd.fill_screen(COLOR_YELLOW);

        // Simple joystick cursor
        static int16_t lx = 120, ly = 120;
        lcd.draw_pixel(lx, ly, COLOR_BLACK); 
        lx = 120 + state.left_x;
        ly = 120 + state.left_y;
        lcd.draw_pixel(lx, ly, COLOR_WHITE);

        vTaskDelay(pdMS_TO_TICKS(16));
    }
}