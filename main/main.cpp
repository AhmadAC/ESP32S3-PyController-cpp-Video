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

    ESP_LOGI("MAIN", "Controller Ready.");

    while (1) {
        GamepadState state = gamepad.read();

        // Simple Visual Feedback: Fill screen based on buttons
        if (state.a) lcd.fill_screen(COLOR_GREEN);
        else if (state.b) lcd.fill_screen(COLOR_RED);
        else if (state.x) lcd.fill_screen(COLOR_BLUE);
        else if (state.y) lcd.fill_screen(COLOR_YELLOW);

        // Optional: Draw pixels based on joystick position
        // This clears and draws a point at the stick location
        static int16_t last_lx = 120, last_ly = 120;
        lcd.draw_pixel(last_lx, last_ly, COLOR_BLACK);
        int16_t cur_x = 120 + state.left_x;
        int16_t cur_y = 120 + state.left_y;
        lcd.draw_pixel(cur_x, cur_y, COLOR_WHITE);
        last_lx = cur_x; last_ly = cur_y;

        vTaskDelay(pdMS_TO_TICKS(16)); // ~60fps refresh
    }
}