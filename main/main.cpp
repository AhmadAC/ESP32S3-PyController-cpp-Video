#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.hpp"
#include "gamepad.hpp"
#include "espnow_handler.hpp"
#include <cstdio>

LCD lcd;
Gamepad gamepad;
EspNowHandler espnow;

CarStatus latest_status = {0.0f, false, false};
volatile bool status_updated = false;

extern "C" void app_main() {
    lcd.init();
    lcd.fill(WHITE);
    lcd.printStr("Booting...", 10, 10, BLACK, WHITE, 2);

    gamepad.init();
    espnow.init();
    
    espnow.setStatusCallback([](const CarStatus& status) {
        latest_status = status;
        status_updated = true;
    });

    lcd.fill(WHITE);
    lcd.printStr("Searching for pyCar...", 10, 100, BLACK, WHITE, 2);

    while (!espnow.isConnected()) {
        espnow.sendDiscover();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lcd.fill(WHITE);
    lcd.printStr("Connected!", 10, 10, GREEN, WHITE, 2);
    lcd.printStr("Ultrasonic:", 10, 160, BLACK, WHITE, 2);

    uint32_t last_lcd_update = 0;
    uint32_t last_tx_time = 0;
    char dist_str[32] = "";

    while (1) {
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

        if (now - last_lcd_update >= 200) {
            if (status_updated) {
                status_updated = false;
                
                // Clear lingering texts securely without screen flickering
                snprintf(dist_str, sizeof(dist_str), "%.2f cm      ", latest_status.distance);
                lcd.printStr(dist_str, 10, 190, BLACK, WHITE, 2);

                if (latest_status.line_follower) {
                    lcd.fillCircle(220, 20, 10, BLACK);
                } else {
                    lcd.fillCircle(220, 20, 10, WHITE);
                    lcd.drawCircle(220, 20, 10, BLACK);
                }

                if (latest_status.sync_state) {
                    lcd.fillCircle(190, 20, 10, RED);
                } else {
                    lcd.fillCircle(190, 20, 10, WHITE);
                    lcd.drawCircle(190, 20, 10, RED);
                }
            }
            last_lcd_update = now;
        }

        if (now - last_tx_time >= 50) {
            GamepadState state = gamepad.read();
            espnow.sendGamepad(state.lx, state.ly, state.rx, state.ry, state.buttons);
            last_tx_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
