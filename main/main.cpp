#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lcd.h"

extern "C" void app_main(void) {
    ESP_LOGI("MAIN", "Initializing LCD...");
    LCD lcd;
    lcd.init();
    ESP_LOGI("MAIN", "LCD Initialized.");

    // --- Verification Test ---
    lcd.fill_screen(COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI("MAIN", "Running color test...");
    lcd.fill_screen(COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    lcd.fill_screen(COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    lcd.fill_screen(COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    lcd.fill_screen(COLOR_BLACK);
    
    ESP_LOGI("MAIN", "Drawing test pixels...");
    // Draw white border
    for(int i = 0; i < 240; i++) {
        lcd.draw_pixel(i, 0, COLOR_WHITE);
        lcd.draw_pixel(i, 239, COLOR_WHITE);
        lcd.draw_pixel(0, i, COLOR_WHITE);
        lcd.draw_pixel(239, i, COLOR_WHITE);
    }

    // Draw diagonal cross
    for(int i = 0; i < 240; i++) {
        lcd.draw_pixel(i, i, COLOR_YELLOW);
        lcd.draw_pixel(239-i, i, COLOR_CYAN);
    }
    
    ESP_LOGI("MAIN", "Test complete. LCD should be working.");
    
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}