#include <stdio.h>
#include "button.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEBOUNCE_MS 30

void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, /* extern 10k vorhanden, intern schadet nicht */
    };
    gpio_config(&cfg);
}

bool button_test(int duration_s)
{
    printf("Taste: BOOT-Taste (GPIO%d) %d s lang druecken, Druecke werden gezaehlt\n",
           PIN_BOOT_BTN, duration_s);
    int presses = 0;
    int stable = gpio_get_level(PIN_BOOT_BTN);
    int last_raw = stable;
    int64_t last_change = esp_timer_get_time();
    int64_t end = last_change + (int64_t)duration_s * 1000000;

    while (esp_timer_get_time() < end) {
        int raw = gpio_get_level(PIN_BOOT_BTN);
        int64_t now = esp_timer_get_time();
        if (raw != last_raw) {
            last_raw = raw;
            last_change = now;
        } else if (raw != stable && (now - last_change) >= DEBOUNCE_MS * 1000) {
            stable = raw;
            if (stable == 0) {
                presses++;
                printf("  Druck #%d erkannt\n", presses);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    printf("Taste: %d Druecke gezaehlt -> %s\n", presses,
           presses > 0 ? "PASS" : "FAIL (kein Druck erkannt - nicht gedrueckt oder Taste/Pull-up defekt)");
    return presses > 0;
}
