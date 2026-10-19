#include <stdio.h>
#include "charger.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void charger_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_CHRG_N,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, /* CHRG_N ist Open-Drain */
    };
    gpio_config(&cfg);
}

bool charger_is_charging(void)
{
    return gpio_get_level(PIN_CHRG_N) == 0;
}

bool charger_test(int watch_s)
{
    bool c = charger_is_charging();
    printf("Laden: CHRG_N (GPIO%d) = %d -> %s\n", PIN_CHRG_N, !c,
           c ? "laedt" : "laedt nicht (kein USB, Akku voll oder kein Akku)");
    if (watch_s > 0) {
        printf("Laden: beobachte %d s, Aenderungen werden angezeigt (USB-Kabel am Ladeeingang ein-/ausstecken)\n", watch_s);
        int64_t t0 = esp_timer_get_time();
        int64_t end = t0 + (int64_t)watch_s * 1000000;
        bool last = c;
        while (esp_timer_get_time() < end) {
            bool now = charger_is_charging();
            if (now != last) {
                last = now;
                printf("  t=%5.1f s: %s\n", (esp_timer_get_time() - t0) / 1e6,
                       now ? "laedt" : "laedt nicht");
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    printf("Laden: PASS (Status gelesen; Erwartung haengt vom Aufbau ab)\n");
    return true;
}
