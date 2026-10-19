#include <stdio.h>
#include "led.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    led_set(false);
}

void led_set(bool on)
{
    gpio_set_level(PIN_LED, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

bool led_test(void)
{
    printf("LED: blinkt 5x (GPIO%d, aktiv LOW) - bitte beobachten\n", PIN_LED);
    for (int i = 0; i < 5; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(250));
        led_set(false);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    printf("LED: PASS (Ausgabe erfolgt; Sichtpruefung durch Bediener)\n");
    return true;
}
