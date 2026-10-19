#include <stdio.h>
#include "drv.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WAKE_MS 5   /* DRV8833 tWAKE ca. 1 ms, hier grosszuegig */

void drv_init(void)
{
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PIN_DRV_NSLEEP,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(PIN_DRV_NSLEEP, 0);

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_DRV_NFAULT,
        .mode = GPIO_MODE_INPUT, /* externer 10k Pull-up */
    };
    gpio_config(&in);
}

void drv_enable(bool on)
{
    gpio_set_level(PIN_DRV_NSLEEP, on ? 1 : 0);
    if (on) vTaskDelay(pdMS_TO_TICKS(WAKE_MS));
}

bool drv_fault(void)
{
    return gpio_get_level(PIN_DRV_NFAULT) == 0;
}

bool drv_test(void)
{
    printf("DRV8833: nFAULT (GPIO%d) bei nSLEEP=LOW: %d (Pull-up, sollte 1 sein)\n",
           PIN_DRV_NFAULT, !drv_fault());
    drv_enable(true);
    int level = gpio_get_level(PIN_DRV_NFAULT);
    drv_enable(false);
    bool ok = level == 1;
    printf("DRV8833: nSLEEP (GPIO%d) HIGH, nFAULT = %d -> %s\n", PIN_DRV_NSLEEP, level,
           ok ? "PASS (kein Fehler)"
              : "FAIL (nFAULT LOW: Ueberstrom/Unterspannung/Kurzschluss, oder Leitung/Pull-up defekt)");
    return ok;
}
