#pragma once
#include <stdbool.h>

void led_init(void);
void led_set(bool on);
/* Test 1: LED blinkt 5x (aktiv LOW). Sichtpruefung, gibt immer true zurueck. */
bool led_test(void);
