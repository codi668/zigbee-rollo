#pragma once
#include <stdbool.h>

void charger_init(void);
/* true = TP4056 laedt (CHRG_N LOW) */
bool charger_is_charging(void);
/* Test 5: Ladestatus lesen; watch_s > 0: so lange Aenderungen live anzeigen. */
bool charger_test(int watch_s);
