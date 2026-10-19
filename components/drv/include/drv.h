#pragma once
#include <stdbool.h>

/* nSLEEP als Ausgang (LOW = Treiber aus), nFAULT als Eingang. */
void drv_init(void);
/* Treiber ein-/ausschalten (nSLEEP). Beim Einschalten wird die Aufwachzeit abgewartet. */
void drv_enable(bool on);
/* true = nFAULT ist LOW (Fehler) */
bool drv_fault(void);
/* Test 8: nSLEEP HIGH, kurz warten, nFAULT muss HIGH sein, nSLEEP wieder LOW. Kein Motorlauf. */
bool drv_test(void);
