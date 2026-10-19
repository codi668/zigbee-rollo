#pragma once
#include <stdbool.h>

/* Initialisiert ADC1 (VBAT_ADC GPIO1, NTC_ADC GPIO4) mit Kalibrierung. Gibt false bei Fehler. */
bool adc_meas_init(void);
/* Mittelwert aus 16 kalibrierten Messungen in mV am Pin (-1 bei Fehler). */
int adc_meas_vbat_pin_mv(void);
int adc_meas_ntc_pin_mv(void);

/* Test 3: Akkuspannung, PASS bei 3,0 ... 4,25 V */
bool vbat_test(void);
/* Test 4: NTC-Widerstand/Temperatur, PASS bei 5 ... 45 Grad C */
bool ntc_test(void);
