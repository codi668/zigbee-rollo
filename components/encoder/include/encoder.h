#pragma once
#include <stdbool.h>

/* PCNT je Encoder: Quadratur x4, Glitch-Filter, interne Pull-ups an A/B. */
bool enc_init(void);
/* Zaehler auf 0 setzen / aktuellen Zaehlerstand (x4-Counts, vorzeichenbehaftet) lesen. */
void enc_clear(int id);
int enc_count(int id);
/* Counts pro Abtriebsumdrehung (x4), 0 = unbekannt (siehe ENC1_PPR_X1 in board_pins.h). */
int enc_counts_per_rev(int id);

/* Test 10a "enc <id> [duty]": Motor 1 s vor, 1 s zurueck, Counts/rpm anzeigen,
 * PASS wenn Zaehlrichtung mit der Drehrichtung umkehrt. Faehrt den Motor! */
bool enc_test(int id, int duty_pct);
/* Test 10b "enc <id> hand [s]": nur zaehlen, Motor aus. PASS wenn Counts auftreten. */
bool enc_hand_test(int id, int duration_s);
/* Test 11 "stall <id> [duty]": niedriger Duty, stoppt bei 200 ms ohne Encoder-Impulse. Faehrt den Motor! */
bool stall_test(int id, int duty_pct);
