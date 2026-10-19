#pragma once
#include <stdbool.h>

#define MOTOR_FWD   1
#define MOTOR_BACK  (-1)

typedef enum {
    MOTOR_RUN_OK = 0,   /* Zeit abgelaufen */
    MOTOR_RUN_ABORT,    /* Callback hat gestoppt */
    MOTOR_RUN_FAULT,    /* nFAULT LOW, sofort gestoppt */
    MOTOR_RUN_ERROR,    /* ungueltige Parameter / Init fehlt */
} motor_result_t;

/* Wird waehrend der Fahrt etwa alle 5 ms aufgerufen; true = sofort stoppen. */
typedef bool (*motor_poll_cb_t)(int elapsed_ms, void *ctx);

/* LEDC (20 kHz) fuer beide Motoren, Ausgaenge LOW. */
void motor_init(void);
/* Faehrt Motor id (1 oder 2) in Richtung dir (MOTOR_FWD/MOTOR_BACK) mit duty_pct (auf
 * MOTOR_DUTY_MAX begrenzt) hoechstens max_ms (auf MOTOR_RUN_MAX_MS begrenzt).
 * Ueberwacht nFAULT. Stoppt am Ende immer: beide IN-Pins LOW, nSLEEP LOW. */
motor_result_t motor_run(int id, int dir, int duty_pct, int max_ms, motor_poll_cb_t cb, void *ctx);
/* Sofort-Stopp (Duty 0, nSLEEP LOW). */
void motor_stop(void);
/* Kurztext fuer ein Ergebnis */
const char *motor_result_str(motor_result_t r);
/* Befehl "motor": eine Fahrt ohne Encoder-Auswertung. PASS, wenn kein Fehler auftrat. */
bool motor_test(int id, int dir, int duty_pct, int ms);
