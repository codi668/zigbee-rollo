/**
 * @file drv8833.h
 * @brief Treiber DRV8833 (Schicht 4): LEDC-PWM 20 kHz/10 Bit, nSLEEP, nFAULT.
 *
 * - Kanal A = Motor 1 (Heben, GPIO10/11), Kanal B = Motor 2 (Wenden, GPIO12/22).
 * - Slow oder Fast Decay je nach MOTOR_SLOW_DECAY (config.h).
 * - nSLEEP ist nach drv8833_init() LOW (Treiber aus) und wird von motor_ctrl
 *   nur während einer Fahrt HIGH geschaltet.
 * - nFAULT: fallende Flanke löst einen Interrupt aus, solange der Treiber
 *   eingeschaltet ist. Die ISR schaltet sofort nSLEEP LOW (alle Ausgänge
 *   hochohmig, Motoren aus) und ruft dann den Fault-Callback auf.
 *
 * Nicht thread-sicher: alle Fahrfunktionen nur aus EINER Task aufrufen
 * (motor_task in motor_ctrl).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** Brückenkanal des DRV8833 */
typedef enum {
    DRV8833_CH_A = 0,   /**< AIN1/AIN2 – Motor 1 (Heben) */
    DRV8833_CH_B = 1,   /**< BIN1/BIN2 – Motor 2 (Wenden) */
    DRV8833_CH_COUNT
} drv8833_ch_t;

/**
 * @brief Fault-Callback, wird im ISR-KONTEXT aufgerufen.
 * Darf nur ISR-sichere Funktionen verwenden (xQueueSendFromISR o. ä.).
 * Zum Zeitpunkt des Aufrufs ist nSLEEP bereits LOW.
 */
typedef void (*drv8833_fault_cb_t)(void *ctx);

/**
 * @brief GPIOs, LEDC-Timer und 4 Kanäle einrichten; alle Ausgänge LOW, nSLEEP LOW.
 * @param cb  Fault-Callback (ISR-Kontext) oder NULL
 * @param ctx Zeiger, der an den Callback übergeben wird
 */
esp_err_t drv8833_init(drv8833_fault_cb_t cb, void *ctx);

/**
 * @brief nSLEEP setzen. Beim Einschalten wird DRV8833_WAKE_MS gewartet und
 * danach der nFAULT-Interrupt scharf geschaltet. Beim Ausschalten werden
 * vorher beide Kanäle auf Freilauf gesetzt.
 * @return ESP_ERR_INVALID_STATE, wenn nFAULT nach dem Aufwachen LOW ist
 *         (Treiber bleibt dann aus).
 */
esp_err_t drv8833_enable(bool on);

/** @brief true, wenn nSLEEP HIGH ist. */
bool drv8833_is_enabled(void);

/** @brief true in den ersten DRV8833_FAULT_BLANK_MS nach dem Einschalten (nFAULT-Puls beim Anlauf ignorieren). */
bool drv8833_in_blanking(void);

/** @brief true, wenn nFAULT gerade LOW ist (nur bei eingeschaltetem Treiber aussagekräftig). */
bool drv8833_fault_active(void);

/**
 * @brief Kanal antreiben.
 * @param ch           Kanal
 * @param forward      true = IN1 treibt (vorwärts), false = IN2 treibt
 * @param duty_permille effektiver Duty 0…1000 (auf MOTOR_DUTY_MAX_PCT begrenzt);
 *                      0 = Freilauf
 */
esp_err_t drv8833_drive(drv8833_ch_t ch, bool forward, uint16_t duty_permille);

/** @brief Kurzschlussbremse: beide IN-Pins HIGH (nur bei nSLEEP HIGH wirksam). */
esp_err_t drv8833_brake(drv8833_ch_t ch);

/** @brief Freilauf: beide IN-Pins LOW. */
esp_err_t drv8833_coast(drv8833_ch_t ch);

/**
 * @brief Sofort alles aus: beide Kanäle Freilauf, nSLEEP LOW, Interrupt aus.
 * Sicher aus jeder Task aufrufbar (nicht aus ISR).
 */
void drv8833_all_off(void);
