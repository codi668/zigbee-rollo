/**
 * @file ui.h
 * @brief Bedienung (Schicht 3): BOOT-Taste (GPIO9) als Ereignisse, LED-Muster (GPIO8).
 *
 * Eigene ui_task (UI_TASK_PRIO): wird von einem GPIO-Interrupt (beide Flanken)
 * geweckt, pollt nur, solange die Taste gedrückt/unentprellt ist (BTN_POLL_MS),
 * und taktet die LED nur, solange ein Blinkmuster aktiv ist (LED_TICK_MS).
 * Im Ruhezustand (Taste los, LED aus) schläft die Task ohne Timeout.
 *
 * Tastenereignisse (Details siehe btn_logic.h):
 * | Ereignis            | wann                                  | Bedeutung laut Spezifikation |
 * |---------------------|---------------------------------------|------------------------------|
 * | UI_EVT_BTN_SHORT     | Loslassen nach < 3 s                 | Stopp bzw. Auf/Ab im Wechsel; Kalibriermodus: Endpunkt setzen / Tippbetrieb |
 * | UI_EVT_BTN_LONG_HINT | 3 s erreicht, Taste noch gedrückt    | nur Rückmeldung (z. B. LED kurz an) |
 * | UI_EVT_BTN_LONG      | Loslassen nach 3 … < 10 s            | Kalibrierung starten / untere Endlage speichern |
 * | UI_EVT_BTN_VERY_LONG | sofort bei 10 s                       | Werkseinstellung (Netz verlassen, NVS löschen) |
 *
 * Die Bedeutung entscheidet die Applikation (app_state); ui kennt keinen Zustand.
 * Der Callback läuft im Kontext der ui_task: nicht blockieren, nur z. B.
 * xQueueSend(app_queue, ..., 0).
 *
 * TODO Teil 2 (Light Sleep): Flanken-Interrupts wecken nicht aus dem Light
 * Sleep; dafür zusätzlich gpio_wakeup_enable(PIN_BOOT_BTN, GPIO_INTR_LOW_LEVEL)
 * + esp_sleep_enable_gpio_wakeup() einrichten und im Test prüfen.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "led_pattern.h"

/** Tastenereignisse */
typedef enum {
    UI_EVT_BTN_SHORT = 0,
    UI_EVT_BTN_LONG_HINT,
    UI_EVT_BTN_LONG,
    UI_EVT_BTN_VERY_LONG,
} ui_event_t;

/** Ereignis-Callback (Kontext: ui_task, nicht blockieren!) */
typedef void (*ui_event_cb_t)(ui_event_t evt, void *ctx);

/**
 * @brief GPIO8 (LED, aus) und GPIO9 (Taste) einrichten, ui_task starten.
 * @param cb  Ereignis-Callback oder NULL
 */
esp_err_t ui_init(ui_event_cb_t cb, void *ctx);

/**
 * @brief LED-Muster setzen (aus jeder Task, nicht aus ISR). Das Muster beginnt
 * neu, wenn es sich ändert; gleiches Muster erneut setzen ändert nichts.
 * Zuordnung laut Spezifikation: Pairing = LED_PAT_PAIRING, Kalibrierung =
 * LED_PAT_CALIBRATION, Fehler = LED_PAT_ERROR, Normalbetrieb = LED_PAT_OFF.
 */
void ui_set_led(led_pattern_t p);

/** @brief Aktuelles LED-Muster. */
led_pattern_t ui_get_led(void);

/** @brief Entprellter Tastenzustand (true = gedrückt). */
bool ui_button_pressed(void);

/** @brief Kurztext eines Ereignisses (Logs). */
const char *ui_event_str(ui_event_t e);
