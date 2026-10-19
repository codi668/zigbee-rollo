/**
 * @file btn_logic.h
 * @brief Entprellung und Klassifizierung der BOOT-Taste (reine Logik, Host-getestet).
 *
 * Mit dem Rohpegel ("gedrückt") alle BTN_POLL_MS aufrufen, solange
 * btn_logic_busy() true ist; dazwischen reicht ein Aufruf bei jeder Flanke.
 *
 * - Entprellen: ein Pegel gilt erst, wenn er BTN_DEBOUNCE_MS stabil anliegt.
 * - kurz  (< BTN_LONG_MS)                   -> BTN_EV_SHORT beim Loslassen
 * - Erreichen von BTN_LONG_MS (noch gedrückt) -> BTN_EV_LONG_HINT (nur Hinweis)
 * - lang  (BTN_LONG_MS … < BTN_VERY_LONG_MS) -> BTN_EV_LONG beim Loslassen
 * - sehr lang (>= BTN_VERY_LONG_MS)         -> BTN_EV_VERY_LONG sofort, das
 *   anschließende Loslassen erzeugt nichts mehr
 * - Ist die Taste bei btn_logic_init() schon gedrückt, wird dieser Druck
 *   ignoriert (erst nach dem Loslassen scharf).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BTN_EV_NONE = 0,
    BTN_EV_SHORT,
    BTN_EV_LONG_HINT,
    BTN_EV_LONG,
    BTN_EV_VERY_LONG,
} btn_event_t;

typedef struct {
    bool     raw;           /**< letzter Rohpegel */
    uint32_t raw_t;         /**< Zeitpunkt der letzten Rohänderung */
    bool     stable;        /**< entprellter Zustand */
    bool     armed;         /**< false bis zum ersten Loslassen nach init */
    uint32_t press_t;       /**< Zeitpunkt des entprellten Drückens */
    bool     hint_sent;
    bool     very_long_sent;
} btn_logic_t;

/** @brief Zustand initialisieren. @param pressed Rohpegel beim Start */
void btn_logic_init(btn_logic_t *b, uint32_t now_ms, bool pressed);

/** @brief Neuen Rohpegel verarbeiten. @return höchstens ein Ereignis pro Aufruf */
btn_event_t btn_logic_update(btn_logic_t *b, uint32_t now_ms, bool pressed);

/** @brief true, solange gepollt werden muss (gedrückt oder Pegel noch nicht entprellt). */
bool btn_logic_busy(const btn_logic_t *b);
