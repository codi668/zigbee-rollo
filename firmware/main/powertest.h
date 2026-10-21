/**
 * @file powertest.h
 * @brief Leistungstest-Variante (ROLLO_POWERTEST): feste Zustände nacheinander für die Strommessung am Akku.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t idx;        /**< Nummer 1…9 */
    bool    full;       /**< true: normale Initialisierung (Akku, Taste, Zigbee …), sonst nur Minimalbetrieb */
    bool    zigbee;     /**< Zigbee starten */
    bool    sleepy;     /**< Sleepy End Device (Funk schläft zwischen den Polls) */
    bool    pm;         /**< automatischer Light Sleep */
    bool    drv_awake;  /**< DRV8833 wach (nSLEEP HIGH), Motor steht */
    bool    motor;      /**< Motor 2 fährt hin und her */
    const char *name;
} pt_cfg_t;

/** @brief Zustand aus dem RTC-Speicher lesen, LED-Kennung blinken, Endezeit planen. Ganz am Anfang von app_main. */
void pt_begin(pt_cfg_t *cfg);

/** @brief Zustand ausführen, der nicht die normale Applikation braucht (Zustände 4–6, 9). Kehrt nicht zurück. */
void pt_run_minimal(const pt_cfg_t *cfg) __attribute__((noreturn));

/** @brief Nach der normalen Initialisierung (Zustände 1–3, 7, 8): Light Sleep, Treiber, Motor je nach Zustand. */
void pt_after_init(const pt_cfg_t *cfg);
