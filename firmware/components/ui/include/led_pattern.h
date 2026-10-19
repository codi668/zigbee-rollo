/**
 * @file led_pattern.h
 * @brief LED-Blinkmuster als Funktion der Zeit (reine Logik, Host-getestet).
 *
 * Zeiten aus config.h (LED_*_MS). t_ms = Zeit seit Musterbeginn.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LED_PAT_OFF = 0,     /**< Normalbetrieb: aus */
    LED_PAT_PAIRING,     /**< schnelles Blinken (LED_FAST_ON/OFF_MS) */
    LED_PAT_CALIBRATION, /**< langsames Blinken (LED_SLOW_ON/OFF_MS) */
    LED_PAT_ERROR,       /**< Doppelblitz: an-aus-an, dann LED_ERR_PAUSE_MS aus */
    LED_PAT_ON,          /**< dauernd an (z. B. Rückmeldung "3 s erreicht") */
    LED_PAT_COUNT
} led_pattern_t;

/** @brief Soll die LED zum Zeitpunkt t_ms leuchten? */
bool led_pattern_level(led_pattern_t p, uint32_t t_ms);

/** @brief Periodendauer des Musters in ms (0 = statisch: OFF/ON). */
uint32_t led_pattern_period(led_pattern_t p);
