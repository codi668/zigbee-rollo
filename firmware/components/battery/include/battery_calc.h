/**
 * @file battery_calc.h
 * @brief Akku-/NTC-Rechnung (reine Logik, ohne ESP-IDF, Host-getestet).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Zustand des NTC-Messkreises */
typedef enum {
    BATT_NTC_OK = 0,
    BATT_NTC_OPEN,      /**< Spannung > NTC_OPEN_MV: NTC fehlt / Leitung offen */
    BATT_NTC_SHORT,     /**< Spannung < NTC_SHORT_MV: Kurzschluss gegen GND */
} batt_ntc_state_t;

/** @brief Akkuspannung aus der Pin-Spannung (Teiler BATT_DIVIDER_NUM/DEN). */
uint32_t batt_calc_vbat_mv(uint32_t pin_mv);

/**
 * @brief Ladezustand aus der Ruhespannung nach BATT_CURVE_POINTS (linear interpoliert).
 * @return 0…200 in 0,5-%-Schritten (wie Zigbee BatteryPercentageRemaining)
 */
uint8_t batt_calc_percent_x2(uint32_t vbat_mv);

/** @brief Ladezustand 0…100 % (gerundet aus batt_calc_percent_x2). */
uint8_t batt_calc_percent(uint32_t vbat_mv);

/**
 * @brief NTC-Temperatur über die Beta-Formel (10k gegen VREF, NTC gegen GND).
 * @param pin_mv  Spannung am Mittelpunkt
 * @param temp_c  Ausgabe in °C (nur bei BATT_NTC_OK gültig)
 */
batt_ntc_state_t batt_calc_ntc(uint32_t pin_mv, float *temp_c);

/**
 * @brief Unterspannungssperre mit Hysterese.
 * Setzt bei vbat < BATT_LOCKOUT_MV, löst erst bei vbat >= BATT_LOCKOUT_RELEASE_MV
 * und wenn nicht geladen wird (beim Laden ist die Spannung überhöht).
 * @param locked  bisheriger Zustand
 * @return neuer Zustand (true = gesperrt)
 */
bool batt_calc_lockout(bool locked, uint32_t vbat_mv, bool charging);
