/**
 * @file battery.h
 * @brief Akku (Schicht 3 + ADC-Treiber): Spannung, Prozent, NTC-Temperatur,
 * Ladestatus (CHRG_N), Unterspannungssperre.
 *
 * ADC1 Oneshot mit Kalibrierung (curve fitting, 12 dB), BATT_ADC_SAMPLES-fach
 * gemittelt. Messen nur, wenn kein Motor läuft (Spannungseinbruch) – das prüft
 * der Aufrufer (motor_ctrl_any_busy()). Beim Laden ist die Spannung überhöht und
 * schwankt (gemessen 4,19–4,35 V); dann ist "charging" gesetzt und der
 * Prozentwert nur bedingt aussagekräftig.
 *
 * Nicht thread-sicher: battery_measure() nur aus einer Task (sensor_task) aufrufen.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "battery_calc.h"

/** Messergebnis */
typedef struct {
    uint16_t         vbat_mv;      /**< Akkuspannung */
    uint8_t          percent;      /**< 0…100 % */
    uint8_t          percent_x2;   /**< 0…200 (Zigbee BatteryPercentageRemaining) */
    bool             charging;     /**< CHRG_N LOW = TP4056 lädt */
    bool             lockout;      /**< Unterspannungssperre aktiv -> keine Fahrten */
    batt_ntc_state_t ntc;          /**< Zustand NTC-Kreis */
    float            temp_c;       /**< Temperatur in °C, nur bei ntc == BATT_NTC_OK gültig */
} battery_status_t;

/** @brief ADC (Akku GPIO1, NTC GPIO4) und CHRG_N (GPIO5, interner Pull-up) einrichten. */
esp_err_t battery_init(void);

/**
 * @brief Akku und NTC messen (blockiert ~2×16 ms), Sperre aktualisieren.
 * @return ESP_ERR_INVALID_STATE ohne Init, ADC-Fehlercodes sonst
 */
esp_err_t battery_measure(battery_status_t *out);

/** @brief Ladestatus direkt vom Pin (schnell, ohne ADC). */
bool battery_is_charging(void);

/**
 * @brief Sperrzustand der letzten Messung (true = keine Fahrten).
 * Vor der ersten Messung false -> die Applikation muss beim Start einmal
 * battery_measure() aufrufen, bevor sie Fahrbefehle zulässt.
 */
bool battery_lockout_active(void);
