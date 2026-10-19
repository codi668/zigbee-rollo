/**
 * @file window_sensor.h
 * @brief Fenstersensor (Schicht 3): Kipp-Erkennung über den LIS2DH12.
 *
 * Ohne Sensor (Rev 0.1: nicht bestückt) ist das Modul dauerhaft deaktiviert:
 * window_sensor_available() = false, Zustand immer WINDOW_UNKNOWN, kein Log
 * außer der einmaligen Meldung bei init.
 *
 * Die Referenzlage gehört der Applikation: sie lädt sie aus dem NVS
 * (storage_load(), Schlüssel win_ref) und übergibt sie mit
 * window_sensor_set_reference(); beim Einlernen liefert window_sensor_calibrate()
 * die neue Lage, die die Applikation speichert. So bleibt window_sensor
 * unabhängig von storage.
 *
 * Aufruf aus der sensor_task etwa jede Sekunde (SENSOR_WINDOW_PERIOD_MS).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lis2dh12_conv.h"
#include "tilt_logic.h"

/** @brief lis2dh12 initialisieren. Fehlender Sensor ist kein Fehler (liefert ESP_OK, deaktiviert). */
esp_err_t window_sensor_init(void);

/** @brief true, wenn ein Sensor gefunden wurde. */
bool window_sensor_available(void);

/** @brief true, wenn Sensor vorhanden UND Referenzlage gesetzt ist. */
bool window_sensor_ready(void);

/** @brief Referenzlage "Fenster zu" setzen (aus NVS). */
void window_sensor_set_reference(const lis2dh12_mg_t *ref);

/**
 * @brief Aktuelle Lage als "Fenster zu" einlernen (WIN_REF_SAMPLES Messungen bei 10 Hz
 * gemittelt, ~1 s blockierend) und als Referenz übernehmen.
 * @param ref_out gemittelte Lage zum Speichern im NVS
 * @return ESP_ERR_NOT_FOUND ohne Sensor
 */
esp_err_t window_sensor_calibrate(lis2dh12_mg_t *ref_out);

/**
 * @brief Eine Messung auswerten.
 * @param motor_busy true während einer Motorfahrt (dann und WIN_MOTION_HOLDOFF_MS danach keine Auswertung)
 * @param changed    optional: true, wenn sich der Zustand geändert hat
 * @return aktueller Zustand
 */
window_state_t window_sensor_poll(bool motor_busy, bool *changed);

/** @brief Letzter Zustand ohne neue Messung. */
window_state_t window_sensor_get_state(void);

/** @brief Letzter gültiger Winkel zur Referenz in Grad (< 0 = keiner). */
float window_sensor_last_angle(void);

/** @brief Kurztext. */
const char *window_state_str(window_state_t s);
