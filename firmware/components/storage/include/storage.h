/**
 * @file storage.h
 * @brief Speicher (Schicht 3): NVS-Wrapper für Kalibrierung, Positionen und Fenster-Referenz.
 *
 * NVS-Partition "nvs", Namespace STORAGE_NVS_NAMESPACE (config.h). Schlüssel:
 *
 * | Schlüssel  | Typ  | Bedeutung                                                  |
 * |------------|------|------------------------------------------------------------|
 * | lift_max   | i32  | Endpunkt unten Heben in Schritten (0 / fehlt = unkalibriert)|
 * | tilt_max   | i32  | Endpunkt unten Wenden (0 / fehlt = unkalibriert)            |
 * | lift_pos   | i32  | Position Heben am Ende der letzten Fahrt                    |
 * | tilt_pos   | i32  | Position Wenden am Ende der letzten Fahrt                   |
 * | moving     | u8   | 1 = eine Fahrt lief, Position unsicher                      |
 * | win_ref_x/y/z | i16 | Referenzlage "Fenster zu" in mg                         |
 *
 * Endpunkt oben ist per Konvention immer 0 (axis.h), deshalb wird nur "max"
 * gespeichert.
 *
 * Schreibregeln (Flash-Abnutzung, Spezifikation):
 * - storage_motion_begin() (moving = 1) VOR jeder Fahrt,
 *   storage_motion_end() (Position + moving = 0) am Ende – nie zyklisch
 *   während der Fahrt schreiben.
 * - Unveränderte Werte werden nicht erneut geschrieben (RAM-Kopie).
 * - Steht nach einem Neustart moving = 1, gilt die Position als unsicher
 *   -> Applikation referenziert neu (oben anfahren).
 *
 * Thread-sicher (Mutex). Alle Funktionen blockieren für die Dauer eines
 * NVS-Commits (einige ms) – nicht aus ISR, nicht aus Zigbee-Callbacks aufrufen.
 * Keine Abhängigkeit von anderen Projektmodulen.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** Achse für die Speicherfunktionen (gleiche Reihenfolge wie motor_id_t) */
typedef enum {
    STORAGE_AXIS_LIFT = 0,
    STORAGE_AXIS_TILT = 1,
    STORAGE_AXIS_COUNT
} storage_axis_t;

/** Beschleunigungsvektor in mg (gleiches Layout wie lis2dh12_mg_t, aber ohne Abhängigkeit) */
typedef struct {
    int16_t x, y, z;
} storage_vec3_t;

/** Gesamter gespeicherter Zustand */
typedef struct {
    int32_t        max[STORAGE_AXIS_COUNT];  /**< Endpunkt unten; 0 = nicht kalibriert */
    int32_t        pos[STORAGE_AXIS_COUNT];  /**< letzte Position (0, wenn nie gespeichert) */
    bool           moving;                   /**< true = Neustart während einer Fahrt */
    bool           has_win_ref;              /**< Referenzlage vorhanden */
    storage_vec3_t win_ref;                  /**< Referenzlage "Fenster zu" */
} storage_data_t;

/**
 * @brief NVS initialisieren (nvs_flash_init) und gespeicherten Zustand in den RAM laden.
 * Ist die Partition voll/alt (ESP_ERR_NVS_NO_FREE_PAGES / NEW_VERSION_FOUND),
 * wird sie gelöscht und neu angelegt (Warnung im Log; Werte sind dann weg).
 */
esp_err_t storage_init(void);

/** @brief Kopie des gespeicherten Zustands (aus der RAM-Kopie, kein Flash-Zugriff). */
esp_err_t storage_load(storage_data_t *out);

/** @brief true, wenn für die Achse ein Endpunkt unten != 0 gespeichert ist. */
bool storage_is_calibrated(storage_axis_t axis);

/**
 * @brief Endpunkt unten einer Achse speichern (nach der Kalibrierung).
 * @param max Endpunkt unten in Schritten, != 0 (0 = ESP_ERR_INVALID_ARG; zum Löschen
 *            storage_clear_calibration() verwenden)
 */
esp_err_t storage_save_calibration(storage_axis_t axis, int32_t max);

/** @brief Kalibrierung einer Achse löschen (max = 0). */
esp_err_t storage_clear_calibration(storage_axis_t axis);

/** @brief moving = 1 schreiben – vor dem Start jeder Fahrt aufrufen. */
esp_err_t storage_motion_begin(void);

/**
 * @brief Position einer Achse und moving = 0 schreiben – am Ende einer Fahrt.
 * Ein einziger Commit für beide Werte.
 */
esp_err_t storage_motion_end(storage_axis_t axis, int32_t pos);

/** @brief Nur die Position speichern (z. B. nach dem Referenzieren), moving bleibt. */
esp_err_t storage_save_position(storage_axis_t axis, int32_t pos);

/** @brief Referenzlage "Fenster zu" speichern. */
esp_err_t storage_save_win_ref(const storage_vec3_t *ref);

/**
 * @brief Werkseinstellung: alle Schlüssel dieses Namespace löschen.
 * Der Zigbee-Netzwerkzustand liegt in der Partition zb_storage und wird hier
 * NICHT gelöscht (das macht Teil 2 über esp_zb_factory_reset()).
 */
esp_err_t storage_erase_all(void);
