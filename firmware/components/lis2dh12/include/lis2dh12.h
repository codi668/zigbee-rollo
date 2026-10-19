/**
 * @file lis2dh12.h
 * @brief I²C-Treiber LIS2DH12 (Schicht 4): WHO_AM_I-Prüfung, Normal Mode 10 Bit, ±2 g.
 *
 * Auf Platine Rev 0.1 ist der Sensor NICHT bestückt. lis2dh12_init() meldet das
 * mit ESP_ERR_NOT_FOUND (einmal geloggt, kurzer I²C-Timeout), gibt den Bus wieder
 * frei und alle weiteren Aufrufe liefern sofort ESP_ERR_INVALID_STATE ohne Log.
 * Nicht thread-sicher: nur aus einer Task verwenden (sensor_task über window_sensor).
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lis2dh12_conv.h"

/** Messrate */
typedef enum {
    LIS2DH12_RATE_1HZ = 0,   /**< Normalbetrieb (Spezifikation) */
    LIS2DH12_RATE_10HZ,      /**< kurzzeitig zum Einlernen der Referenzlage */
} lis2dh12_rate_t;

/**
 * @brief Bus anlegen, Sensor suchen, WHO_AM_I prüfen, 1 Hz / ±2 g / 10 Bit konfigurieren.
 * @return ESP_OK, ESP_ERR_NOT_FOUND (keine Antwort / falscher WHO_AM_I) oder I²C-Fehler
 */
esp_err_t lis2dh12_init(void);

/** @brief true, wenn der Sensor bei init gefunden wurde. */
bool lis2dh12_present(void);

/** @brief Messrate umschalten. */
esp_err_t lis2dh12_set_rate(lis2dh12_rate_t rate);

/** @brief X/Y/Z in mg lesen (letzter Messwert). */
esp_err_t lis2dh12_read_mg(lis2dh12_mg_t *out);

/** @brief Sensor in Power-down schalten. */
esp_err_t lis2dh12_power_down(void);
