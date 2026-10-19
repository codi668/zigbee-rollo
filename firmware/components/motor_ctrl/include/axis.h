/**
 * @file axis.h
 * @brief Achsen-Datenstruktur und Umrechnungen (reine Logik, ohne ESP-IDF).
 *
 * Konvention: end_top = 0 nach der Kalibrierung (Anschlag oben, Blockieren),
 * end_bottom = eingelernter Zählerstand unten. Positive Zählschritte führen
 * also von oben nach unten. 0 % = oben = offen (wie Zigbee Lift: 0 % = ganz offen),
 * 100 % = unten = geschlossen.
 *
 * Damit normale Fahrten nicht gegen einen Anschlag laufen, wird 0…100 % auf den
 * nutzbaren Bereich [end_top + margin_top, end_bottom - margin_bottom] abgebildet.
 * Die harten Grenzen (motor_ctrl stoppt dort sofort) sind end_top/end_bottom.
 *
 * Funktioniert auch mit end_bottom < end_top (vertauschte Zählrichtung).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Je eine Achse: Heben, Wenden (Spezifikation, um Sicherheitsabstände erweitert). */
typedef struct {
    int32_t pos;            /**< aktueller Zählerstand (Encoder) */
    int32_t end_top;        /**< Endpunkt oben, nach Kalibrierung = 0 */
    int32_t end_bottom;     /**< Endpunkt unten (eingelernt) */
    bool    calibrated;     /**< Endpunkte gültig */
    int32_t margin_top;     /**< Sicherheitsabstand oben in Schritten (>= 0) */
    int32_t margin_bottom;  /**< Sicherheitsabstand unten in Schritten (>= 0) */
} axis_t;

/**
 * @brief Endpunkte setzen, Ränder aus Promille des Fahrwegs berechnen, calibrated = true.
 * Ränder werden so begrenzt, dass der nutzbare Bereich nie negativ wird.
 */
void axis_set_endpoints(axis_t *a, int32_t end_top, int32_t end_bottom,
                        uint16_t margin_top_permille, uint16_t margin_bottom_permille);

/** @brief Kalibrierung verwerfen (Endpunkte/Ränder 0, calibrated = false). Position bleibt. */
void axis_clear(axis_t *a);

/** @brief Fahrweg |end_bottom - end_top| in Schritten (0 wenn unkalibriert). */
int32_t axis_travel(const axis_t *a);

/** @brief Kleinere / größere harte Grenze (min/max von end_top, end_bottom). */
int32_t axis_limit_lo(const axis_t *a);
int32_t axis_limit_hi(const axis_t *a);

/**
 * @brief Ziel auf die harten Grenzen begrenzen ("Befehl darüber hinaus wird abgeschnitten").
 * Unkalibriert: Ziel unverändert.
 */
int32_t axis_clamp(const axis_t *a, int32_t goal);

/**
 * @brief Prozent (0…100, größere Werte = 100) in Zählschritte im nutzbaren Bereich.
 * Gerundet. Unkalibriert: liefert pos (keine Fahrt).
 */
int32_t axis_percent_to_steps(const axis_t *a, uint8_t percent);

/**
 * @brief Zählerstand in Prozent (0…100, gerundet, begrenzt).
 * Unkalibriert oder Fahrweg 0: liefert 0.
 */
uint8_t axis_steps_to_percent(const axis_t *a, int32_t pos);

/** @brief true, wenn pos auf oder jenseits einer harten Grenze liegt (in Fahrtrichtung dir = ±1). */
bool axis_at_limit(const axis_t *a, int32_t pos, int dir);
