/**
 * @file tilt_logic.h
 * @brief Kipp-Erkennung mit Referenzlage, Hysterese und Entprellung (reine Logik, Host-getestet).
 *
 * gekippt:  Winkel zur Referenz > WIN_TILT_ON_DEG  (WIN_DEBOUNCE_COUNT Messungen in Folge)
 * zu:       Winkel zur Referenz < WIN_TILT_OFF_DEG (WIN_DEBOUNCE_COUNT Messungen in Folge)
 * dazwischen bleibt der Zustand. Messungen mit Betrag < WIN_MIN_MAG_MG werden verworfen.
 * Während einer Motorfahrt und WIN_MOTION_HOLDOFF_MS danach wird nicht ausgewertet.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "lis2dh12_conv.h"

/** Fensterzustand */
typedef enum {
    WINDOW_UNKNOWN = 0,   /**< kein Sensor, keine Referenz oder noch keine gültige Auswertung */
    WINDOW_CLOSED,        /**< zu (Lage = Referenz) */
    WINDOW_TILTED,        /**< gekippt */
} window_state_t;

typedef struct {
    window_state_t state;
    window_state_t cand;      /**< Kandidat für den Zustandswechsel */
    uint8_t        cnt;       /**< Messungen in Folge für cand */
    bool           has_ref;
    lis2dh12_mg_t  ref;
    bool           motion_seen;
    uint32_t       last_motion_ms;
    float          last_angle;  /**< letzter gültiger Winkel (Diagnose) */
} tilt_det_t;

/** @brief Winkel zwischen zwei Vektoren in Grad; < 0 wenn ein Betrag < WIN_MIN_MAG_MG. */
float tilt_angle_deg(const lis2dh12_mg_t *a, const lis2dh12_mg_t *b);

/** @brief Zustand UNKNOWN, ohne Referenz. */
void tilt_det_init(tilt_det_t *t);

/** @brief Referenzlage "Fenster zu" setzen; Zustand wird neu bestimmt (UNKNOWN bis zur Entprellung). */
void tilt_det_set_ref(tilt_det_t *t, const lis2dh12_mg_t *ref);

/** @brief Motorfahrt melden (startet/verlängert die Sperrzeit). */
void tilt_det_note_motion(tilt_det_t *t, uint32_t now_ms);

/** @brief true, solange nach einer Fahrt nicht ausgewertet werden darf. */
bool tilt_det_holdoff(const tilt_det_t *t, uint32_t now_ms);

/**
 * @brief Neue Messung auswerten (Aufrufer prüft vorher tilt_det_holdoff()).
 * @return true, wenn sich der Zustand geändert hat
 */
bool tilt_det_update(tilt_det_t *t, const lis2dh12_mg_t *v);
