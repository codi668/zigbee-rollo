/**
 * @file motion.h
 * @brief Fahrprofil einer Achse als Zustandsautomat (reine Logik, ohne ESP-IDF).
 *
 * motor_ctrl ruft motion_tick() alle MOTOR_TICK_MS mit Zeit und Encoder-Position
 * auf und setzt das Ergebnis (Antreiben/Bremsen/Aus) am DRV8833 um. Die ganze
 * Logik ist damit auf dem Host testbar.
 *
 * Ablauf einer Fahrt:
 *   [PAUSE]  nur vor einem Richtungswechsel: dir_pause_ms stehen
 *   KICK     kick_duty, bis kick_steps gezählt sind; zählt der Encoder in kick_ms
 *            nicht -> BLOCKED (Motor läuft nicht an / steht am Anschlag)
 *   RAMP     linear min_duty -> run_duty in ramp_up_ms (Blockiererkennung noch aus)
 *   CRUISE   run_duty, Blockiererkennung scharf
 *   (Bremszone, nur mit Ziel: Restweg < brake_zone_steps -> Duty linear bis min_duty)
 *   BRAKE    Kurzschlussbremse brake_ms, dann IDLE und Ergebnis
 *   STOPPING Richtungswechsel während der Fahrt: Duty in ramp_down_ms auf min_duty,
 *            dann BRAKE, PAUSE, neue Fahrt
 *
 * Abbruchbedingungen in KICK/RAMP/CRUISE: Timeout, Richtungsfehler (Encoder zählt
 * wrong_dir_steps gegen die Sollrichtung), Ziel/Endpunkt erreicht oder überfahren.
 *
 * Pro Fahrauftrag kommt genau ein Ergebnis (motion_out_t.result != NONE).
 * Wird ein Auftrag durch einen neuen in Gegenrichtung ersetzt, meldet nur der neue.
 * Alle Duty-Werte in Promille (0…1000).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "block_detect.h"

/** Parameter je Motor (aus config.h, siehe motor_ctrl.c) */
typedef struct {
    uint16_t min_duty;          /**< ‰, kleinster Duty, bei dem der Motor weiterläuft */
    uint16_t kick_duty;         /**< ‰, Anlauf-Kick */
    uint16_t run_duty;          /**< ‰, Fahr-Duty */
    uint32_t kick_ms;           /**< max. Kick-Dauer */
    int32_t  kick_steps;        /**< Schritte, die den Kick beenden */
    uint32_t ramp_up_ms;        /**< Rampe min -> run */
    uint32_t ramp_down_ms;      /**< Rampe -> min vor Richtungswechsel */
    int32_t  brake_zone_steps;  /**< Bremszone vor dem Ziel */
    int32_t  stop_tol_steps;    /**< Zieltoleranz */
    uint32_t brake_ms;          /**< Kurzschlussbremse am Ende */
    uint32_t dir_pause_ms;      /**< Pause vor Richtungswechsel */
    uint32_t block_window_ms;   /**< Blockiererkennung T */
    int32_t  block_min_steps;   /**< Blockiererkennung N */
    int32_t  wrong_dir_steps;   /**< Richtungsfehler-Schwelle */
    uint32_t timeout_max_ms;    /**< Obergrenze für jeden Timeout */
} motion_params_t;

typedef enum {
    MOTION_IDLE = 0,
    MOTION_PAUSE,
    MOTION_KICK,
    MOTION_RAMP,
    MOTION_CRUISE,
    MOTION_STOPPING,
    MOTION_BRAKE,
    MOTION_FINISH,      /**< Ergebnis wird im nächsten Tick gemeldet (ohne Fahrt) */
} motion_phase_t;

/** Ergebnis eines Fahrauftrags */
typedef enum {
    MOTION_RES_NONE = 0,
    MOTION_RES_DONE,        /**< Ziel erreicht (innerhalb Toleranz) */
    MOTION_RES_LIMIT,       /**< Endlosfahrt hat einen Endpunkt erreicht */
    MOTION_RES_STOPPED,     /**< durch motion_stop() angehalten */
    MOTION_RES_BLOCKED,     /**< Blockiert (oder Anlauf ohne Encoder-Impulse) */
    MOTION_RES_TIMEOUT,     /**< maximale Fahrzeit überschritten */
    MOTION_RES_WRONG_DIR,   /**< Encoder zählt gegen die Sollrichtung (Vorzeichen in config.h falsch?) */
} motion_result_t;

/** Was der Treiber in diesem Takt tun soll */
typedef enum {
    MOTION_OUT_OFF = 0,     /**< Freilauf (beide IN LOW) */
    MOTION_OUT_DRIVE,       /**< antreiben mit dir/duty */
    MOTION_OUT_BRAKE,       /**< Kurzschlussbremse */
} motion_drive_t;

typedef struct {
    motion_drive_t  drive;
    int8_t          dir;      /**< +1 = Position steigt, -1 = fällt */
    uint16_t        duty;     /**< ‰, nur bei DRIVE */
    motion_result_t result;   /**< != NONE genau einmal am Ende eines Auftrags */
    int32_t         goal;     /**< nur bei result != NONE: Ziel des BEENDETEN Auftrags (ohne Ziel: 0) */
    uint32_t        elapsed_ms; /**< nur bei result != NONE: Fahrzeit des beendeten Auftrags */
} motion_out_t;

/** Fahrauftrag (intern) */
typedef struct {
    bool            has_goal;
    int32_t         goal;
    int8_t          dir;            /**< nur ohne Ziel */
    uint32_t        timeout_ms;
    motion_result_t reach_result;   /**< DONE bei Zielfahrt, LIMIT bei Endlosfahrt bis Endpunkt */
} motion_job_t;

/** Zustand einer Achse – Felder nur über die Funktionen ändern */
typedef struct {
    motion_params_t p;
    motion_phase_t  phase;
    uint32_t        phase_t0;
    uint32_t        pause_end;
    motion_job_t    job;
    motion_job_t    next;
    bool            next_valid;
    int8_t          dir;
    uint32_t        job_t0;
    int32_t         start_pos;
    uint16_t        duty;
    uint16_t        stop_from;
    motion_result_t result_pending;
    bool            limits_on;
    int32_t         lo, hi;
    int8_t          last_dir;
    uint32_t        idle_t0;
    bool            ever_moved;
    block_det_t     blk;
} motion_t;

/** @brief Achse initialisieren (IDLE, keine Grenzen). Parameter werden plausibilisiert. */
void motion_init(motion_t *m, const motion_params_t *p);

/** @brief Harte Grenzen setzen (kalibrierte Achse) oder abschalten. */
void motion_set_limits(motion_t *m, bool enabled, int32_t lo, int32_t hi);

/**
 * @brief Zielfahrt. Ziel wird bei aktiven Grenzen auf [lo, hi] begrenzt.
 * Läuft schon eine Fahrt in dieselbe Richtung, wird nur das Ziel aktualisiert;
 * in Gegenrichtung wird abgebremst, pausiert und neu angefahren.
 * @param timeout_ms 0 oder > timeout_max_ms -> timeout_max_ms
 */
void motion_goto(motion_t *m, uint32_t now_ms, int32_t pos, int32_t goal, uint32_t timeout_ms);

/**
 * @brief Fahrt ohne Ziel in Richtung dir (±1) bis Blockieren/Stopp/Timeout
 * (unkalibriert, Kalibrierung). Bei aktiven Grenzen: bis zum Endpunkt (Ergebnis LIMIT).
 */
void motion_run(motion_t *m, uint32_t now_ms, int32_t pos, int8_t dir, uint32_t timeout_ms);

/**
 * @brief Anhalten (Kurzschlussbremse, Ergebnis STOPPED). Verwirft wartende Aufträge.
 * @return true, wenn dadurch ein Ergebnis folgt (Achse war aktiv)
 */
bool motion_stop(motion_t *m, uint32_t now_ms);

/** @brief Sofort IDLE ohne Bremsen und ohne Ergebnis (nFAULT: Treiber ist schon aus). */
void motion_abort(motion_t *m, uint32_t now_ms);

/** @brief Einen Regeltakt rechnen. */
motion_out_t motion_tick(motion_t *m, uint32_t now_ms, int32_t pos);

/** @brief true, solange die Achse nicht IDLE ist. */
bool motion_is_active(const motion_t *m);

/** @brief Zeit seit Start des aktuellen Auftrags (ms). */
uint32_t motion_job_elapsed(const motion_t *m, uint32_t now_ms);

/** @brief Kurztext für Logs. */
const char *motion_result_str(motion_result_t r);
