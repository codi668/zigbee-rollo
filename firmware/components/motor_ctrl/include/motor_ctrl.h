/**
 * @file motor_ctrl.h
 * @brief Motorsteuerung (Schicht 3): Positionsfahrt beider Achsen in Encoder-Schritten.
 *
 * Besitzt die motor_task (10-ms-Takt, läuft nur während einer Fahrt, sonst blockiert
 * sie auf ihrer Befehlsqueue), den DRV8833 und die Encoder.
 *
 * Schnittstelle nach oben:
 * - Befehle: motor_ctrl_goto()/move_by()/run()/stop() legen nur einen Befehl in die
 *   interne Queue und kehren sofort zurück (aus jeder Task aufrufbar, nicht aus ISR).
 * - Ergebnisse: über den bei motor_ctrl_init() übergebenen Callback. Eine Achse ist
 *   "busy" (motor_ctrl_is_busy()) vom ersten Befehl bis zu GENAU EINEM Abschluss-
 *   Ereignis (DONE, LIMIT, STOPPED, BLOCKED, TIMEOUT, WRONG_DIR, FAULT, REJECTED).
 *   Wird ein laufender Auftrag durch einen neuen ersetzt (neues Ziel, Gegenrichtung),
 *   bekommt der ersetzte KEIN eigenes Ereignis – nur der letzte meldet. Beim
 *   Abschluss-Ereignis ist is_busy() bereits false. Dazu PROGRESS etwa jede Sekunde
 *   während der Fahrt (kein Abschluss).
 *   Der Callback läuft im Kontext der motor_task: nicht blockieren, nur z. B.
 *   xQueueSend(app_queue, ..., 0) aufrufen. Er darf keine motor_ctrl-Befehle
 *   synchron erwarten (Befehle aus dem Callback sind erlaubt, werden aber erst im
 *   nächsten Takt verarbeitet).
 *
 * Positionskonvention (siehe axis.h): end_top = 0 oben, positive Schritte nach unten.
 * MOTOR_DIR_DOWN = Position steigt, MOTOR_DIR_UP = Position fällt.
 *
 * Sicherheit: nFAULT (Interrupt + Abfrage je Takt), Timeout, Blockiererkennung,
 * Richtungsfehler und harte Endpunkte (bei kalibrierter Achse) stoppen jede Fahrt.
 * Nach dem Start sind die Motoren aus (nSLEEP LOW). Während einer Fahrt hält
 * motor_ctrl einen PM-Lock (kein Light Sleep) und ist beim Task-Watchdog angemeldet.
 * Die Unterspannungssperre prüft die Applikation (battery_lockout_active()) vor
 * jedem Fahrbefehl – motor_ctrl kennt den Akku nicht.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "axis.h"

/** Achse / Motor */
typedef enum {
    MOTOR_LIFT = 0,   /**< Motor 1, Heben (JGA25-370), Encoder 1 */
    MOTOR_TILT = 1,   /**< Motor 2, Wenden (N20), Encoder 2 */
    MOTOR_COUNT
} motor_id_t;

/** Fahrtrichtung für motor_ctrl_run() */
typedef enum {
    MOTOR_DIR_UP = -1,    /**< Richtung end_top (Position fällt), Lift: auf/offen */
    MOTOR_DIR_DOWN = 1,   /**< Richtung end_bottom (Position steigt), Lift: ab/zu */
} motor_dir_t;

/** Ereignistypen */
typedef enum {
    MOTOR_EVT_DONE = 0,   /**< Ziel erreicht (innerhalb Toleranz) */
    MOTOR_EVT_LIMIT,      /**< run() an einem Endpunkt angehalten (nur kalibriert) */
    MOTOR_EVT_STOPPED,    /**< durch motor_ctrl_stop() angehalten */
    MOTOR_EVT_BLOCKED,    /**< blockiert bzw. läuft nicht an (bei Kalibrierung = Anschlag) */
    MOTOR_EVT_TIMEOUT,    /**< maximale Fahrzeit überschritten */
    MOTOR_EVT_WRONG_DIR,  /**< Encoder zählt gegen die Sollrichtung -> Mx_DIR_INVERT/ENCx_INVERT prüfen */
    MOTOR_EVT_FAULT,      /**< nFAULT des DRV8833 (Überstrom/Übertemperatur/Unterspannung) oder Notaus;
                               setzt die Fehlersperre, siehe motor_ctrl_clear_fault() */
    MOTOR_EVT_REJECTED,   /**< Auftrag verworfen (andere Achse fährt, MOTOR_ALLOW_PARALLEL = 0) */
    MOTOR_EVT_PROGRESS,   /**< Zwischenstand während der Fahrt (kein Abschluss) */
} motor_event_type_t;

/** Ereignis an die Applikation */
typedef struct {
    motor_event_type_t type;
    motor_id_t         id;
    int32_t            pos;         /**< Position beim Ereignis (Schritte) */
    int32_t            goal;        /**< Ziel des Auftrags (bei run() ohne Grenzen: 0) */
    uint32_t           elapsed_ms;  /**< Fahrzeit des Auftrags bis hier */
} motor_event_t;

/** Ereignis-Callback (Kontext: motor_task, nicht blockieren!) */
typedef void (*motor_event_cb_t)(const motor_event_t *evt, void *ctx);

/**
 * @brief DRV8833 und Encoder initialisieren, motor_task starten. Motoren bleiben aus.
 * Beide Achsen starten unkalibriert mit Position 0.
 */
esp_err_t motor_ctrl_init(motor_event_cb_t cb, void *ctx);

/**
 * @brief Auf eine Position fahren. Bei kalibrierter Achse wird das Ziel auf
 * [end_top, end_bottom] begrenzt. Läuft schon eine Fahrt, wird das Ziel ersetzt
 * (Gegenrichtung: abbremsen, 100 ms Pause, neu anlaufen).
 * @param timeout_ms 0 = Standard des Motors (Mx_TIMEOUT_DEFAULT_MS), max. MOTOR_TIMEOUT_MAX_MS
 * @return ESP_ERR_INVALID_STATE, wenn die andere Achse fährt (MOTOR_ALLOW_PARALLEL = 0)
 *         oder nach nFAULT noch nicht quittiert ist (motor_ctrl_clear_fault()),
 *         ESP_ERR_TIMEOUT bei voller Queue
 */
esp_err_t motor_ctrl_goto(motor_id_t id, int32_t goal_steps, uint32_t timeout_ms);

/**
 * @brief Auf einen Prozentwert fahren (0 % = oben/offen, 100 % = unten/zu, mit
 * Sicherheitsabstand nach axis_percent_to_steps()).
 * @return ESP_ERR_INVALID_STATE, wenn die Achse nicht kalibriert ist
 */
esp_err_t motor_ctrl_goto_percent(motor_id_t id, uint8_t percent, uint32_t timeout_ms);

/** @brief Relativ fahren (Tippbetrieb im Kalibriermodus), Bezug = Position bei Verarbeitung. */
esp_err_t motor_ctrl_move_by(motor_id_t id, int32_t delta_steps, uint32_t timeout_ms);

/**
 * @brief Ohne Ziel fahren, bis Blockieren, Stopp oder Timeout (unkalibriert:
 * Auf/Ab; Kalibrierung: oben anfahren bis BLOCKED, unten bis Stopp).
 * Bei kalibrierter Achse endet die Fahrt am Endpunkt (MOTOR_EVT_LIMIT).
 */
esp_err_t motor_ctrl_run(motor_id_t id, motor_dir_t dir, uint32_t timeout_ms);

/**
 * @brief Fahrt anhalten (Kurzschlussbremse ~50 ms, dann aus) -> MOTOR_EVT_STOPPED.
 * Kann der Stopp nicht in die Queue gelegt werden, wird der Treiber sofort
 * abgeschaltet (beide Achsen) -> MOTOR_EVT_FAULT + Fehlersperre.
 * Hat Vorrang vor allen vorher abgeschickten, noch nicht verarbeiteten Fahrbefehlen
 * dieser Achse (die werden verworfen). Stand die Achse und war kein Befehl offen,
 * kommt kein Ereignis.
 */
esp_err_t motor_ctrl_stop(motor_id_t id);

/** @brief Beide Achsen anhalten. */
void motor_ctrl_stop_all(void);

/**
 * @brief true nach einem nFAULT (MOTOR_EVT_FAULT). Solange gesetzt, lehnen alle
 * Fahrbefehle mit ESP_ERR_INVALID_STATE ab (Zustand FEHLER der Applikation).
 * Auch ein Notaus aus motor_ctrl_stop()/stop_all() bei klemmender Queue führt hierher.
 */
bool motor_ctrl_fault_latched(void);

/**
 * @brief nFAULT quittieren (Applikation: FEHLER -> quittieren -> BEREIT).
 * @return ESP_ERR_INVALID_STATE, solange eine Achse busy ist
 */
esp_err_t motor_ctrl_clear_fault(void);

/** @brief Sicherheitsabstand (TILT_/LIFT_MARGIN_*_PERMILLE) an den Endpunkten ein-/ausschalten; rechnet die
 *         Ränder der kalibrierten Achsen neu. Nur im Stillstand (sonst ESP_ERR_INVALID_STATE). */
esp_err_t motor_ctrl_set_margins(bool on);

/**
 * @brief Motor als Buzzer (bei BEEP_ALL_MOTORS beide Motoren gleichzeitig, sonst nur id): Ton mit freq_hz für ms (blockiert, nur im Stillstand, nicht bei nFAULT-Sperre).
 *        Die Encoder-Position wird danach auf den Wert vor dem Ton zurückgesetzt.
 * @param duty_permille Pulsbreite je Polarität in ‰ (z. B. 250), auf BEEP_DUTY_MAX_PCT begrenzt
 * @return ESP_ERR_INVALID_STATE bei laufender Fahrt, Fehlersperre oder laufendem Ton
 */
esp_err_t motor_ctrl_beep(motor_id_t id, uint32_t freq_hz, uint32_t ms, uint16_t duty_permille);

/** Ein Ton: Frequenz in Hz (0 = Pause) und Dauer in ms. */
typedef struct {
    uint16_t hz;
    uint16_t ms;
} motor_note_t;

/**
 * @brief Tonfolge auf dem Motor (Treiber bleibt während der ganzen Folge eingeschaltet, kaum Lücken zwischen den Tönen).
 *        Blockiert bis zum Ende; Bedingungen wie motor_ctrl_beep().
 */
esp_err_t motor_ctrl_tune(motor_id_t id, const motor_note_t *notes, size_t n, uint16_t duty_permille);
bool motor_ctrl_margins_on(void);

/** @brief Messspur: Position/Duty alle 20 ms (Log-Tag "tr": "<motor> <ms> <pos> <duty>") für so viele Sekunden;
 *         0 = aus. Schaltet sich selbst ab. Für tools/rollo-debug.py. */
void motor_ctrl_trace(uint32_t seconds);

/** @brief true ab dem Befehl bis zum Abschluss-Ereignis. */
bool motor_ctrl_is_busy(motor_id_t id);

/** @brief true, wenn irgendeine Achse fährt (z. B. Akku nicht messen, Fenster nicht auswerten). */
bool motor_ctrl_any_busy(void);

/** @brief Aktuelle Position (Encoder, live). */
int32_t motor_ctrl_get_position(motor_id_t id);

/**
 * @brief Position setzen (Referenzieren, z. B. 0 am oberen Anschlag, oder Wert aus NVS).
 * @return ESP_ERR_INVALID_STATE während einer Fahrt
 */
esp_err_t motor_ctrl_set_position(motor_id_t id, int32_t pos);

/**
 * @brief Endpunkte setzen, Achse gilt danach als kalibriert (harte Grenzen aktiv,
 * Ränder aus LIFT_/TILT_MARGIN_*_PERMILLE).
 * @return ESP_ERR_INVALID_STATE während einer Fahrt, ESP_ERR_INVALID_ARG bei end_top == end_bottom
 */
esp_err_t motor_ctrl_set_endpoints(motor_id_t id, int32_t end_top, int32_t end_bottom);

/** @brief Kalibrierung verwerfen (z. B. vor dem Einlernen). Nur im Stillstand. */
esp_err_t motor_ctrl_clear_calibration(motor_id_t id);

/** @brief Kopie der Achsdaten (pos = aktuelle Encoder-Position). */
void motor_ctrl_get_axis(motor_id_t id, axis_t *out);

/** @brief Kurztext eines Ereignistyps (Logs). */
const char *motor_event_str(motor_event_type_t t);
