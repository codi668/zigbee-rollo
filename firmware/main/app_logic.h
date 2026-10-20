/**
 * @file app_logic.h
 * @brief Zustandsautomat der Rollosteuerung als reine Funktion (ohne ESP-IDF, Host-getestet).
 *
 * app_logic_step() bekommt ein Eingangsereignis (Start, Taste, Befehl, Motor,
 * Akku) und liefert eine Liste von Aktionen (Motorbefehle, NVS, Zigbee-Meldung).
 * Ausgeführt werden die Aktionen von app_state.c in der app_task. Damit ist die
 * ganze Entscheidungslogik – Zustandsübergänge, Plausibilität, Kalibrierablauf,
 * moving-Flag-Protokoll – auf dem PC testbar.
 *
 * Zustände (Spezifikation "Zustandsautomat"):
 *   NEUSTART      wartet auf APP_IN_BOOT (NVS gelesen)
 *   UNKALIBRIERT  mindestens eine Achse ohne Endpunkte: Auf/Ab/Stopp mit Blockiererkennung;
 *                 Prozentfahrten nur für bereits kalibrierte Achsen
 *   KALIBRIERUNG  Endpunkte einlernen (oben per Blockieren, unten manuell)
 *   BEREIT        beide Achsen kalibriert, wartet auf Befehl
 *   FAHRT         genau eine Achse fährt (MOTOR_ALLOW_PARALLEL = 0); Befehle für die andere
 *                 Achse werden vorgemerkt und danach ausgeführt
 *   FEHLER        Motor aus, LED-Code; quittieren mit kurzer Taste / Zigbee Stop / Konsole ack
 *   AKKU_LEER     Unterspannungssperre, keine Fahrten
 *
 * Achsen: 0 = Heben (MOTOR_LIFT), 1 = Wenden (MOTOR_TILT), gleiche Nummern wie motor_id_t.
 * Positionen in Encoder-Schritten, oben = 0, positive Schritte nach unten (axis.h).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "led_pattern.h"
#include "config.h"

#define APP_AXIS_LIFT      0
#define APP_AXIS_TILT      1
#define APP_AXIS_COUNT     2
#if ROLLO_TILT_ONLY
#define APP_AXIS_MASK_ALL  0x02   /**< Kalibrierung: nur Wenden (ROLLO_TILT_ONLY) */
#define APP_AXIS_BTN       APP_AXIS_TILT   /**< Achse der Taste "Auf/Ab im Wechsel" */
#else
#define APP_AXIS_MASK_ALL  0x03   /**< Kalibrierung: Bit 0 = Heben, Bit 1 = Wenden */
#define APP_AXIS_BTN       APP_AXIS_LIFT
#endif

/** Zustände */
typedef enum {
    APP_ST_BOOT = 0,       /**< NEUSTART */
    APP_ST_UNCALIBRATED,   /**< UNKALIBRIERT */
    APP_ST_CALIBRATING,    /**< KALIBRIERUNG */
    APP_ST_READY,          /**< BEREIT */
    APP_ST_MOVING,         /**< FAHRT */
    APP_ST_ERROR,          /**< FEHLER */
    APP_ST_BATTERY_EMPTY,  /**< AKKU_LEER */
    APP_ST_COUNT
} app_st_t;

/** Fehlerursache im Zustand FEHLER */
typedef enum {
    APP_ERR_NONE = 0,
    APP_ERR_BLOCKED,       /**< blockiert während einer Fahrt (Hindernis) */
    APP_ERR_TIMEOUT,       /**< Fahrzeit überschritten */
    APP_ERR_WRONG_DIR,     /**< Encoder zählt falsch herum (Mx_DIR_INVERT/ENCx_INVERT) */
    APP_ERR_FAULT,         /**< nFAULT des DRV8833 / Notaus */
    APP_ERR_HOMING,        /**< Referenzfahrt unplausibel (Blockieren mitten im Fahrbereich) */
    APP_ERR_CAL,           /**< Kalibrierung gescheitert (kein Anschlag, Motor läuft nicht, ...) */
} app_err_t;

/** Schritte der Kalibrierung einer Achse */
typedef enum {
    APP_CAL_NONE = 0,
    APP_CAL_UP,            /**< hochfahren bis Blockieren -> Endpunkt oben = 0 */
    APP_CAL_BACKOFF,       /**< kleines Stück zurück (Seil entlasten) */
    APP_CAL_DOWN,          /**< runterfahren bis Stopp (Taste kurz / Zigbee Stop / Konsole) */
    APP_CAL_JOG,           /**< Tippbetrieb, Speichern mit Taste 3 s / Mode-Bit 1 löschen / "cal save" */
} app_cal_t;

/** Art der laufenden Fahrt */
typedef enum {
    APP_MV_NONE = 0,
    APP_MV_RUN,            /**< ohne Ziel (unkalibriert Auf/Ab) */
    APP_MV_GOTO,           /**< Prozentfahrt */
    APP_MV_HOME,           /**< Referenzfahrt oben, danach optional Prozentfahrt */
    APP_MV_JOG,            /**< relativ */
} app_mv_t;

/** Tastenereignisse (gleiche Bedeutung wie ui_event_t) */
typedef enum {
    APP_BTN_SHORT = 0,
    APP_BTN_LONG_HINT,
    APP_BTN_LONG,
    APP_BTN_VERY_LONG,
} app_btn_t;

/** Motorereignisse (gleiche Bedeutung wie motor_event_type_t) */
typedef enum {
    APP_MEV_DONE = 0,
    APP_MEV_LIMIT,
    APP_MEV_STOPPED,
    APP_MEV_BLOCKED,
    APP_MEV_TIMEOUT,
    APP_MEV_WRONG_DIR,
    APP_MEV_FAULT,
    APP_MEV_REJECTED,
    APP_MEV_PROGRESS,
} app_mev_t;

/** Befehle (aus Zigbee, Konsole oder intern) */
typedef enum {
    APP_CMD_UP = 0,        /**< axis; Heben: Auf/Open */
    APP_CMD_DOWN,          /**< axis; Ab/Close */
    APP_CMD_STOP,          /**< alle Achsen; im FEHLER = quittieren */
    APP_CMD_GOTO,          /**< axis, value = Prozent 0…100 (Zigbee: 0 = offen); sonst abgewiesen */
    APP_CMD_CAL_START,     /**< axis = Maske (APP_AXIS_MASK_ALL) */
    APP_CMD_CAL_SAVE,      /**< Endpunkt unten speichern */
    APP_CMD_CAL_ABORT,
    APP_CMD_JOG,           /**< axis, value = Schritte (+ = nach unten) */
    APP_CMD_ACK,           /**< Fehler quittieren */
    APP_CMD_MODE_WRITE,    /**< value = Zigbee Window Covering Mode (Bit 1 = Kalibriermodus) */
    APP_CMD_FACTORY_RESET,
    APP_CMD_LIMIT_TOP,     /**< axis; nur im Stillstand: aktuelle Lage = oben (Position 0), Kalibrierung der Achse ungültig */
    APP_CMD_LIMIT_BOTTOM,  /**< axis; nur im Stillstand: aktuelle Lage = unten, speichern (Achse danach kalibriert) */
    APP_CMD_LIMIT_SET,     /**< axis, value = Weg oben->unten in Schritten: aktuelle Lage = oben, unten = value, speichern */
    APP_CMD_LIMIT_CLEAR,   /**< axis; Kalibrierung der Achse löschen (auch im NVS) */
    APP_CMD_MARGIN,        /**< value = 1/0: Sicherheitsabstand an den Endpunkten ein/aus (nur Stillstand) */
} app_cmd_t;

/** Art des Eingangsereignisses */
typedef enum {
    APP_IN_BOOT = 0,
    APP_IN_BTN,
    APP_IN_CMD,
    APP_IN_MOTOR,
    APP_IN_BATTERY,
} app_in_type_t;

/** Eingangsereignis */
typedef struct {
    app_in_type_t type;
    union {
        struct {
            int32_t bottom[APP_AXIS_COUNT];  /**< Endpunkt unten aus NVS, 0 = unkalibriert */
            int32_t pos[APP_AXIS_COUNT];     /**< letzte gespeicherte Position */
            bool    moving;                  /**< NVS moving = 1 -> Position unsicher */
            bool    lockout;                 /**< Unterspannungssperre der ersten Messung */
        } boot;
        app_btn_t btn;
        struct {
            app_cmd_t cmd;
            uint8_t   axis;
            int32_t   value;
        } cmd;
        struct {
            app_mev_t ev;
            uint8_t   axis;
            int32_t   pos;
        } motor;
        struct {
            bool lockout;
        } battery;
    };
} app_in_t;

/** Aktionen, die die app_task ausführt (in dieser Reihenfolge) */
typedef enum {
    APP_ACT_RUN = 0,         /**< axis, a = Richtung (-1 auf, +1 ab), b = Timeout ms (0 = Standard) */
    APP_ACT_GOTO_PCT,        /**< axis, a = Prozent */
    APP_ACT_MOVE_BY,         /**< axis, a = Schritte, b = Timeout ms (0 = Standard) */
    APP_ACT_STOP,            /**< beide Achsen anhalten */
    APP_ACT_CLEAR_CAL,       /**< axis: Endpunkte in motor_ctrl verwerfen (NVS bleibt) */
    APP_ACT_SET_POS,         /**< axis, a = Position (referenzieren) */
    APP_ACT_SET_ENDPOINTS,   /**< axis, a = Endpunkt unten (oben = 0) in motor_ctrl */
    APP_ACT_CLEAR_FAULT,     /**< motor_ctrl_clear_fault() */
    APP_ACT_NV_MOTION_BEGIN, /**< storage_motion_begin(): moving = 1 */
    APP_ACT_NV_MOTION_END,   /**< axis, a = Position: Position + moving = 0 */
    APP_ACT_NV_SAVE_POS,     /**< axis, a = Position (moving bleibt 1 -> nach Neustart Referenzfahrt) */
    APP_ACT_NV_SAVE_CAL,     /**< axis, a = Endpunkt unten */
    APP_ACT_REPORT,          /**< axis, a = 1 Endwert (sofort melden) / 0 Zwischenstand */
    APP_ACT_MEASURE,         /**< Akku nach der Fahrt messen (sensor_task, nach BATT_SETTLE_MS) */
    APP_ACT_FACTORY_RESET,   /**< NVS löschen, Zigbee-Netz verlassen, Neustart */
    APP_ACT_NV_CLEAR_CAL,    /**< axis: Endpunkt unten im NVS löschen */
    APP_ACT_SET_MARGIN,      /**< a = 1/0: Sicherheitsabstand setzen, im NVS speichern (nur Stillstand) */
} app_act_type_t;

typedef struct {
    app_act_type_t type;
    uint8_t        axis;
    int32_t        a;
    int32_t        b;
} app_act_t;

/** Hinweis für das Log (höchstens einer pro Schritt) */
typedef enum {
    APP_NOTE_NONE = 0,
    APP_NOTE_REJECTED_STATE,   /**< Befehl im aktuellen Zustand nicht erlaubt */
    APP_NOTE_NOT_CALIBRATED,   /**< Prozentfahrt ohne Kalibrierung der Achse */
    APP_NOTE_BATTERY_EMPTY,    /**< Fahrt wegen Unterspannungssperre abgelehnt */
    APP_NOTE_PENDING,          /**< Befehl für die andere Achse vorgemerkt */
    APP_NOTE_HOMING,           /**< Referenzfahrt oben vor der Prozentfahrt */
    APP_NOTE_CAL_TOO_SHORT,    /**< Endpunkt unten zu nah an oben, nicht gespeichert */
    APP_NOTE_CAL_AXIS_DONE,    /**< Achse kalibriert, ggf. nächste Achse */
    APP_NOTE_CAL_ABORTED,
    APP_NOTE_CAL_JOG,          /**< Kalibrierung: jetzt Tippbetrieb / Speichern möglich */
    APP_NOTE_ACKED,            /**< Fehler quittiert */
    APP_NOTE_IGNORED,          /**< Ereignis ohne Wirkung (z. B. verspätetes Motorereignis) */
    APP_NOTE_MODE_REVERSE,     /**< Mode-Bit 0 (Richtung umkehren) wird nicht unterstützt */
    APP_NOTE_INVALID_VALUE,    /**< Prozentwert außerhalb 0…100 (z. B. 0xFF), Befehl abgewiesen */
    APP_NOTE_LIMIT_TOP,        /**< manuell: oben gesetzt (Position 0), Achse unkalibriert */
    APP_NOTE_LIMIT_BOTTOM,     /**< manuell: unten gesetzt und gespeichert, Achse kalibriert */
    APP_NOTE_LIMIT_CLEARED,    /**< manuell: Kalibrierung gelöscht */
    APP_NOTE_MARGIN_ON,        /**< Sicherheitsabstand an den Endpunkten eingeschaltet */
    APP_NOTE_MARGIN_OFF,       /**< Sicherheitsabstand ausgeschaltet: 0 %/100 % = Endpunkte */
} app_note_t;

#define APP_ACTS_MAX 12

typedef struct {
    uint8_t    n;
    app_note_t note;
    app_act_t  a[APP_ACTS_MAX];
} app_acts_t;

/** Vorgemerkter Befehl einer Achse (während die andere fährt) */
typedef struct {
    bool      valid;
    app_cmd_t cmd;
    int32_t   value;
} app_pending_t;

/** Gesamter Zustand – Felder nur über app_logic_* ändern, lesen ist erlaubt */
typedef struct {
    app_st_t  st;
    app_err_t err;
    uint8_t   err_axis;
    bool      lockout;                         /**< letzte Akku-Messung: gesperrt */
    bool      hint;                            /**< Taste 3 s erreicht (LED-Rückmeldung) */

    bool      calibrated[APP_AXIS_COUNT];      /**< Endpunkte in motor_ctrl aktiv */
    int32_t   bottom[APP_AXIS_COUNT];          /**< Endpunkt unten (aktiv) */
    int32_t   nv_bottom[APP_AXIS_COUNT];       /**< Endpunkt unten im NVS (zum Wiederherstellen) */
    bool      needs_homing[APP_AXIS_COUNT];    /**< Position unsicher -> vor der nächsten Prozentfahrt referenzieren */
    int8_t    last_dir[APP_AXIS_COUNT];        /**< Taste "Auf/Ab im Wechsel" */
    int32_t   pos[APP_AXIS_COUNT];             /**< letzte bekannte Position (aus Ereignissen) */

    /* FAHRT */
    uint8_t   mv_axis;
    app_mv_t  mv_kind;
    int8_t    mv_dir;                          /**< nur RUN */
    int16_t   mv_target;                       /**< HOME: Prozentziel danach, -1 = keins */
    bool      stop_req;
    app_pending_t pending[APP_AXIS_COUNT];

    /* KALIBRIERUNG */
    app_cal_t cal_step;
    uint8_t   cal_axis;
    uint8_t   cal_mask;                        /**< noch zu kalibrierende Achsen (ohne cal_axis) */
    bool      cal_moving;
    bool      cal_save_req;                    /**< nach dem Stopp speichern */
    bool      cal_abort_req;                   /**< nach dem Stopp abbrechen */
} app_ctx_t;

/** @brief Zustand NEUSTART, alles unkalibriert. */
void app_logic_init(app_ctx_t *c);

/** @brief Ein Ereignis verarbeiten; out wird geleert und gefüllt. */
void app_logic_step(app_ctx_t *c, const app_in_t *in, app_acts_t *out);

/** @brief ConfigStatus "Operational": BEREIT oder FAHRT mit beiden Achsen kalibriert. */
bool app_logic_operational(const app_ctx_t *c);

/** @brief Mode-Bit 1 (Kalibriermodus aktiv). */
bool app_logic_calibrating(const app_ctx_t *c);

/**
 * @brief LED-Muster: Identify/3-s-Hinweis = an, FEHLER = Doppelblitz,
 * KALIBRIERUNG = langsam, Pairing (Netzsuche) = schnell, sonst aus.
 * AKKU_LEER = aus (Strom sparen).
 */
led_pattern_t app_logic_led(const app_ctx_t *c, bool zb_joining, bool identify);

/** @brief Zustandsname wie in der Spezifikation (NEUSTART, BEREIT, ...). */
const char *app_logic_state_name(app_st_t st);
const char *app_logic_err_name(app_err_t e);
const char *app_logic_note_str(app_note_t n);
const char *app_logic_cal_name(app_cal_t s);
