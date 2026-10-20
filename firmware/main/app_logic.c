/*
 * app_logic: Zustandsautomat als reine Funktion (Spezifikation "Zustandsautomat",
 * "Endpunkte setzen und Mitzählen", "Speicher"). Keine ESP-IDF-Abhängigkeit,
 * Host-getestet in test_host/test_app_logic.c.
 *
 * Regeln, die überall gelten:
 * - Vor jeder Fahrt NV_MOTION_BEGIN (moving = 1), am Ende NV_MOTION_END (Position,
 *   moving = 0). Ist danach noch eine Achse "needs_homing", bleibt moving = 1
 *   (NV_SAVE_POS), damit nach einem Neustart neu referenziert wird.
 * - Motorbefehle stehen immer am Ende der Aktionsliste. Scheitert einer, speist
 *   app_state ein Ersatz-Ereignis (REJECTED/FAULT) ein.
 * - SET_ENDPOINTS/SET_POS/CLEAR_CAL nur im Stillstand (nach einem Abschluss-Ereignis
 *   oder bevor eine Fahrt beginnt) – so verlangt es motor_ctrl.
 */
#include "app_logic.h"
#include <string.h>
#include "config.h"

/* ---- Parameter je Achse aus config.h ---- */
static const int32_t s_backoff[APP_AXIS_COUNT]    = { CAL_BACKOFF_STEPS_LIFT, CAL_BACKOFF_STEPS_TILT };
static const int32_t s_jog[APP_AXIS_COUNT]        = { CAL_JOG_STEPS_LIFT, CAL_JOG_STEPS_TILT };
static const int32_t s_min_travel[APP_AXIS_COUNT] = { CAL_MIN_TRAVEL_STEPS_LIFT, CAL_MIN_TRAVEL_STEPS_TILT };
static const bool    s_bottom_block[APP_AXIS_COUNT] = { false, TILT_CAL_BOTTOM_BY_BLOCK != 0 };
static const bool    s_rehome_open[APP_AXIS_COUNT]  = { LIFT_REHOME_ON_OPEN != 0, TILT_REHOME_ON_OPEN != 0 };

#define CAL_TIMEOUT_MS   MOTOR_TIMEOUT_MAX_MS   /* Kalibrier- und Referenzfahrten: längstes erlaubtes Limit */

/* ------------------------------------------------------------------------- */

static void act(app_acts_t *o, app_act_type_t t, uint8_t axis, int32_t a, int32_t b)
{
    if (o->n < APP_ACTS_MAX) {
        o->a[o->n].type = t;
        o->a[o->n].axis = axis;
        o->a[o->n].a = a;
        o->a[o->n].b = b;
        o->n++;
    }
}

static void note(app_acts_t *o, app_note_t n)
{
    if (o->note == APP_NOTE_NONE) o->note = n;
}

/** Gültige Achse für Befehle. Bei ROLLO_TILT_ONLY gibt es nur Wenden: Befehle für Heben
 *  (Zigbee Auf/Zu/Lift-Prozent) werden abgewiesen, damit nur der Kipp-Regler fährt. */
static bool valid_axis(uint8_t a)
{
#if ROLLO_TILT_ONLY
    return a == APP_AXIS_TILT;
#else
    return a < APP_AXIS_COUNT;
#endif
}

static bool all_calibrated(const app_ctx_t *c)
{
#if ROLLO_TILT_ONLY
    return c->calibrated[APP_AXIS_TILT];
#else
    return c->calibrated[APP_AXIS_LIFT] && c->calibrated[APP_AXIS_TILT];
#endif
}

static bool any_needs_homing(const app_ctx_t *c)
{
#if ROLLO_TILT_ONLY
    return c->needs_homing[APP_AXIS_TILT];
#else
    return c->needs_homing[APP_AXIS_LIFT] || c->needs_homing[APP_AXIS_TILT];
#endif
}


/** Ruhezustand: AKKU_LEER > BEREIT/UNKALIBRIERT */
static void go_base(app_ctx_t *c)
{
    c->mv_kind = APP_MV_NONE;
    c->cal_step = APP_CAL_NONE;
    c->cal_moving = false;
    c->stop_req = false;
    if (c->lockout) c->st = APP_ST_BATTERY_EMPTY;
    else c->st = all_calibrated(c) ? APP_ST_READY : APP_ST_UNCALIBRATED;
}

/** Position im NVS ablegen: moving = 0 nur, wenn keine Achse referenziert werden muss */
static void nv_end(const app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t pos)
{
    act(o, any_needs_homing(c) ? APP_ACT_NV_SAVE_POS : APP_ACT_NV_MOTION_END, axis, pos, 0);
}

static int32_t home_tol(const app_ctx_t *c, uint8_t axis)
{
    int32_t travel = c->bottom[axis] > 0 ? c->bottom[axis] : -c->bottom[axis];
    int32_t t = (int32_t)((int64_t)travel * HOME_TOL_PERMILLE / 1000);
    return t > HOME_TOL_MIN_STEPS ? t : HOME_TOL_MIN_STEPS;
}

/** Prozentwert aus Zigbee/Konsole: nur 0…100 (0xFF = "unbekannt" o. Ä. wird abgewiesen, nicht begrenzt) */
static bool valid_pct(int32_t v)
{
    return v >= 0 && v <= 100;
}

/* ---- Fahrten starten (nur aus BEREIT/UNKALIBRIERT) ---- */

static void begin_motion(app_ctx_t *c, app_acts_t *o, uint8_t axis, app_mv_t kind)
{
    c->st = APP_ST_MOVING;
    c->mv_axis = axis;
    c->mv_kind = kind;
    c->stop_req = false;
    c->mv_target = -1;
    act(o, APP_ACT_NV_MOTION_BEGIN, axis, 0, 0);
}

static void start_home(app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t target_pct)
{
    /* motor_ctrl kennt dabei keine Endpunkte (sonst stoppte run() am alten Endpunkt oben) */
    act(o, APP_ACT_CLEAR_CAL, axis, 0, 0);
    begin_motion(c, o, axis, APP_MV_HOME);
    c->mv_target = (int16_t)target_pct;
    c->last_dir[axis] = -1;
    act(o, APP_ACT_RUN, axis, -1, CAL_TIMEOUT_MS);
    note(o, APP_NOTE_HOMING);
}

static void start_goto(app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t pct)
{
    begin_motion(c, o, axis, APP_MV_GOTO);
    act(o, APP_ACT_GOTO_PCT, axis, pct, 0);
}

static void start_run(app_ctx_t *c, app_acts_t *o, uint8_t axis, int8_t dir)
{
    begin_motion(c, o, axis, APP_MV_RUN);
    c->mv_dir = dir;
    act(o, APP_ACT_RUN, axis, dir, 0);
}

static void start_jog(app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t delta)
{
    begin_motion(c, o, axis, APP_MV_JOG);
    act(o, APP_ACT_MOVE_BY, axis, delta, 0);
}

/** Befehl aus dem Ruhezustand ausführen. @return true, wenn eine Fahrt beginnt */
static bool start_cmd(app_ctx_t *c, app_acts_t *o, app_cmd_t cmd, uint8_t axis, int32_t value)
{
    if (!valid_axis(axis)) {
        note(o, APP_NOTE_REJECTED_STATE);
        return false;
    }
    if (c->lockout) {
        note(o, APP_NOTE_BATTERY_EMPTY);
        return false;
    }
    switch (cmd) {
    case APP_CMD_UP:
        c->last_dir[axis] = -1;
        if (!c->calibrated[axis]) start_run(c, o, axis, -1);
        else if (c->needs_homing[axis] || s_rehome_open[axis]) start_home(c, o, axis, 0);
        else start_goto(c, o, axis, 0);
        return true;
    case APP_CMD_DOWN:
        c->last_dir[axis] = 1;
        if (!c->calibrated[axis]) start_run(c, o, axis, 1);
        else if (c->needs_homing[axis]) start_home(c, o, axis, 100);
        else start_goto(c, o, axis, 100);
        return true;
    case APP_CMD_GOTO: {
        if (!valid_pct(value)) {
            note(o, APP_NOTE_INVALID_VALUE);
            return false;
        }
        if (!c->calibrated[axis]) {
            note(o, APP_NOTE_NOT_CALIBRATED);
            return false;
        }
        int32_t pct = value;
        c->last_dir[axis] = 1;
        if (c->needs_homing[axis] || (pct == 0 && s_rehome_open[axis])) start_home(c, o, axis, pct);
        else start_goto(c, o, axis, pct);
        return true;
    }
    case APP_CMD_JOG:
        if (value == 0) return false;
        c->last_dir[axis] = value < 0 ? -1 : 1;
        start_jog(c, o, axis, value);
        return true;
    default:
        return false;
    }
}

/** Nach einer Fahrt: vorgemerkten Befehl der anderen Achse starten */
static void run_pending(app_ctx_t *c, app_acts_t *o)
{
    if (c->st != APP_ST_READY && c->st != APP_ST_UNCALIBRATED) {
        c->pending[0].valid = c->pending[1].valid = false;
        return;
    }
    for (uint8_t i = 0; i < APP_AXIS_COUNT; i++) {
        if (!c->pending[i].valid) continue;
        app_pending_t p = c->pending[i];
        c->pending[i].valid = false;
        if (start_cmd(c, o, p.cmd, i, p.value)) return;
    }
}

/* ---- FEHLER ---- */

static void enter_error(app_ctx_t *c, app_acts_t *o, app_err_t err, uint8_t axis, int32_t pos)
{
    bool was_cal = c->st == APP_ST_CALIBRATING;
    bool was_home = c->st == APP_ST_MOVING && c->mv_kind == APP_MV_HOME && c->mv_axis == axis;
    c->st = APP_ST_ERROR;
    c->err = err;
    c->err_axis = axis;
    c->mv_kind = APP_MV_NONE;
    c->cal_step = APP_CAL_NONE;
    c->cal_moving = false;
    c->cal_mask = 0;
    c->stop_req = false;
    c->pending[0].valid = c->pending[1].valid = false;
    act(o, APP_ACT_STOP, axis, 0, 0);   /* zur Sicherheit, ohne Wirkung wenn alles steht */
    if (!valid_axis(axis)) return;
    c->pos[axis] = pos;

    if (was_cal || was_home) {
        /* Endpunkte waren in motor_ctrl verworfen: aus dem NVS wiederherstellen.
         * Der Positionsbezug (oben = 0) ist danach unsicher -> neu referenzieren. */
        if (c->nv_bottom[axis] != 0) {
            c->calibrated[axis] = true;
            c->bottom[axis] = c->nv_bottom[axis];
            act(o, APP_ACT_SET_ENDPOINTS, axis, c->bottom[axis], 0);
            c->needs_homing[axis] = true;
        } else {
            c->calibrated[axis] = false;
            c->bottom[axis] = 0;
        }
    } else if (c->calibrated[axis]) {
        /* Blockieren/Timeout/Richtungsfehler/nFAULT mitten im Fahrbereich: nach dem
         * Quittieren oben neu referenzieren (Spezifikation). nFAULT eingeschlossen:
         * Überstrom heißt meist Blockieren, und die Achse trudelt unkontrolliert aus. */
        c->needs_homing[axis] = true;
    }
    nv_end(c, o, axis, pos);
    act(o, APP_ACT_REPORT, axis, 1, 0);
}

static void ack_error(app_ctx_t *c, app_acts_t *o)
{
    act(o, APP_ACT_CLEAR_FAULT, 0, 0, 0);
    c->err = APP_ERR_NONE;
    go_base(c);
    note(o, APP_NOTE_ACKED);
}

/* ---- FAHRT beendet ---- */

static void finish_motion(app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t pos)
{
    nv_end(c, o, axis, pos);
    act(o, APP_ACT_REPORT, axis, 1, 0);
    act(o, APP_ACT_MEASURE, axis, 0, 0);
    bool stopped = c->stop_req;
    go_base(c);
    if (!stopped) run_pending(c, o);
    else c->pending[0].valid = c->pending[1].valid = false;
}

static void restore_endpoints(app_ctx_t *c, app_acts_t *o, uint8_t axis)
{
    if (c->calibrated[axis]) act(o, APP_ACT_SET_ENDPOINTS, axis, c->bottom[axis], 0);
}

static void moving_motor(app_ctx_t *c, app_acts_t *o, app_mev_t ev, uint8_t axis, int32_t pos)
{
    if (axis != c->mv_axis) {
        note(o, APP_NOTE_IGNORED);
        return;
    }
    c->pos[axis] = pos;
    switch (c->mv_kind) {
    case APP_MV_RUN:
        switch (ev) {
        case APP_MEV_BLOCKED:
            if (c->mv_dir < 0) {
                /* unkalibriert "Auf": Anschlag oben erreicht -> Position 0 */
                act(o, APP_ACT_SET_POS, axis, 0, 0);
                c->pos[axis] = 0;
                finish_motion(c, o, axis, 0);
            } else {
                enter_error(c, o, APP_ERR_BLOCKED, axis, pos);
            }
            break;
        case APP_MEV_TIMEOUT:   enter_error(c, o, APP_ERR_TIMEOUT, axis, pos); break;
        case APP_MEV_WRONG_DIR: enter_error(c, o, APP_ERR_WRONG_DIR, axis, pos); break;
        default:                finish_motion(c, o, axis, pos); break;   /* STOPPED, DONE, LIMIT, REJECTED */
        }
        break;

    case APP_MV_GOTO:
    case APP_MV_JOG:
        switch (ev) {
        case APP_MEV_BLOCKED:   enter_error(c, o, APP_ERR_BLOCKED, axis, pos); break;
        case APP_MEV_TIMEOUT:   enter_error(c, o, APP_ERR_TIMEOUT, axis, pos); break;
        case APP_MEV_WRONG_DIR: enter_error(c, o, APP_ERR_WRONG_DIR, axis, pos); break;
        default:                finish_motion(c, o, axis, pos); break;
        }
        break;

    case APP_MV_HOME:
        switch (ev) {
        case APP_MEV_BLOCKED:
            if (!c->needs_homing[axis] && pos > home_tol(c, axis)) {
                /* Position war bekannt und der "Anschlag" liegt mitten im Fahrbereich:
                 * Hindernis, nicht die Kopfschiene */
                enter_error(c, o, APP_ERR_HOMING, axis, pos);
                break;
            }
            act(o, APP_ACT_SET_POS, axis, 0, 0);
            act(o, APP_ACT_SET_ENDPOINTS, axis, c->bottom[axis], 0);
            c->needs_homing[axis] = false;
            c->pos[axis] = 0;
            if (c->stop_req || c->mv_target < 0) {
                finish_motion(c, o, axis, 0);
            } else {
                /* weiter zum Prozentziel, moving bleibt 1 */
                c->mv_kind = APP_MV_GOTO;
                act(o, APP_ACT_REPORT, axis, 0, 0);
                act(o, APP_ACT_GOTO_PCT, axis, c->mv_target, 0);
            }
            break;
        case APP_MEV_TIMEOUT:   enter_error(c, o, APP_ERR_TIMEOUT, axis, pos); break;
        case APP_MEV_WRONG_DIR: enter_error(c, o, APP_ERR_WRONG_DIR, axis, pos); break;
        default:
            /* gestoppt/abgelehnt vor dem Anschlag: alter Bezug bleibt, Endpunkte zurück */
            restore_endpoints(c, o, axis);
            finish_motion(c, o, axis, pos);
            break;
        }
        break;

    default:
        note(o, APP_NOTE_IGNORED);
        break;
    }
}

/** Befehl während der Fahrt */
static void moving_cmd(app_ctx_t *c, app_acts_t *o, app_cmd_t cmd, uint8_t axis, int32_t value)
{
    if (cmd == APP_CMD_STOP) {
        c->stop_req = true;
        c->pending[0].valid = c->pending[1].valid = false;
        act(o, APP_ACT_STOP, c->mv_axis, 0, 0);
        return;
    }
    if (cmd == APP_CMD_FACTORY_RESET) {
        act(o, APP_ACT_STOP, 0, 0, 0);
        act(o, APP_ACT_FACTORY_RESET, 0, 0, 0);
        return;
    }
    if (cmd != APP_CMD_UP && cmd != APP_CMD_DOWN && cmd != APP_CMD_GOTO && cmd != APP_CMD_JOG) {
        note(o, APP_NOTE_REJECTED_STATE);
        return;
    }
    if (!valid_axis(axis)) {
        note(o, APP_NOTE_REJECTED_STATE);
        return;
    }
    if (cmd == APP_CMD_GOTO && !valid_pct(value)) {
        note(o, APP_NOTE_INVALID_VALUE);
        return;
    }
    if (axis != c->mv_axis) {
        /* MOTOR_ALLOW_PARALLEL = 0: nach dieser Fahrt ausführen (letzter Befehl gewinnt) */
        c->pending[axis].valid = true;
        c->pending[axis].cmd = cmd;
        c->pending[axis].value = value;
        note(o, APP_NOTE_PENDING);
        return;
    }
    if (c->stop_req || cmd == APP_CMD_JOG) {
        note(o, APP_NOTE_REJECTED_STATE);
        return;
    }
    int32_t pct = cmd == APP_CMD_UP ? 0 : cmd == APP_CMD_DOWN ? 100 : value;
    if (!c->calibrated[axis] && c->mv_kind == APP_MV_JOG) {
        /* Tippen auf unkalibrierter Achse: Auf/Ab wie unkalibriert (RUN), nie Prozentfahrt */
        if (cmd == APP_CMD_GOTO) {
            note(o, APP_NOTE_NOT_CALIBRATED);
            return;
        }
        c->mv_kind = APP_MV_RUN;
    }
    switch (c->mv_kind) {
    case APP_MV_HOME:
        c->mv_target = (int16_t)pct;   /* Referenzfahrt läuft weiter, danach neues Ziel */
        break;
    case APP_MV_GOTO:
    case APP_MV_JOG:
        c->mv_kind = APP_MV_GOTO;
        c->last_dir[axis] = cmd == APP_CMD_UP ? -1 : 1;
        act(o, APP_ACT_GOTO_PCT, axis, pct, 0);   /* motor_ctrl ersetzt das Ziel */
        break;
    case APP_MV_RUN:
        if (cmd == APP_CMD_GOTO) {
            note(o, APP_NOTE_NOT_CALIBRATED);
        } else {
            c->mv_dir = cmd == APP_CMD_UP ? -1 : 1;
            c->last_dir[axis] = c->mv_dir;
            act(o, APP_ACT_RUN, axis, c->mv_dir, 0);
        }
        break;
    default:
        break;
    }
}

/* ---- KALIBRIERUNG ---- */

static void cal_finish(app_ctx_t *c, app_acts_t *o);

static void cal_start_axis(app_ctx_t *c, app_acts_t *o)
{
    uint8_t axis = (c->cal_mask & 1) ? APP_AXIS_LIFT : APP_AXIS_TILT;
    c->cal_mask &= (uint8_t)~(1u << axis);
    c->cal_axis = axis;
    c->cal_step = APP_CAL_UP;
    c->cal_save_req = false;
    c->cal_abort_req = false;
    c->calibrated[axis] = false;
    act(o, APP_ACT_CLEAR_CAL, axis, 0, 0);
    act(o, APP_ACT_NV_MOTION_BEGIN, axis, 0, 0);
    act(o, APP_ACT_REPORT, axis, 1, 0);
    act(o, APP_ACT_RUN, axis, -1, CAL_TIMEOUT_MS);
    c->cal_moving = true;
}

static void cal_begin(app_ctx_t *c, app_acts_t *o, uint8_t mask)
{
    mask &= APP_AXIS_MASK_ALL;
    if (!mask) return;
    if (c->lockout) {
        note(o, APP_NOTE_BATTERY_EMPTY);
        return;
    }
    c->st = APP_ST_CALIBRATING;
    c->mv_kind = APP_MV_NONE;
    c->pending[0].valid = c->pending[1].valid = false;
    c->cal_mask = mask;
    cal_start_axis(c, o);
}

static void cal_finish(app_ctx_t *c, app_acts_t *o)
{
    (void)o;
    c->cal_mask = 0;
    go_base(c);
}

static void cal_jog(app_ctx_t *c, app_acts_t *o, int32_t delta)
{
    if (delta == 0) return;
    act(o, APP_ACT_NV_MOTION_BEGIN, c->cal_axis, 0, 0);
    act(o, APP_ACT_MOVE_BY, c->cal_axis, delta, 0);
    c->cal_moving = true;
}

static void cal_save(app_ctx_t *c, app_acts_t *o)
{
    uint8_t axis = c->cal_axis;
    int32_t pos = c->pos[axis];
    c->cal_save_req = false;
    if (pos < s_min_travel[axis]) {
        note(o, APP_NOTE_CAL_TOO_SHORT);
        c->cal_step = APP_CAL_JOG;
        return;
    }
    act(o, APP_ACT_SET_ENDPOINTS, axis, pos, 0);
    act(o, APP_ACT_NV_SAVE_CAL, axis, pos, 0);
    c->calibrated[axis] = true;
    c->bottom[axis] = c->nv_bottom[axis] = pos;
    c->needs_homing[axis] = false;
    nv_end(c, o, axis, pos);
    act(o, APP_ACT_REPORT, axis, 1, 0);
    note(o, APP_NOTE_CAL_AXIS_DONE);
    if (c->cal_mask) cal_start_axis(c, o);
    else cal_finish(c, o);
}

static void cal_abort(app_ctx_t *c, app_acts_t *o)
{
    uint8_t axis = c->cal_axis;
    /* Alte Endpunkte aus dem NVS. Der Bezug oben = 0 ist danach nicht mehr sicher:
     * Die Achse ist ohne Endpunkte gefahren, und "oben" kann in dieser Kalibrierung an
     * einem Hindernis statt am Anschlag gesetzt worden sein -> vor der nächsten
     * Prozentfahrt neu referenzieren (kostet höchstens eine Referenzfahrt). */
    if (c->nv_bottom[axis] != 0) {
        c->calibrated[axis] = true;
        c->bottom[axis] = c->nv_bottom[axis];
        c->needs_homing[axis] = true;
        act(o, APP_ACT_SET_ENDPOINTS, axis, c->bottom[axis], 0);
    } else {
        c->calibrated[axis] = false;
        c->bottom[axis] = 0;
    }
    c->cal_abort_req = false;
    nv_end(c, o, axis, c->pos[axis]);
    act(o, APP_ACT_REPORT, axis, 1, 0);
    note(o, APP_NOTE_CAL_ABORTED);
    cal_finish(c, o);
}

static void cal_motor(app_ctx_t *c, app_acts_t *o, app_mev_t ev, uint8_t axis, int32_t pos)
{
    if (axis != c->cal_axis || !c->cal_moving) {
        note(o, APP_NOTE_IGNORED);
        return;
    }
    c->cal_moving = false;
    c->pos[axis] = pos;
    if (ev == APP_MEV_REJECTED) {
        cal_abort(c, o);
        return;
    }
    if (ev == APP_MEV_WRONG_DIR) {
        enter_error(c, o, APP_ERR_WRONG_DIR, axis, pos);
        return;
    }
    if (c->cal_abort_req) {
        cal_abort(c, o);
        return;
    }
    switch (c->cal_step) {
    case APP_CAL_UP:
        if (ev == APP_MEV_BLOCKED) {
            /* Anschlag oben = Endpunkt oben = 0 */
            act(o, APP_ACT_SET_POS, axis, 0, 0);
            c->pos[axis] = 0;
            act(o, APP_ACT_REPORT, axis, 0, 0);
            if (s_backoff[axis] > 0) {
                c->cal_step = APP_CAL_BACKOFF;
                act(o, APP_ACT_MOVE_BY, axis, s_backoff[axis], 0);
            } else {
                c->cal_step = APP_CAL_DOWN;
                act(o, APP_ACT_RUN, axis, 1, CAL_TIMEOUT_MS);
            }
            c->cal_moving = true;
        } else if (ev == APP_MEV_STOPPED) {
            cal_abort(c, o);
        } else {
            enter_error(c, o, ev == APP_MEV_TIMEOUT ? APP_ERR_TIMEOUT : APP_ERR_CAL, axis, pos);
        }
        break;

    case APP_CAL_BACKOFF:
        if (ev == APP_MEV_DONE || ev == APP_MEV_LIMIT || ev == APP_MEV_STOPPED) {
            c->cal_step = APP_CAL_DOWN;
            act(o, APP_ACT_RUN, axis, 1, CAL_TIMEOUT_MS);
            c->cal_moving = true;
        } else {
            enter_error(c, o, APP_ERR_CAL, axis, pos);
        }
        break;

    case APP_CAL_DOWN:
        if (ev == APP_MEV_BLOCKED) {
            if (s_bottom_block[axis]) {
                act(o, APP_ACT_REPORT, axis, 0, 0);
                cal_save(c, o);
            } else {
                /* kein Anschlag unten erwartet (Seil): Hindernis oder Motor läuft nicht */
                enter_error(c, o, APP_ERR_CAL, axis, pos);
            }
            break;
        }
        /* STOPPED (Taste/Zigbee Stop), TIMEOUT (zu lange), DONE/LIMIT: jetzt Tippbetrieb */
        c->cal_step = APP_CAL_JOG;
        act(o, APP_ACT_REPORT, axis, 0, 0);
        if (c->cal_save_req) cal_save(c, o);
        else note(o, APP_NOTE_CAL_JOG);
        break;

    case APP_CAL_JOG:
        if (ev == APP_MEV_BLOCKED && !s_bottom_block[axis]) {
            enter_error(c, o, APP_ERR_CAL, axis, pos);
            break;
        }
        if (ev == APP_MEV_TIMEOUT) {
            enter_error(c, o, APP_ERR_TIMEOUT, axis, pos);
            break;
        }
        act(o, APP_ACT_REPORT, axis, 0, 0);
        if (c->cal_save_req) cal_save(c, o);
        break;

    default:
        note(o, APP_NOTE_IGNORED);
        break;
    }
}

static void cal_request_save(app_ctx_t *c, app_acts_t *o)
{
    if (c->cal_step == APP_CAL_JOG && !c->cal_moving) {
        cal_save(c, o);
    } else if (c->cal_step == APP_CAL_DOWN || c->cal_step == APP_CAL_JOG) {
        c->cal_save_req = true;
        act(o, APP_ACT_STOP, c->cal_axis, 0, 0);
    } else {
        note(o, APP_NOTE_REJECTED_STATE);   /* oben noch nicht gefunden */
    }
}

static void cal_request_abort(app_ctx_t *c, app_acts_t *o)
{
    if (c->cal_moving) {
        c->cal_abort_req = true;
        act(o, APP_ACT_STOP, c->cal_axis, 0, 0);
    } else {
        cal_abort(c, o);
    }
}

static void cal_cmd(app_ctx_t *c, app_acts_t *o, app_cmd_t cmd, int32_t value)
{
    switch (cmd) {
    case APP_CMD_STOP:
        if (c->cal_moving) act(o, APP_ACT_STOP, c->cal_axis, 0, 0);
        break;
    case APP_CMD_UP:
    case APP_CMD_DOWN:
    case APP_CMD_JOG:
        if (c->cal_step == APP_CAL_JOG && !c->cal_moving) {
            int32_t d = cmd == APP_CMD_UP ? -s_jog[c->cal_axis]
                      : cmd == APP_CMD_DOWN ? s_jog[c->cal_axis] : value;
            cal_jog(c, o, d);
        } else {
            note(o, APP_NOTE_REJECTED_STATE);
        }
        break;
    case APP_CMD_CAL_SAVE:
        cal_request_save(c, o);
        break;
    case APP_CMD_CAL_ABORT:
        cal_request_abort(c, o);
        break;
    case APP_CMD_MODE_WRITE:
        if (value & 0x01) note(o, APP_NOTE_MODE_REVERSE);
        if (!(value & 0x02)) {
            /* Kalibriermodus-Bit gelöscht: speichern, wenn unten schon angefahren, sonst abbrechen */
            if (c->cal_step == APP_CAL_DOWN || c->cal_step == APP_CAL_JOG) cal_request_save(c, o);
            else cal_request_abort(c, o);
        }
        break;
    case APP_CMD_FACTORY_RESET:
        act(o, APP_ACT_STOP, 0, 0, 0);
        act(o, APP_ACT_FACTORY_RESET, 0, 0, 0);
        break;
    default:
        note(o, APP_NOTE_REJECTED_STATE);
        break;
    }
}

static void cal_btn(app_ctx_t *c, app_acts_t *o, app_btn_t b)
{
    switch (b) {
    case APP_BTN_SHORT:
        if (c->cal_moving) act(o, APP_ACT_STOP, c->cal_axis, 0, 0);   /* Endpunkt unten anhalten */
        else if (c->cal_step == APP_CAL_JOG) cal_jog(c, o, s_jog[c->cal_axis]);   /* Tippbetrieb */
        break;
    case APP_BTN_LONG:
        cal_request_save(c, o);
        break;
    default:
        break;
    }
}

/* ---- Ruhezustände ---- */

/** Kalibrierung einer Achse verwerfen (motor_ctrl und NVS); Achse danach unkalibriert */
static void drop_cal(app_ctx_t *c, app_acts_t *o, uint8_t axis)
{
    act(o, APP_ACT_CLEAR_CAL, axis, 0, 0);
    act(o, APP_ACT_NV_CLEAR_CAL, axis, 0, 0);
    c->calibrated[axis] = false;
    c->bottom[axis] = c->nv_bottom[axis] = 0;
    c->needs_homing[axis] = false;
}

/** Endpunkt unten (Weg ab oben = 0) setzen und speichern; Achse danach kalibriert */
static void store_bottom(app_ctx_t *c, app_acts_t *o, uint8_t axis, int32_t bottom)
{
    act(o, APP_ACT_SET_ENDPOINTS, axis, bottom, 0);
    act(o, APP_ACT_NV_SAVE_CAL, axis, bottom, 0);
    c->calibrated[axis] = true;
    c->bottom[axis] = c->nv_bottom[axis] = bottom;
    c->needs_homing[axis] = false;
}

/** Manuelle Endpunkte (Stillstand): Konsole "limit ...", für Aufbauten ohne Anschlag */
static void limit_cmd(app_ctx_t *c, app_acts_t *o, app_cmd_t cmd, uint8_t axis, int32_t value)
{
    if (!valid_axis(axis)) {
        note(o, APP_NOTE_REJECTED_STATE);
        return;
    }
    switch (cmd) {
    case APP_CMD_LIMIT_TOP:
        /* neuer Nullpunkt: der alte Endpunkt unten passt nicht mehr dazu */
        drop_cal(c, o, axis);
        act(o, APP_ACT_SET_POS, axis, 0, 0);
        c->pos[axis] = 0;
        note(o, APP_NOTE_LIMIT_TOP);
        break;
    case APP_CMD_LIMIT_BOTTOM:
        if (c->pos[axis] < s_min_travel[axis]) {
            note(o, APP_NOTE_CAL_TOO_SHORT);
            return;
        }
        store_bottom(c, o, axis, c->pos[axis]);
        note(o, APP_NOTE_LIMIT_BOTTOM);
        break;
    case APP_CMD_LIMIT_SET:
        if (value < s_min_travel[axis]) {
            note(o, APP_NOTE_CAL_TOO_SHORT);
            return;
        }
        act(o, APP_ACT_SET_POS, axis, 0, 0);
        c->pos[axis] = 0;
        store_bottom(c, o, axis, value);
        note(o, APP_NOTE_LIMIT_BOTTOM);
        break;
    default:   /* APP_CMD_LIMIT_CLEAR */
        drop_cal(c, o, axis);
        note(o, APP_NOTE_LIMIT_CLEARED);
        break;
    }
    nv_end(c, o, axis, c->pos[axis]);
    act(o, APP_ACT_REPORT, axis, 1, 0);
    go_base(c);
}

static void idle_cmd(app_ctx_t *c, app_acts_t *o, app_cmd_t cmd, uint8_t axis, int32_t value)
{
    switch (cmd) {
    case APP_CMD_LIMIT_TOP:
    case APP_CMD_LIMIT_BOTTOM:
    case APP_CMD_LIMIT_SET:
    case APP_CMD_LIMIT_CLEAR:
        limit_cmd(c, o, cmd, axis, value);
        break;
    case APP_CMD_UP:
    case APP_CMD_DOWN:
    case APP_CMD_GOTO:
    case APP_CMD_JOG:
        start_cmd(c, o, cmd, axis, value);
        break;
    case APP_CMD_CAL_START:
        cal_begin(c, o, axis);
        break;
    case APP_CMD_MODE_WRITE:
        if (value & 0x01) note(o, APP_NOTE_MODE_REVERSE);
        if (value & 0x02) cal_begin(c, o, APP_AXIS_MASK_ALL);
        break;
    case APP_CMD_FACTORY_RESET:
        act(o, APP_ACT_STOP, 0, 0, 0);
        act(o, APP_ACT_FACTORY_RESET, 0, 0, 0);
        break;
    default:
        break;   /* STOP, ACK, CAL_SAVE, CAL_ABORT: nichts zu tun */
    }
}

static void idle_btn(app_ctx_t *c, app_acts_t *o, app_btn_t b)
{
    switch (b) {
    case APP_BTN_SHORT:
        /* Auf/Ab im Wechsel (Heben); nach dem Start zuerst "Auf" (oben gibt es einen Anschlag) */
        start_cmd(c, o, c->last_dir[APP_AXIS_BTN] < 0 ? APP_CMD_DOWN : APP_CMD_UP, APP_AXIS_BTN, 0);
        break;
    case APP_BTN_LONG:
        cal_begin(c, o, APP_AXIS_MASK_ALL);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */

void app_logic_init(app_ctx_t *c)
{
    memset(c, 0, sizeof(*c));
    c->st = APP_ST_BOOT;
    c->mv_target = -1;
    c->last_dir[APP_AXIS_LIFT] = c->last_dir[APP_AXIS_TILT] = 1;   /* erster Tastendruck = Auf */
}

static void step_boot(app_ctx_t *c, const app_in_t *in, app_acts_t *o)
{
    for (uint8_t i = 0; i < APP_AXIS_COUNT; i++) {
        c->nv_bottom[i] = c->bottom[i] = in->boot.bottom[i];
        c->calibrated[i] = in->boot.bottom[i] != 0;
        c->pos[i] = in->boot.pos[i];
        c->needs_homing[i] = c->calibrated[i] && in->boot.moving;
        act(o, APP_ACT_SET_POS, i, c->pos[i], 0);
        if (c->calibrated[i]) act(o, APP_ACT_SET_ENDPOINTS, i, c->bottom[i], 0);
    }
    c->lockout = in->boot.lockout;
    go_base(c);
    act(o, APP_ACT_REPORT, APP_AXIS_LIFT, 1, 0);
    act(o, APP_ACT_REPORT, APP_AXIS_TILT, 1, 0);
}

void app_logic_step(app_ctx_t *c, const app_in_t *in, app_acts_t *o)
{
    o->n = 0;
    o->note = APP_NOTE_NONE;

    if (c->st == APP_ST_BOOT) {
        if (in->type == APP_IN_BOOT) step_boot(c, in, o);
        else note(o, APP_NOTE_REJECTED_STATE);
        return;
    }

    switch (in->type) {
    case APP_IN_BOOT:
        note(o, APP_NOTE_IGNORED);
        return;

    case APP_IN_BATTERY:
        c->lockout = in->battery.lockout;
        if (c->lockout) {
            if (c->st == APP_ST_READY || c->st == APP_ST_UNCALIBRATED) {
                go_base(c);
            } else if (c->st == APP_ST_MOVING) {
                /* sofort anhalten; das Abschluss-Ereignis führt über go_base in AKKU_LEER */
                if (!c->stop_req) {
                    c->stop_req = true;
                    c->pending[0].valid = c->pending[1].valid = false;
                    act(o, APP_ACT_STOP, c->mv_axis, 0, 0);
                }
            } else if (c->st == APP_ST_CALIBRATING) {
                if (c->cal_moving) {
                    /* anhalten, nach dem Stopp abbrechen (kein weiterer Kalibrierschritt) */
                    c->cal_abort_req = true;
                    act(o, APP_ACT_STOP, c->cal_axis, 0, 0);
                } else {
                    cal_abort(c, o);   /* landet über go_base in AKKU_LEER */
                }
            }
            /* FEHLER: beim Quittieren. Die Sperre löst sich erst mit dem Ladeende (battery_calc). */
        } else if (c->st == APP_ST_BATTERY_EMPTY) {
            go_base(c);
        }
        return;

    case APP_IN_BTN: {
        app_btn_t b = in->btn;
        c->hint = b == APP_BTN_LONG_HINT;
        if (b == APP_BTN_LONG_HINT) return;
        if (b == APP_BTN_VERY_LONG) {
            act(o, APP_ACT_STOP, 0, 0, 0);
            act(o, APP_ACT_FACTORY_RESET, 0, 0, 0);
            return;
        }
        switch (c->st) {
        case APP_ST_READY:
        case APP_ST_UNCALIBRATED:
            idle_btn(c, o, b);
            break;
        case APP_ST_MOVING:
            if (b == APP_BTN_SHORT) moving_cmd(c, o, APP_CMD_STOP, 0, 0);
            break;
        case APP_ST_CALIBRATING:
            cal_btn(c, o, b);
            break;
        case APP_ST_ERROR:
            if (b == APP_BTN_SHORT) ack_error(c, o);
            break;
        default:
            break;
        }
        return;
    }

    case APP_IN_CMD: {
        app_cmd_t cmd = in->cmd.cmd;
        switch (c->st) {
        case APP_ST_READY:
        case APP_ST_UNCALIBRATED:
            idle_cmd(c, o, cmd, in->cmd.axis, in->cmd.value);
            break;
        case APP_ST_MOVING:
            moving_cmd(c, o, cmd, in->cmd.axis, in->cmd.value);
            break;
        case APP_ST_CALIBRATING:
            cal_cmd(c, o, cmd, in->cmd.value);
            break;
        case APP_ST_ERROR:
            if (cmd == APP_CMD_ACK || cmd == APP_CMD_STOP) ack_error(c, o);
            else if (cmd == APP_CMD_FACTORY_RESET) idle_cmd(c, o, cmd, 0, 0);
            else note(o, APP_NOTE_REJECTED_STATE);
            break;
        case APP_ST_BATTERY_EMPTY:
            if (cmd == APP_CMD_FACTORY_RESET) idle_cmd(c, o, cmd, 0, 0);
            else if (cmd != APP_CMD_STOP && cmd != APP_CMD_ACK) note(o, APP_NOTE_BATTERY_EMPTY);
            break;
        default:
            break;
        }
        return;
    }

    case APP_IN_MOTOR: {
        app_mev_t ev = in->motor.ev;
        uint8_t axis = in->motor.axis;
        if (!valid_axis(axis)) return;
        if (ev == APP_MEV_PROGRESS) {
            c->pos[axis] = in->motor.pos;
            act(o, APP_ACT_REPORT, axis, 0, 0);
            return;
        }
        if (ev == APP_MEV_FAULT) {
            /* nFAULT / Notaus: in jedem Zustand FEHLER (FEHLER bleibt FEHLER) */
            if (c->st != APP_ST_ERROR) enter_error(c, o, APP_ERR_FAULT, axis, in->motor.pos);
            else c->err = APP_ERR_FAULT;
            return;
        }
        if (c->st == APP_ST_MOVING) moving_motor(c, o, ev, axis, in->motor.pos);
        else if (c->st == APP_ST_CALIBRATING) cal_motor(c, o, ev, axis, in->motor.pos);
        else note(o, APP_NOTE_IGNORED);
        return;
    }
    }
}

bool app_logic_operational(const app_ctx_t *c)
{
    return (c->st == APP_ST_READY || c->st == APP_ST_MOVING) && all_calibrated(c);
}

bool app_logic_calibrating(const app_ctx_t *c)
{
    return c->st == APP_ST_CALIBRATING;
}

led_pattern_t app_logic_led(const app_ctx_t *c, bool zb_joining, bool identify)
{
    if (identify || c->hint) return LED_PAT_ON;
    if (c->st == APP_ST_ERROR) return LED_PAT_ERROR;
    if (c->st == APP_ST_CALIBRATING) return LED_PAT_CALIBRATION;
    if (zb_joining) return LED_PAT_PAIRING;
    return LED_PAT_OFF;
}

const char *app_logic_state_name(app_st_t st)
{
    switch (st) {
    case APP_ST_BOOT:          return "NEUSTART";
    case APP_ST_UNCALIBRATED:  return "UNKALIBRIERT";
    case APP_ST_CALIBRATING:   return "KALIBRIERUNG";
    case APP_ST_READY:         return "BEREIT";
    case APP_ST_MOVING:        return "FAHRT";
    case APP_ST_ERROR:         return "FEHLER";
    case APP_ST_BATTERY_EMPTY: return "AKKU_LEER";
    default:                   return "?";
    }
}

const char *app_logic_err_name(app_err_t e)
{
    switch (e) {
    case APP_ERR_NONE:      return "-";
    case APP_ERR_BLOCKED:   return "blockiert (Hindernis)";
    case APP_ERR_TIMEOUT:   return "Fahrzeit ueberschritten";
    case APP_ERR_WRONG_DIR: return "Richtungsfehler (Mx_DIR_INVERT/ENCx_INVERT pruefen)";
    case APP_ERR_FAULT:     return "nFAULT DRV8833 / Notaus";
    case APP_ERR_HOMING:    return "Referenzfahrt unplausibel (Hindernis)";
    case APP_ERR_CAL:       return "Kalibrierung gescheitert";
    default:                return "?";
    }
}

const char *app_logic_note_str(app_note_t n)
{
    switch (n) {
    case APP_NOTE_REJECTED_STATE: return "Befehl in diesem Zustand nicht erlaubt";
    case APP_NOTE_NOT_CALIBRATED: return "Achse nicht kalibriert: keine Prozentfahrt";
    case APP_NOTE_BATTERY_EMPTY:  return "Akku leer: keine Fahrt";
    case APP_NOTE_PENDING:        return "andere Achse faehrt: Befehl vorgemerkt";
    case APP_NOTE_HOMING:         return "Referenzfahrt oben";
    case APP_NOTE_CAL_TOO_SHORT:  return "Endpunkt unten zu nah an oben: nicht gespeichert";
    case APP_NOTE_CAL_AXIS_DONE:  return "Achse kalibriert, Endpunkte gespeichert";
    case APP_NOTE_CAL_ABORTED:    return "Kalibrierung abgebrochen";
    case APP_NOTE_CAL_JOG:        return "unten angehalten: Tippbetrieb (kurz), speichern (3 s / cal save)";
    case APP_NOTE_ACKED:          return "Fehler quittiert";
    case APP_NOTE_IGNORED:        return "ignoriert";
    case APP_NOTE_MODE_REVERSE:   return "Mode-Bit 0 (Richtung) nicht unterstuetzt: Mx_DIR_INVERT in config.h";
    case APP_NOTE_INVALID_VALUE:  return "ungueltiger Prozentwert (erlaubt 0..100): abgewiesen";
    case APP_NOTE_LIMIT_TOP:      return "oben gesetzt (Position 0); unten noch setzen (limit bottom)";
    case APP_NOTE_LIMIT_BOTTOM:   return "unten gesetzt, Achse kalibriert und gespeichert";
    case APP_NOTE_LIMIT_CLEARED:  return "Kalibrierung geloescht";
    default:                      return "";
    }
}

const char *app_logic_cal_name(app_cal_t s)
{
    switch (s) {
    case APP_CAL_UP:      return "hoch bis Anschlag";
    case APP_CAL_BACKOFF: return "zurueck";
    case APP_CAL_DOWN:    return "runter bis Stopp";
    case APP_CAL_JOG:     return "Tippbetrieb";
    default:              return "-";
    }
}
