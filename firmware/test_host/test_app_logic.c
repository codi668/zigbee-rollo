/* Zustandsautomat (main/app_logic.c): Übergänge, Kalibrierablauf, moving-Flag, Plausibilität */
#include <string.h>
#include "unity.h"
#include "tests.h"
#include "app_logic.h"
#include "config.h"

#define L APP_AXIS_LIFT
#define T APP_AXIS_TILT

static app_ctx_t c;
static app_acts_t o;

/* ---- Hilfen ---- */

static bool is_motion(app_act_type_t t)
{
    return t == APP_ACT_RUN || t == APP_ACT_GOTO_PCT || t == APP_ACT_MOVE_BY;
}

/** Motorbefehle müssen am Ende der Liste stehen (app_state bricht dort bei Fehlern ab) */
static void check_motion_last(void)
{
    int first = -1;
    for (int i = 0; i < o.n; i++)
        if (is_motion(o.a[i].type)) { first = i; break; }
    if (first < 0) return;
    for (int i = first; i < o.n; i++) TEST_ASSERT_TRUE_MESSAGE(is_motion(o.a[i].type), "Motorbefehl nicht am Ende");
}

static void step(const app_in_t *in)
{
    app_logic_step(&c, in, &o);
    check_motion_last();
}

static void boot(int32_t bl, int32_t bt, bool moving, bool lockout)
{
    app_logic_init(&c);
    app_in_t in = { .type = APP_IN_BOOT };
    in.boot.bottom[L] = bl;
    in.boot.bottom[T] = bt;
    in.boot.pos[L] = bl ? 100 : 0;
    in.boot.pos[T] = bt ? 200 : 0;
    in.boot.moving = moving;
    in.boot.lockout = lockout;
    step(&in);
}

static void btn(app_btn_t b)
{
    app_in_t in = { .type = APP_IN_BTN, .btn = b };
    step(&in);
}

static void cmd(app_cmd_t cm, uint8_t axis, int32_t value)
{
    app_in_t in = { .type = APP_IN_CMD };
    in.cmd.cmd = cm;
    in.cmd.axis = axis;
    in.cmd.value = value;
    step(&in);
}

static void mev(app_mev_t ev, uint8_t axis, int32_t pos)
{
    app_in_t in = { .type = APP_IN_MOTOR };
    in.motor.ev = ev;
    in.motor.axis = axis;
    in.motor.pos = pos;
    step(&in);
}

static void batt(bool lockout)
{
    app_in_t in = { .type = APP_IN_BATTERY };
    in.battery.lockout = lockout;
    step(&in);
}

/** Index der ersten Aktion vom Typ t für die Achse (axis < 0: beliebig), sonst -1 */
static int find(app_act_type_t t, int axis)
{
    for (int i = 0; i < o.n; i++)
        if (o.a[i].type == t && (axis < 0 || o.a[i].axis == axis)) return i;
    return -1;
}

static const app_act_t *get(app_act_type_t t, int axis)
{
    int i = find(t, axis);
    TEST_ASSERT_TRUE_MESSAGE(i >= 0, "Aktion fehlt");
    return &o.a[i];
}

static bool any_motion(void)
{
    for (int i = 0; i < o.n; i++)
        if (is_motion(o.a[i].type)) return true;
    return false;
}

/* ---- NEUSTART ---- */

static void test_boot_uncalibrated(void)
{
    boot(0, 0, false, false);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    TEST_ASSERT_TRUE(find(APP_ACT_SET_POS, L) >= 0);
    TEST_ASSERT_TRUE(find(APP_ACT_SET_POS, T) >= 0);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_SET_ENDPOINTS, -1));
    TEST_ASSERT_FALSE(any_motion());   /* nach dem Start keine Fahrt */
    TEST_ASSERT_FALSE(app_logic_operational(&c));
}

static void test_boot_calibrated_ready(void)
{
    boot(10000, 4000, false, false);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL_INT32(10000, get(APP_ACT_SET_ENDPOINTS, L)->a);
    TEST_ASSERT_EQUAL_INT32(4000, get(APP_ACT_SET_ENDPOINTS, T)->a);
    TEST_ASSERT_EQUAL_INT32(100, get(APP_ACT_SET_POS, L)->a);
    TEST_ASSERT_FALSE(c.needs_homing[L]);
    TEST_ASSERT_FALSE(any_motion());
    TEST_ASSERT_TRUE(app_logic_operational(&c));
}

static void test_boot_lockout(void)
{
    boot(10000, 4000, false, true);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    cmd(APP_CMD_GOTO, T, 50);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_EQUAL(0, o.n);
    TEST_ASSERT_EQUAL(APP_NOTE_BATTERY_EMPTY, o.note);
    btn(APP_BTN_SHORT);
    TEST_ASSERT_FALSE(any_motion());
    batt(false);   /* geladen */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
}

static void test_boot_state_rejects_commands(void)
{
    app_logic_init(&c);
    cmd(APP_CMD_UP, L, 0);
    TEST_ASSERT_EQUAL(APP_ST_BOOT, c.st);
    TEST_ASSERT_EQUAL(0, o.n);
    TEST_ASSERT_EQUAL(APP_NOTE_REJECTED_STATE, o.note);
}

/* ---- Fahrten kalibriert ---- */

static void test_goto_done(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 30);
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_BEGIN, -1) >= 0);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_BEGIN, -1) < find(APP_ACT_GOTO_PCT, T));
    TEST_ASSERT_EQUAL_INT32(30, get(APP_ACT_GOTO_PCT, T)->a);
    mev(APP_MEV_PROGRESS, T, 600);
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_REPORT, T)->a);   /* Zwischenstand */
    mev(APP_MEV_DONE, T, 1234);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL_INT32(1234, get(APP_ACT_NV_MOTION_END, T)->a);
    TEST_ASSERT_EQUAL_INT32(1, get(APP_ACT_REPORT, T)->a);   /* Endwert sofort */
    TEST_ASSERT_TRUE(find(APP_ACT_MEASURE, -1) >= 0);        /* Akku nach der Fahrt */
}

static void test_goto_invalid_rejected_and_replaced(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 250);          /* außerhalb 0…100: abweisen, nicht auf 100 begrenzen */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL(APP_NOTE_INVALID_VALUE, o.note);
    TEST_ASSERT_EQUAL(0, o.n);
    cmd(APP_CMD_GOTO, L, 0xFF);         /* Zigbee "unbekannt" */
    TEST_ASSERT_EQUAL(APP_NOTE_INVALID_VALUE, o.note);
    TEST_ASSERT_FALSE(any_motion());
    cmd(APP_CMD_GOTO, T, -1);
    TEST_ASSERT_EQUAL(APP_NOTE_INVALID_VALUE, o.note);
    cmd(APP_CMD_GOTO, T, 100);
    TEST_ASSERT_EQUAL_INT32(100, get(APP_ACT_GOTO_PCT, T)->a);
    cmd(APP_CMD_GOTO, T, 0xFF);         /* während der Fahrt: Ziel bleibt */
    TEST_ASSERT_EQUAL(APP_NOTE_INVALID_VALUE, o.note);
    TEST_ASSERT_EQUAL(0, o.n);
    cmd(APP_CMD_GOTO, L, 101);          /* andere Achse: nicht vormerken */
    TEST_ASSERT_EQUAL(APP_NOTE_INVALID_VALUE, o.note);
    TEST_ASSERT_FALSE(c.pending[L].valid);
    cmd(APP_CMD_GOTO, T, 20);   /* neues Ziel während der Fahrt */
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_EQUAL_INT32(20, get(APP_ACT_GOTO_PCT, T)->a);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_MOTION_BEGIN, -1));
}

static void test_open_rehomes_lift(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_UP, L, 0);   /* LIFT_REHOME_ON_OPEN = 1 */
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
    TEST_ASSERT_TRUE(find(APP_ACT_CLEAR_CAL, L) >= 0);
    const app_act_t *r = get(APP_ACT_RUN, L);
    TEST_ASSERT_EQUAL_INT32(-1, r->a);
    TEST_ASSERT_EQUAL_INT32(MOTOR_TIMEOUT_MAX_MS, r->b);
    mev(APP_MEV_BLOCKED, L, 20);   /* Anschlag knapp über der alten 0 */
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, L)->a);
    TEST_ASSERT_EQUAL_INT32(10000, get(APP_ACT_SET_ENDPOINTS, L)->a);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_GOTO_PCT, L)->a);   /* weiter auf 0 % (mit Rand) */
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    mev(APP_MEV_DONE, L, 150);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_END, L) >= 0);
}

static void test_tilt_open_no_rehome(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_UP, T, 0);   /* TILT_REHOME_ON_OPEN = 0 */
    TEST_ASSERT_EQUAL(APP_MV_GOTO, c.mv_kind);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_GOTO_PCT, T)->a);
}

static void test_moving_flag_homing_after_reboot(void)
{
    boot(10000, 4000, true, false);   /* Neustart während einer Fahrt */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_TRUE(c.needs_homing[L]);
    TEST_ASSERT_TRUE(c.needs_homing[T]);
    cmd(APP_CMD_GOTO, T, 50);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
    TEST_ASSERT_EQUAL(APP_NOTE_HOMING, o.note);
    mev(APP_MEV_BLOCKED, T, 3000);   /* Position war unbekannt: jeder Anschlag gilt */
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_FALSE(c.needs_homing[T]);
    TEST_ASSERT_EQUAL_INT32(50, get(APP_ACT_GOTO_PCT, T)->a);
    mev(APP_MEV_DONE, T, 2000);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    /* Heben muss noch referenziert werden -> moving bleibt 1 */
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_MOTION_END, -1));
    TEST_ASSERT_EQUAL_INT32(2000, get(APP_ACT_NV_SAVE_POS, T)->a);
}

static void test_homing_obstacle_is_error(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, L, 0);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
    mev(APP_MEV_BLOCKED, L, 3000);   /* weit unter 0: Hindernis */
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL(APP_ERR_HOMING, c.err);
    TEST_ASSERT_EQUAL_INT32(10000, get(APP_ACT_SET_ENDPOINTS, L)->a);   /* Endpunkte zurück */
    TEST_ASSERT_TRUE(c.calibrated[L]);
    TEST_ASSERT_TRUE(c.needs_homing[L]);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_SAVE_POS, L) >= 0);   /* moving bleibt 1 */
}

static void test_homing_stopped_restores(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_UP, L, 0);
    cmd(APP_CMD_STOP, 0, 0);
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    mev(APP_MEV_STOPPED, L, 900);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL_INT32(10000, get(APP_ACT_SET_ENDPOINTS, L)->a);
    TEST_ASSERT_FALSE(c.needs_homing[L]);
}

static void test_blocked_is_error_and_ack(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 80);
    mev(APP_MEV_BLOCKED, T, 1500);
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL(APP_ERR_BLOCKED, c.err);
    TEST_ASSERT_TRUE(c.needs_homing[T]);   /* nach Quittieren oben neu referenzieren */
    TEST_ASSERT_EQUAL(LED_PAT_ERROR, app_logic_led(&c, false, false));
    TEST_ASSERT_FALSE(app_logic_operational(&c));
    cmd(APP_CMD_GOTO, T, 10);              /* im FEHLER keine Fahrt */
    TEST_ASSERT_FALSE(any_motion());
    TEST_ASSERT_EQUAL(APP_NOTE_REJECTED_STATE, o.note);
    btn(APP_BTN_SHORT);                    /* quittieren */
    TEST_ASSERT_TRUE(find(APP_ACT_CLEAR_FAULT, -1) >= 0);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    cmd(APP_CMD_GOTO, T, 10);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
}

static void test_timeout_and_zigbee_stop_acks(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_DOWN, L, 0);
    mev(APP_MEV_TIMEOUT, L, 7000);
    TEST_ASSERT_EQUAL(APP_ERR_TIMEOUT, c.err);
    cmd(APP_CMD_STOP, 0, 0);   /* Zigbee Stop quittiert */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_TRUE(find(APP_ACT_CLEAR_FAULT, -1) >= 0);
}

static void test_fault_any_state(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    mev(APP_MEV_FAULT, T, 999);
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL(APP_ERR_FAULT, c.err);
    TEST_ASSERT_TRUE(c.needs_homing[T]);    /* Achse trudelte aus / Überstrom: neu referenzieren */
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_MOTION_END, -1));
    TEST_ASSERT_EQUAL_INT32(999, get(APP_ACT_NV_SAVE_POS, T)->a);   /* moving bleibt 1 */
    btn(APP_BTN_SHORT);                     /* quittieren */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    cmd(APP_CMD_GOTO, T, 30);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
    /* auch aus dem Ruhezustand */
    boot(0, 0, false, false);
    mev(APP_MEV_FAULT, L, 0);
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
}

static void test_rejected_returns_to_ready(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    mev(APP_MEV_REJECTED, T, 0);   /* von app_state bei Fehler eingespeist */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_END, T) >= 0);
}

static void test_pending_other_axis(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    cmd(APP_CMD_GOTO, L, 40);   /* MOTOR_ALLOW_PARALLEL = 0 */
    TEST_ASSERT_EQUAL(APP_NOTE_PENDING, o.note);
    TEST_ASSERT_FALSE(any_motion());
    mev(APP_MEV_DONE, T, 2000);
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    TEST_ASSERT_EQUAL(L, c.mv_axis);
    TEST_ASSERT_EQUAL_INT32(40, get(APP_ACT_GOTO_PCT, L)->a);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_END, T) < find(APP_ACT_NV_MOTION_BEGIN, L));
}

static void test_stop_clears_pending(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    cmd(APP_CMD_GOTO, L, 40);
    cmd(APP_CMD_STOP, 0, 0);
    mev(APP_MEV_STOPPED, T, 1000);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_FALSE(any_motion());
}

static void test_lockout_during_motion(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    cmd(APP_CMD_GOTO, L, 40);      /* vorgemerkt */
    batt(true);                    /* Sperre während der Fahrt: sofort anhalten */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    TEST_ASSERT_FALSE(any_motion());
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);   /* bis das Abschluss-Ereignis kommt */
    cmd(APP_CMD_GOTO, T, 10);      /* neues Ziel wird nicht mehr angenommen */
    TEST_ASSERT_FALSE(any_motion());
    mev(APP_MEV_STOPPED, T, 2000);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_FALSE(any_motion());          /* Vormerkung verworfen */
    TEST_ASSERT_EQUAL_INT32(2000, get(APP_ACT_NV_MOTION_END, T)->a);
    batt(true);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    batt(false);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    batt(true);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);   /* BEREIT -> AKKU_LEER */
}

static void test_lockout_during_homing(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, L, 0);       /* Referenzfahrt, danach 0 % */
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
    batt(true);
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    mev(APP_MEV_BLOCKED, L, 10);   /* Anschlag erreicht, bevor der Stopp griff */
    TEST_ASSERT_FALSE(any_motion());   /* keine Weiterfahrt zum Prozentziel */
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_FALSE(c.needs_homing[L]);
}

/* ---- unkalibriert ---- */

static void test_uncalibrated_run(void)
{
    boot(0, 0, false, false);
    cmd(APP_CMD_GOTO, L, 50);
    TEST_ASSERT_EQUAL(APP_NOTE_NOT_CALIBRATED, o.note);
    TEST_ASSERT_FALSE(any_motion());
    btn(APP_BTN_SHORT);            /* erster Druck: Auf */
    TEST_ASSERT_EQUAL_INT32(-1, get(APP_ACT_RUN, L)->a);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_RUN, L)->b);   /* Standard-Timeout */
    mev(APP_MEV_BLOCKED, L, -400); /* oben am Anschlag: kein Fehler */
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, L)->a);
    btn(APP_BTN_SHORT);            /* Wechsel: Ab */
    TEST_ASSERT_EQUAL_INT32(1, get(APP_ACT_RUN, L)->a);
    btn(APP_BTN_SHORT);            /* während der Fahrt: Stopp */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    mev(APP_MEV_STOPPED, L, 3000);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    cmd(APP_CMD_DOWN, L, 0);
    mev(APP_MEV_BLOCKED, L, 3100); /* unten blockiert = Hindernis */
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL(APP_ERR_BLOCKED, c.err);
}

static void test_uncalibrated_jog_then_up_is_run(void)
{
    boot(0, 0, false, false);
    cmd(APP_CMD_JOG, L, 300);
    TEST_ASSERT_EQUAL(APP_MV_JOG, c.mv_kind);
    cmd(APP_CMD_GOTO, L, 50);      /* unkalibriert: keine Prozentfahrt */
    TEST_ASSERT_EQUAL(APP_NOTE_NOT_CALIBRATED, o.note);
    TEST_ASSERT_FALSE(any_motion());
    TEST_ASSERT_EQUAL(APP_MV_JOG, c.mv_kind);
    cmd(APP_CMD_UP, L, 0);         /* Auf während des Tippens -> RUN, nicht GOTO_PCT */
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_GOTO_PCT, -1));
    TEST_ASSERT_EQUAL_INT32(-1, get(APP_ACT_RUN, L)->a);
    TEST_ASSERT_EQUAL(APP_MV_RUN, c.mv_kind);
    mev(APP_MEV_BLOCKED, L, -40);  /* oben am Anschlag */
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, L)->a);
    /* kalibrierte Achse beim Tippen: weiterhin Prozentfahrt */
    boot(10000, 4000, false, false);
    cmd(APP_CMD_JOG, T, 100);
    cmd(APP_CMD_DOWN, T, 0);
    TEST_ASSERT_EQUAL_INT32(100, get(APP_ACT_GOTO_PCT, T)->a);
}

static void test_partial_calibration_allows_tilt_percent(void)
{
    boot(0, 4000, false, false);   /* Motor 1 noch nicht angeschlossen, Wenden kalibriert */
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    cmd(APP_CMD_GOTO, T, 25);
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
    mev(APP_MEV_DONE, T, 1000);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
}

/* ---- Kalibrierung ---- */

static void cal_lift_to_jog(int32_t stop_pos)
{
    btn(APP_BTN_LONG);   /* 3 s */
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    TEST_ASSERT_EQUAL(L, c.cal_axis);
    TEST_ASSERT_EQUAL(LED_PAT_CALIBRATION, app_logic_led(&c, true, false));
    TEST_ASSERT_TRUE(find(APP_ACT_CLEAR_CAL, L) >= 0);
    TEST_ASSERT_EQUAL_INT32(-1, get(APP_ACT_RUN, L)->a);
    mev(APP_MEV_BLOCKED, L, -12);              /* Anschlag oben */
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, L)->a);
    TEST_ASSERT_EQUAL_INT32(CAL_BACKOFF_STEPS_LIFT, get(APP_ACT_MOVE_BY, L)->a);
    TEST_ASSERT_EQUAL(APP_CAL_BACKOFF, c.cal_step);
    mev(APP_MEV_DONE, L, CAL_BACKOFF_STEPS_LIFT);
    TEST_ASSERT_EQUAL_INT32(1, get(APP_ACT_RUN, L)->a);
    TEST_ASSERT_EQUAL(APP_CAL_DOWN, c.cal_step);
    btn(APP_BTN_SHORT);                        /* unten anhalten */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    mev(APP_MEV_STOPPED, L, stop_pos);
    TEST_ASSERT_EQUAL(APP_CAL_JOG, c.cal_step);
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_JOG, o.note);
}

static void test_calibration_full(void)
{
    boot(0, 0, false, false);
    cal_lift_to_jog(5000);
    btn(APP_BTN_SHORT);                        /* Tippbetrieb */
    TEST_ASSERT_EQUAL_INT32(CAL_JOG_STEPS_LIFT, get(APP_ACT_MOVE_BY, L)->a);
    mev(APP_MEV_DONE, L, 5000 + CAL_JOG_STEPS_LIFT);
    btn(APP_BTN_LONG);                         /* untere Endlage speichern */
    TEST_ASSERT_EQUAL_INT32(5000 + CAL_JOG_STEPS_LIFT, get(APP_ACT_SET_ENDPOINTS, L)->a);
    TEST_ASSERT_EQUAL_INT32(5000 + CAL_JOG_STEPS_LIFT, get(APP_ACT_NV_SAVE_CAL, L)->a);
    TEST_ASSERT_TRUE(find(APP_ACT_NV_MOTION_END, L) >= 0);
    TEST_ASSERT_TRUE(c.calibrated[L]);
    /* weiter mit Wenden */
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    TEST_ASSERT_EQUAL(T, c.cal_axis);
    TEST_ASSERT_EQUAL_INT32(-1, get(APP_ACT_RUN, T)->a);
    mev(APP_MEV_BLOCKED, T, 0);
    mev(APP_MEV_DONE, T, CAL_BACKOFF_STEPS_TILT);
    cmd(APP_CMD_STOP, 0, 0);                   /* Zigbee Stop */
    mev(APP_MEV_STOPPED, T, 3000);
    cmd(APP_CMD_MODE_WRITE, 0, 0);             /* Mode-Bit 1 löschen = speichern */
    TEST_ASSERT_EQUAL_INT32(3000, get(APP_ACT_NV_SAVE_CAL, T)->a);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_TRUE(app_logic_operational(&c));
}

static void test_calibration_save_while_moving_down(void)
{
    boot(0, 0, false, false);
    cmd(APP_CMD_CAL_START, 1 << L, 0);
    mev(APP_MEV_BLOCKED, L, 0);
    mev(APP_MEV_DONE, L, CAL_BACKOFF_STEPS_LIFT);
    btn(APP_BTN_LONG);                         /* speichern während der Fahrt nach unten */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_SAVE_CAL, -1));
    mev(APP_MEV_STOPPED, L, 6000);
    TEST_ASSERT_EQUAL_INT32(6000, get(APP_ACT_NV_SAVE_CAL, L)->a);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);   /* nur Heben kalibriert */
}

static void test_calibration_too_short(void)
{
    boot(0, 0, false, false);
    cal_lift_to_jog(CAL_MIN_TRAVEL_STEPS_LIFT - 1);
    btn(APP_BTN_LONG);
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_TOO_SHORT, o.note);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_SAVE_CAL, -1));
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    TEST_ASSERT_EQUAL(APP_CAL_JOG, c.cal_step);
}

static void test_calibration_motor_missing(void)
{
    /* Motor 1 nicht angeschlossen: "Anschlag" sofort (nicht unterscheidbar), dann
     * läuft auch das Zurückfahren nicht an -> FEHLER statt falscher Kalibrierung */
    boot(0, 0, false, false);
    btn(APP_BTN_LONG);
    mev(APP_MEV_BLOCKED, L, 0);
    mev(APP_MEV_BLOCKED, L, 0);
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL(APP_ERR_CAL, c.err);
    TEST_ASSERT_FALSE(c.calibrated[L]);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_SAVE_CAL, -1));
}

static void test_calibration_down_blocked_is_error(void)
{
    boot(0, 0, false, false);
    btn(APP_BTN_LONG);
    mev(APP_MEV_BLOCKED, L, 0);
    mev(APP_MEV_DONE, L, CAL_BACKOFF_STEPS_LIFT);
    mev(APP_MEV_BLOCKED, L, 2000);   /* Heben hat unten keinen Anschlag */
    TEST_ASSERT_EQUAL(APP_ERR_CAL, c.err);
}

static void test_calibration_abort_restores_old(void)
{
    boot(8000, 4000, false, false);
    cmd(APP_CMD_CAL_START, 1 << L, 0);
    TEST_ASSERT_FALSE(c.calibrated[L]);
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    mev(APP_MEV_BLOCKED, L, 30);
    cmd(APP_CMD_CAL_ABORT, 0, 0);    /* während des Zurückfahrens */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    mev(APP_MEV_STOPPED, L, 20);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL_INT32(8000, get(APP_ACT_SET_ENDPOINTS, L)->a);
    TEST_ASSERT_TRUE(c.calibrated[L]);
    TEST_ASSERT_TRUE(c.needs_homing[L]);   /* Bezug oben unsicher -> neu referenzieren */
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_MOTION_END, -1));
    TEST_ASSERT_TRUE(find(APP_ACT_NV_SAVE_POS, L) >= 0);   /* moving bleibt 1 */
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_ABORTED, o.note);
    cmd(APP_CMD_GOTO, L, 50);
    TEST_ASSERT_EQUAL(APP_MV_HOME, c.mv_kind);
}

static void test_calibration_error_restores_old(void)
{
    boot(8000, 4000, false, false);
    cmd(APP_CMD_CAL_START, 1 << T, 0);
    mev(APP_MEV_TIMEOUT, T, -9000);  /* kein Anschlag oben gefunden */
    TEST_ASSERT_EQUAL(APP_ST_ERROR, c.st);
    TEST_ASSERT_EQUAL_INT32(4000, get(APP_ACT_SET_ENDPOINTS, T)->a);
    TEST_ASSERT_TRUE(c.calibrated[T]);
    TEST_ASSERT_TRUE(c.needs_homing[T]);
}

static void test_mode_write_starts_calibration(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_MODE_WRITE, 0, 0x02);
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    TEST_ASSERT_TRUE(app_logic_calibrating(&c));
    cmd(APP_CMD_MODE_WRITE, 0, 0x00);   /* noch oben: Abbruch */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    mev(APP_MEV_STOPPED, L, -50);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    cmd(APP_CMD_MODE_WRITE, 0, 0x01);   /* Richtungsbit: nicht unterstützt */
    TEST_ASSERT_EQUAL(APP_NOTE_MODE_REVERSE, o.note);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
}

static void test_calibration_rejected_when_lockout(void)
{
    boot(0, 0, false, true);
    btn(APP_BTN_LONG);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_FALSE(any_motion());
}

static void test_lockout_aborts_idle_calibration(void)
{
    boot(0, 0, false, false);
    cal_lift_to_jog(4000);
    batt(true);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_ABORTED, o.note);
}

static void test_lockout_stops_moving_calibration(void)
{
    boot(8000, 4000, false, false);
    btn(APP_BTN_LONG);
    TEST_ASSERT_TRUE(c.cal_moving);
    batt(true);                              /* während "hoch bis Anschlag" */
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    TEST_ASSERT_FALSE(any_motion());
    mev(APP_MEV_BLOCKED, L, 0);              /* Anschlag kam vor dem Stopp: nicht weiterfahren */
    TEST_ASSERT_FALSE(any_motion());
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_ABORTED, o.note);
    TEST_ASSERT_TRUE(c.calibrated[L]);       /* alte Endpunkte zurück */
    TEST_ASSERT_TRUE(c.needs_homing[L]);
    TEST_ASSERT_EQUAL(0, c.cal_mask);        /* Wenden wird nicht mehr kalibriert */
    btn(APP_BTN_SHORT);
    TEST_ASSERT_FALSE(any_motion());
    batt(false);                             /* Ladeende: Sperre aufgehoben */
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
}

static void test_lockout_during_cal_down(void)
{
    boot(0, 0, false, false);
    btn(APP_BTN_LONG);
    mev(APP_MEV_BLOCKED, L, 0);
    mev(APP_MEV_DONE, L, CAL_BACKOFF_STEPS_LIFT);
    TEST_ASSERT_EQUAL(APP_CAL_DOWN, c.cal_step);
    btn(APP_BTN_LONG);                       /* speichern angefordert ... */
    batt(true);                              /* ... aber Sperre: abbrechen statt speichern */
    mev(APP_MEV_STOPPED, L, 5000);
    TEST_ASSERT_EQUAL(-1, find(APP_ACT_NV_SAVE_CAL, -1));
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_FALSE(c.calibrated[L]);
}

/* ---- Sonstiges ---- */

static void test_factory_reset_any_state(void)
{
    boot(10000, 4000, false, false);
    cmd(APP_CMD_GOTO, T, 50);
    btn(APP_BTN_VERY_LONG);
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) >= 0);
    TEST_ASSERT_TRUE(find(APP_ACT_STOP, -1) < find(APP_ACT_FACTORY_RESET, -1));
    boot(0, 0, false, true);
    cmd(APP_CMD_FACTORY_RESET, 0, 0);
    TEST_ASSERT_TRUE(find(APP_ACT_FACTORY_RESET, -1) >= 0);
}

static void test_led_priorities(void)
{
    boot(10000, 4000, false, false);
    TEST_ASSERT_EQUAL(LED_PAT_OFF, app_logic_led(&c, false, false));
    TEST_ASSERT_EQUAL(LED_PAT_PAIRING, app_logic_led(&c, true, false));
    TEST_ASSERT_EQUAL(LED_PAT_ON, app_logic_led(&c, true, true));   /* Identify */
    btn(APP_BTN_LONG_HINT);
    TEST_ASSERT_EQUAL(LED_PAT_ON, app_logic_led(&c, false, false));
    TEST_ASSERT_FALSE(any_motion());
    btn(APP_BTN_LONG);   /* Loslassen: Hinweis aus, Kalibrierung */
    TEST_ASSERT_EQUAL(LED_PAT_CALIBRATION, app_logic_led(&c, false, false));
    boot(10000, 4000, false, true);
    TEST_ASSERT_EQUAL(APP_ST_BATTERY_EMPTY, c.st);
    TEST_ASSERT_EQUAL(LED_PAT_OFF, app_logic_led(&c, false, false));   /* AKKU_LEER: LED aus (Strom) */
}

static void test_names(void)
{
    TEST_ASSERT_EQUAL_STRING("NEUSTART", app_logic_state_name(APP_ST_BOOT));
    TEST_ASSERT_EQUAL_STRING("AKKU_LEER", app_logic_state_name(APP_ST_BATTERY_EMPTY));
    TEST_ASSERT_EQUAL_STRING("?", app_logic_state_name(APP_ST_COUNT));
}

void run_app_logic_tests(void)
{
    RUN_TEST(test_boot_uncalibrated);
    RUN_TEST(test_boot_calibrated_ready);
    RUN_TEST(test_boot_lockout);
    RUN_TEST(test_boot_state_rejects_commands);
    RUN_TEST(test_goto_done);
    RUN_TEST(test_goto_invalid_rejected_and_replaced);
    RUN_TEST(test_open_rehomes_lift);
    RUN_TEST(test_tilt_open_no_rehome);
    RUN_TEST(test_moving_flag_homing_after_reboot);
    RUN_TEST(test_homing_obstacle_is_error);
    RUN_TEST(test_homing_stopped_restores);
    RUN_TEST(test_blocked_is_error_and_ack);
    RUN_TEST(test_timeout_and_zigbee_stop_acks);
    RUN_TEST(test_fault_any_state);
    RUN_TEST(test_rejected_returns_to_ready);
    RUN_TEST(test_pending_other_axis);
    RUN_TEST(test_stop_clears_pending);
    RUN_TEST(test_lockout_during_motion);
    RUN_TEST(test_lockout_during_homing);
    RUN_TEST(test_uncalibrated_jog_then_up_is_run);
    RUN_TEST(test_uncalibrated_run);
    RUN_TEST(test_partial_calibration_allows_tilt_percent);
    RUN_TEST(test_calibration_full);
    RUN_TEST(test_calibration_save_while_moving_down);
    RUN_TEST(test_calibration_too_short);
    RUN_TEST(test_calibration_motor_missing);
    RUN_TEST(test_calibration_down_blocked_is_error);
    RUN_TEST(test_calibration_abort_restores_old);
    RUN_TEST(test_calibration_error_restores_old);
    RUN_TEST(test_mode_write_starts_calibration);
    RUN_TEST(test_calibration_rejected_when_lockout);
    RUN_TEST(test_lockout_aborts_idle_calibration);
    RUN_TEST(test_lockout_stops_moving_calibration);
    RUN_TEST(test_lockout_during_cal_down);
    RUN_TEST(test_factory_reset_any_state);
    RUN_TEST(test_led_priorities);
    RUN_TEST(test_names);
}
