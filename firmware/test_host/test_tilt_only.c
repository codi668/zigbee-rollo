/* ROLLO_TILT_ONLY = 1: nur Motor 2 (Wenden) angeschlossen, plus manuelle Endpunkte ("limit ...").
 * Eigenes Programm, weil das Makro für die ganze Übersetzungseinheit gilt (Makefile: -DROLLO_TILT_ONLY=1). */
#include "unity.h"
#include "app_logic.h"
#include "config.h"

#define L APP_AXIS_LIFT
#define T APP_AXIS_TILT

static app_ctx_t c;
static app_acts_t o;

void setUp(void) {}
void tearDown(void) {}

static void step(const app_in_t *in) { app_logic_step(&c, in, &o); }

static void boot(int32_t bt)
{
    app_logic_init(&c);
    app_in_t in = { .type = APP_IN_BOOT };
    in.boot.bottom[T] = bt;
    in.boot.pos[T] = bt ? 100 : 0;
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

static const app_act_t *get(app_act_type_t t, int axis)
{
    for (int i = 0; i < o.n; i++)
        if (o.a[i].type == t && (axis < 0 || o.a[i].axis == axis)) return &o.a[i];
    TEST_FAIL_MESSAGE("Aktion fehlt");
    return NULL;
}

static bool has(app_act_type_t t)
{
    for (int i = 0; i < o.n; i++)
        if (o.a[i].type == t) return true;
    return false;
}

static void test_ready_with_only_tilt_calibrated(void)
{
    boot(4200);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    boot(0);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
}

static void test_lift_commands_rejected(void)
{
    boot(4200);
    cmd(APP_CMD_GOTO, L, 50);
    TEST_ASSERT_EQUAL(APP_NOTE_REJECTED_STATE, o.note);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL(0, o.n);
    cmd(APP_CMD_DOWN, L, 0);
    cmd(APP_CMD_UP, L, 0);
    cmd(APP_CMD_LIMIT_SET, L, 1000);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL(0, o.n);
    cmd(APP_CMD_GOTO, T, 50);   /* Kipp-Regler fährt */
    TEST_ASSERT_EQUAL(T, get(APP_ACT_GOTO_PCT, -1)->axis);
    mev(APP_MEV_DONE, T, 2100);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
}

static void test_cal_all_is_tilt_only(void)
{
    boot(0);
    cmd(APP_CMD_CAL_START, APP_AXIS_MASK_ALL, 0);
    TEST_ASSERT_EQUAL(APP_ST_CALIBRATING, c.st);
    TEST_ASSERT_EQUAL(T, c.cal_axis);
    TEST_ASSERT_EQUAL(T, get(APP_ACT_RUN, -1)->axis);
}

static void test_limit_top_bottom(void)
{
    boot(0);
    cmd(APP_CMD_JOG, T, 600);
    mev(APP_MEV_DONE, T, 600);
    cmd(APP_CMD_LIMIT_TOP, T, 0);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, T)->a);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
    cmd(APP_CMD_JOG, T, 1500);
    mev(APP_MEV_DONE, T, 1500);
    cmd(APP_CMD_LIMIT_BOTTOM, T, 0);
    TEST_ASSERT_EQUAL_INT32(1500, get(APP_ACT_SET_ENDPOINTS, T)->a);
    TEST_ASSERT_EQUAL_INT32(1500, get(APP_ACT_NV_SAVE_CAL, T)->a);
    TEST_ASSERT_TRUE(c.calibrated[T]);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    TEST_ASSERT_EQUAL(APP_NOTE_LIMIT_BOTTOM, o.note);
}

static void test_limit_bottom_too_short_rejected(void)
{
    boot(0);
    cmd(APP_CMD_LIMIT_BOTTOM, T, 0);   /* Position 0: kein Fahrweg */
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_TOO_SHORT, o.note);
    TEST_ASSERT_FALSE(c.calibrated[T]);
    TEST_ASSERT_FALSE(has(APP_ACT_NV_SAVE_CAL));
}

static void test_limit_set_and_clear(void)
{
    boot(0);
    cmd(APP_CMD_LIMIT_SET, T, 1050);
    TEST_ASSERT_EQUAL_INT32(0, get(APP_ACT_SET_POS, T)->a);
    TEST_ASSERT_EQUAL_INT32(1050, get(APP_ACT_NV_SAVE_CAL, T)->a);
    TEST_ASSERT_EQUAL(APP_ST_READY, c.st);
    cmd(APP_CMD_LIMIT_SET, T, 10);
    TEST_ASSERT_EQUAL(APP_NOTE_CAL_TOO_SHORT, o.note);
    TEST_ASSERT_TRUE(c.calibrated[T]);
    cmd(APP_CMD_LIMIT_CLEAR, T, 0);
    TEST_ASSERT_TRUE(has(APP_ACT_NV_CLEAR_CAL));
    TEST_ASSERT_FALSE(c.calibrated[T]);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
}

static void test_limit_top_invalidates_calibration(void)
{
    boot(4200);
    cmd(APP_CMD_LIMIT_TOP, T, 0);
    TEST_ASSERT_TRUE(has(APP_ACT_NV_CLEAR_CAL));
    TEST_ASSERT_FALSE(c.calibrated[T]);
    TEST_ASSERT_EQUAL(APP_ST_UNCALIBRATED, c.st);
}

static void test_limit_rejected_while_moving(void)
{
    boot(4200);
    cmd(APP_CMD_GOTO, T, 50);
    cmd(APP_CMD_LIMIT_TOP, T, 0);
    TEST_ASSERT_EQUAL(APP_NOTE_REJECTED_STATE, o.note);
    TEST_ASSERT_EQUAL(APP_ST_MOVING, c.st);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ready_with_only_tilt_calibrated);
    RUN_TEST(test_lift_commands_rejected);
    RUN_TEST(test_cal_all_is_tilt_only);
    RUN_TEST(test_limit_top_bottom);
    RUN_TEST(test_limit_bottom_too_short_rejected);
    RUN_TEST(test_limit_set_and_clear);
    RUN_TEST(test_limit_top_invalidates_calibration);
    RUN_TEST(test_limit_rejected_while_moving);
    return UNITY_END();
}
