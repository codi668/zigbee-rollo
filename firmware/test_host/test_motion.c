/*
 * Fahrprofil (motion.c) gegen ein einfaches Motormodell:
 * dreht nur ab min_duty (wie der N20 ab 70 %), Schritte pro 10-ms-Takt ~ Duty.
 */
#include <stdlib.h>
#include "unity.h"
#include "tests.h"
#include "motion.h"

#define TICK 10

static const motion_params_t P = {
    .min_duty = 700, .kick_duty = 1000, .run_duty = 800,
    .kick_ms = 300, .kick_steps = 20,
    .ramp_up_ms = 300, .ramp_down_ms = 200,
    .brake_zone_steps = 200, .stop_tol_steps = 10,
    .brake_ms = 50, .dir_pause_ms = 100,
    .block_window_ms = 200, .block_min_steps = 20,
    .wrong_dir_steps = 100, .timeout_max_ms = 60000,
};

typedef struct {
    int32_t pos;
    bool    stuck;       /* Welle festgehalten */
    int     sign;        /* +1 normal, -1 = Encoder/Motor falsch herum */
    uint16_t min_turn;   /* darunter steht der Motor */
    /* Statistik */
    uint16_t max_duty, min_drive_duty;
    int     drive_ticks, brake_ticks;
    uint32_t first_drive_t, last_drive_t;
    bool    reversed_without_brake;
    int8_t  last_drive_dir;
    motion_drive_t last_drive;
} sim_t;

static void sim_init(sim_t *s, int32_t pos)
{
    *s = (sim_t){ .pos = pos, .sign = 1, .min_turn = 700, .min_drive_duty = 0xFFFF, .first_drive_t = UINT32_MAX };
}

static void sim_apply(sim_t *s, uint32_t t, const motion_out_t *o)
{
    if (o->drive == MOTION_OUT_DRIVE) {
        if (o->duty > s->max_duty) s->max_duty = o->duty;
        if (o->duty < s->min_drive_duty) s->min_drive_duty = o->duty;
        if (s->drive_ticks && s->last_drive == MOTION_OUT_DRIVE && o->dir != s->last_drive_dir)
            s->reversed_without_brake = true;
        s->last_drive_dir = o->dir;
        s->drive_ticks++;
        if (s->first_drive_t == UINT32_MAX) s->first_drive_t = t;
        s->last_drive_t = t;
        if (!s->stuck && o->duty >= s->min_turn) s->pos += s->sign * o->dir * (o->duty / 50);
    } else if (o->drive == MOTION_OUT_BRAKE) {
        s->brake_ticks++;
    }
    s->last_drive = o->drive;
}

/* Takte rechnen, bis ein Ergebnis kommt (oder max_ms); liefert Ergebnis, *t = Zeitpunkt */
static motion_result_t run(motion_t *m, sim_t *s, uint32_t *t, uint32_t max_ms)
{
    uint32_t end = *t + max_ms;
    while (*t < end) {
        motion_out_t o = motion_tick(m, *t, s->pos);
        sim_apply(s, *t, &o);
        if (o.result != MOTION_RES_NONE) return o.result;
        *t += TICK;
    }
    return MOTION_RES_NONE;
}

static void test_goto_reaches_goal(void)
{
    motion_t m; sim_t s; uint32_t t = 1000;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 5000, 0);
    TEST_ASSERT_TRUE(motion_is_active(&m));
    TEST_ASSERT_EQUAL(MOTION_RES_DONE, run(&m, &s, &t, 20000));
    TEST_ASSERT_INT32_WITHIN(20, 5000, s.pos);
    TEST_ASSERT_FALSE(motion_is_active(&m));
    TEST_ASSERT_EQUAL_UINT16(1000, s.max_duty);          /* Kick */
    TEST_ASSERT_TRUE(s.min_drive_duty >= P.min_duty);    /* nie unter Anlaufschwelle */
    TEST_ASSERT_TRUE(s.brake_ticks >= (int)(P.brake_ms / TICK));/* Kurzschlussbremse am Ende */
    /* Nach dem Ergebnis: aus */
    motion_out_t o = motion_tick(&m, t + TICK, s.pos);
    TEST_ASSERT_EQUAL(MOTION_OUT_OFF, o.drive);
    TEST_ASSERT_EQUAL(MOTION_RES_NONE, o.result);
}

static void test_goto_backwards(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 3000);
    motion_goto(&m, t, s.pos, -1000, 0);
    TEST_ASSERT_EQUAL(MOTION_RES_DONE, run(&m, &s, &t, 20000));
    TEST_ASSERT_INT32_WITHIN(20, -1000, s.pos);
}

static void test_already_at_goal(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 500);
    motion_goto(&m, t, s.pos, 505, 0);                   /* innerhalb Toleranz */
    TEST_ASSERT_EQUAL(MOTION_RES_DONE, run(&m, &s, &t, 1000));
    TEST_ASSERT_EQUAL_INT(0, s.drive_ticks);
}

static void test_cruise_duty_and_brake_zone(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 3000, 0);
    /* Bis in die Cruise-Phase laufen und Duty prüfen */
    bool saw_cruise = false;
    for (int i = 0; i < 500; i++, t += TICK) {
        motion_out_t o = motion_tick(&m, t, s.pos);
        sim_apply(&s, t, &o);
        if (m.phase == MOTION_CRUISE && 3000 - s.pos > P.brake_zone_steps) {
            TEST_ASSERT_EQUAL_UINT16(P.run_duty, o.duty);
            saw_cruise = true;
        }
        if (o.drive == MOTION_OUT_DRIVE && 3000 - s.pos < P.brake_zone_steps / 2) {
            TEST_ASSERT_TRUE(o.duty < P.run_duty);       /* Bremszone */
            TEST_ASSERT_TRUE(o.duty >= P.min_duty);
        }
        if (o.result != MOTION_RES_NONE) break;
    }
    TEST_ASSERT_TRUE(saw_cruise);
}

static void test_kick_without_movement_blocks(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    s.stuck = true;                                      /* steht am Anschlag */
    motion_run(&m, t, s.pos, -1, 0);
    TEST_ASSERT_EQUAL(MOTION_RES_BLOCKED, run(&m, &s, &t, 5000));
    TEST_ASSERT_TRUE(t >= P.kick_ms && t <= P.kick_ms + P.brake_ms + 2 * TICK);
}

static void test_block_in_cruise_only_after_ramp(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_run(&m, t, s.pos, 1, 0);
    /* Kick abwarten, dann sofort festhalten (in der Rampe) */
    while (m.phase == MOTION_KICK) { motion_out_t o = motion_tick(&m, t, s.pos); sim_apply(&s, t, &o); t += TICK; }
    TEST_ASSERT_EQUAL(MOTION_RAMP, m.phase);
    uint32_t ramp_start = t;
    s.stuck = true;
    TEST_ASSERT_EQUAL(MOTION_RES_BLOCKED, run(&m, &s, &t, 5000));
    /* Erkennung erst nach Rampe + Fenster (+ Bremszeit) */
    TEST_ASSERT_TRUE(t - ramp_start >= P.ramp_up_ms + P.block_window_ms);
    TEST_ASSERT_TRUE(t - ramp_start <= P.ramp_up_ms + P.block_window_ms + P.brake_ms + 3 * TICK);
}

static void test_block_mid_travel(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 100000, 0);
    TEST_ASSERT_EQUAL(MOTION_RES_NONE, run(&m, &s, &t, 2000));
    s.stuck = true;
    uint32_t t0 = t;
    TEST_ASSERT_EQUAL(MOTION_RES_BLOCKED, run(&m, &s, &t, 2000));
    TEST_ASSERT_TRUE(t - t0 <= P.block_window_ms + P.brake_ms + 3 * TICK);
}

static void test_timeout(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_run(&m, t, s.pos, 1, 1500);                   /* ohne Grenzen, ohne Ziel */
    TEST_ASSERT_EQUAL(MOTION_RES_TIMEOUT, run(&m, &s, &t, 10000));
    TEST_ASSERT_TRUE(t >= 1500 && t <= 1500 + P.brake_ms + 2 * TICK);
}

static void test_timeout_capped_by_max(void)
{
    motion_params_t p = P;
    p.timeout_max_ms = 800;
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &p);
    sim_init(&s, 0);
    motion_run(&m, t, s.pos, 1, 99999);
    TEST_ASSERT_EQUAL(MOTION_RES_TIMEOUT, run(&m, &s, &t, 10000));
    TEST_ASSERT_TRUE(t <= 800 + p.brake_ms + 2 * TICK);
}

static void test_wrong_direction(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    s.sign = -1;                                         /* Vorzeichen falsch */
    motion_goto(&m, t, s.pos, 5000, 0);
    TEST_ASSERT_EQUAL(MOTION_RES_WRONG_DIR, run(&m, &s, &t, 5000));
    TEST_ASSERT_TRUE(s.pos > -200);                      /* früh abgebrochen */
}

static void test_limits_run_until_endpoint(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    motion_set_limits(&m, true, 0, 2000);
    sim_init(&s, 1000);
    motion_run(&m, t, s.pos, 1, 0);
    TEST_ASSERT_EQUAL(MOTION_RES_LIMIT, run(&m, &s, &t, 10000));
    TEST_ASSERT_INT32_WITHIN(25, 2000, s.pos);
}

static void test_goto_clamped_to_limits(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    motion_set_limits(&m, true, 0, 2000);
    sim_init(&s, 1000);
    motion_goto(&m, t, s.pos, -5000, 0);
    TEST_ASSERT_EQUAL_INT32(0, m.job.goal);
    TEST_ASSERT_EQUAL(MOTION_RES_DONE, run(&m, &s, &t, 10000));
    TEST_ASSERT_INT32_WITHIN(25, 0, s.pos);
}

static void test_stop(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 100000, 0);
    run(&m, &s, &t, 1000);
    TEST_ASSERT_TRUE(motion_stop(&m, t));
    int brakes_before = s.brake_ticks;
    TEST_ASSERT_EQUAL(MOTION_RES_STOPPED, run(&m, &s, &t, 1000));
    TEST_ASSERT_TRUE(s.brake_ticks - brakes_before >= (int)(P.brake_ms / TICK));
    TEST_ASSERT_FALSE(motion_is_active(&m));
    TEST_ASSERT_FALSE(motion_stop(&m, t));               /* stand schon: kein Ergebnis */
}

static void test_reversal_ramps_brakes_pauses(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 100000, 0);
    run(&m, &s, &t, 1500);                               /* in Fahrt (+) */
    motion_goto(&m, t, s.pos, -3000, 0);                 /* Gegenrichtung */
    TEST_ASSERT_EQUAL(MOTION_STOPPING, m.phase);
    int results = 0;
    uint32_t last_pos_drive_t = 0, first_neg_drive_t = 0;
    bool in_reverse = false;
    for (int i = 0; i < 3000; i++, t += TICK) {
        motion_out_t o = motion_tick(&m, t, s.pos);
        if (o.drive == MOTION_OUT_DRIVE && o.dir > 0) last_pos_drive_t = t;
        if (o.drive == MOTION_OUT_DRIVE && o.dir < 0 && !in_reverse) { in_reverse = true; first_neg_drive_t = t; }
        sim_apply(&s, t, &o);
        if (o.result != MOTION_RES_NONE) {
            results++;
            TEST_ASSERT_EQUAL(MOTION_RES_DONE, o.result);
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT(1, results);                   /* nur der neue Auftrag meldet */
    TEST_ASSERT_TRUE(in_reverse);
    TEST_ASSERT_FALSE(s.reversed_without_brake);
    /* Bremsen + Pause zwischen den Richtungen */
    TEST_ASSERT_TRUE(first_neg_drive_t - last_pos_drive_t >= P.brake_ms + P.dir_pause_ms);
    TEST_ASSERT_INT32_WITHIN(25, -3000, s.pos);
}

static void test_reversal_respects_limit_while_stopping(void)
{
    /* ohne Bremszone fährt die Achse mit run_duty bis kurz vor den Endpunkt;
     * das Abbremsen vor dem Richtungswechsel darf ihn nicht überfahren */
    motion_params_t p = P;
    p.brake_zone_steps = 0;
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &p);
    motion_set_limits(&m, true, 0, 2000);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 2000, 0);
    /* knapp vor dem Endpunkt die Richtung wechseln */
    while (s.pos < 1990 - 20) { motion_out_t o = motion_tick(&m, t, s.pos); sim_apply(&s, t, &o); t += TICK; }
    motion_goto(&m, t, s.pos, 500, 0);
    TEST_ASSERT_EQUAL(MOTION_STOPPING, m.phase);
    for (int i = 0; i < 100; i++, t += TICK) {
        motion_out_t o = motion_tick(&m, t, s.pos);
        sim_apply(&s, t, &o);
        if (m.phase != MOTION_STOPPING) break;
    }
    TEST_ASSERT_TRUE(s.pos <= 2000 + 20);                /* nicht über den Endpunkt gerampt */
}

static void test_same_direction_updates_goal(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 2000, 0);
    run(&m, &s, &t, 500);
    motion_goto(&m, t, s.pos, 4000, 0);                  /* gleiche Richtung, weiter */
    TEST_ASSERT_NOT_EQUAL(MOTION_STOPPING, m.phase);
    TEST_ASSERT_EQUAL(MOTION_RES_DONE, run(&m, &s, &t, 20000));
    TEST_ASSERT_INT32_WITHIN(20, 4000, s.pos);
}

static void test_abort_no_result(void)
{
    motion_t m; sim_t s; uint32_t t = 0;
    motion_init(&m, &P);
    sim_init(&s, 0);
    motion_goto(&m, t, s.pos, 100000, 0);
    run(&m, &s, &t, 500);
    motion_abort(&m, t);                                 /* nFAULT */
    TEST_ASSERT_FALSE(motion_is_active(&m));
    TEST_ASSERT_EQUAL(MOTION_RES_NONE, run(&m, &s, &t, 500));
}

static void test_params_sanitized(void)
{
    motion_params_t p = P;
    p.run_duty = 300;        /* unter min */
    p.kick_duty = 1500;      /* über 100 % */
    p.kick_steps = 0;
    motion_t m;
    motion_init(&m, &p);
    TEST_ASSERT_EQUAL_UINT16(700, m.p.run_duty);
    TEST_ASSERT_EQUAL_UINT16(1000, m.p.kick_duty);
    TEST_ASSERT_EQUAL_INT32(1, m.p.kick_steps);
}

void run_motion_tests(void)
{
    RUN_TEST(test_goto_reaches_goal);
    RUN_TEST(test_goto_backwards);
    RUN_TEST(test_already_at_goal);
    RUN_TEST(test_cruise_duty_and_brake_zone);
    RUN_TEST(test_kick_without_movement_blocks);
    RUN_TEST(test_block_in_cruise_only_after_ramp);
    RUN_TEST(test_block_mid_travel);
    RUN_TEST(test_timeout);
    RUN_TEST(test_timeout_capped_by_max);
    RUN_TEST(test_wrong_direction);
    RUN_TEST(test_limits_run_until_endpoint);
    RUN_TEST(test_goto_clamped_to_limits);
    RUN_TEST(test_stop);
    RUN_TEST(test_reversal_ramps_brakes_pauses);
    RUN_TEST(test_reversal_respects_limit_while_stopping);
    RUN_TEST(test_same_direction_updates_goal);
    RUN_TEST(test_abort_no_result);
    RUN_TEST(test_params_sanitized);
}
