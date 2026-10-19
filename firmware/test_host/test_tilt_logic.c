/* Kipp-Erkennung: Winkel, Hysterese 3°/2°, Entprellung, Sperrzeit nach Fahrt */
#include <math.h>
#include "unity.h"
#include "tests.h"
#include "tilt_logic.h"
#include "config.h"

/* Vektor 1 g, um deg Grad aus der z-Achse in Richtung y gedreht */
static lis2dh12_mg_t rot(float deg)
{
    float r = deg * 3.14159265f / 180.0f;
    return (lis2dh12_mg_t){ 0, (int16_t)lroundf(1000.0f * sinf(r)), (int16_t)lroundf(1000.0f * cosf(r)) };
}

static void test_angle(void)
{
    lis2dh12_mg_t a = rot(0), b = rot(10), c = rot(90);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, tilt_angle_deg(&a, &a));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, tilt_angle_deg(&a, &b));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 90.0f, tilt_angle_deg(&a, &c));
    lis2dh12_mg_t small = { 10, 10, 10 };                 /* freier Fall / Fehler */
    TEST_ASSERT_TRUE(tilt_angle_deg(&a, &small) < 0.0f);
}

static void feed(tilt_det_t *t, float deg, int n, int *changes)
{
    lis2dh12_mg_t v = rot(deg);
    for (int i = 0; i < n; i++)
        if (tilt_det_update(t, &v)) (*changes)++;
}

static void test_no_ref_unknown(void)
{
    tilt_det_t t;
    tilt_det_init(&t);
    int ch = 0;
    feed(&t, 10, 5, &ch);
    TEST_ASSERT_EQUAL(WINDOW_UNKNOWN, t.state);
    TEST_ASSERT_EQUAL_INT(0, ch);
}

static void test_debounce_and_hysteresis(void)
{
    tilt_det_t t;
    tilt_det_init(&t);
    lis2dh12_mg_t ref = rot(0);
    tilt_det_set_ref(&t, &ref);
    int ch = 0;
    feed(&t, 0.5f, WIN_DEBOUNCE_COUNT - 1, &ch);
    TEST_ASSERT_EQUAL(WINDOW_UNKNOWN, t.state);           /* noch nicht entprellt */
    feed(&t, 0.5f, 1, &ch);
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);
    TEST_ASSERT_EQUAL_INT(1, ch);

    feed(&t, 2.5f, 10, &ch);                              /* Band 2..3°: bleibt zu */
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);
    feed(&t, 5.0f, WIN_DEBOUNCE_COUNT - 1, &ch);
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);
    feed(&t, 5.0f, 1, &ch);
    TEST_ASSERT_EQUAL(WINDOW_TILTED, t.state);
    feed(&t, 2.5f, 10, &ch);                              /* Band: bleibt gekippt */
    TEST_ASSERT_EQUAL(WINDOW_TILTED, t.state);
    feed(&t, 1.0f, WIN_DEBOUNCE_COUNT, &ch);
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);
    TEST_ASSERT_EQUAL_INT(3, ch);
}

static void test_single_spike_ignored(void)
{
    tilt_det_t t;
    tilt_det_init(&t);
    lis2dh12_mg_t ref = rot(0);
    tilt_det_set_ref(&t, &ref);
    int ch = 0;
    feed(&t, 0, WIN_DEBOUNCE_COUNT, &ch);
    for (int i = 0; i < 5; i++) {
        feed(&t, 8, 1, &ch);                               /* einzelner Ausreißer */
        feed(&t, 0, 1, &ch);
    }
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);
    TEST_ASSERT_EQUAL_INT(1, ch);
}

static void test_invalid_sample_resets_debounce(void)
{
    tilt_det_t t;
    tilt_det_init(&t);
    lis2dh12_mg_t ref = rot(0), bad = { 0, 0, 0 };
    tilt_det_set_ref(&t, &ref);
    int ch = 0;
    feed(&t, 0, WIN_DEBOUNCE_COUNT, &ch);
    feed(&t, 6, WIN_DEBOUNCE_COUNT - 1, &ch);
    TEST_ASSERT_FALSE(tilt_det_update(&t, &bad));
    feed(&t, 6, 1, &ch);
    TEST_ASSERT_EQUAL(WINDOW_CLOSED, t.state);            /* Zähler war zurückgesetzt */
    feed(&t, 6, WIN_DEBOUNCE_COUNT, &ch);
    TEST_ASSERT_EQUAL(WINDOW_TILTED, t.state);
}

static void test_motion_holdoff(void)
{
    tilt_det_t t;
    tilt_det_init(&t);
    TEST_ASSERT_FALSE(tilt_det_holdoff(&t, 0));
    tilt_det_note_motion(&t, 5000);
    TEST_ASSERT_TRUE(tilt_det_holdoff(&t, 5000));
    TEST_ASSERT_TRUE(tilt_det_holdoff(&t, 5000 + WIN_MOTION_HOLDOFF_MS - 1));
    TEST_ASSERT_FALSE(tilt_det_holdoff(&t, 5000 + WIN_MOTION_HOLDOFF_MS));
}

void run_tilt_logic_tests(void)
{
    RUN_TEST(test_angle);
    RUN_TEST(test_no_ref_unknown);
    RUN_TEST(test_debounce_and_hysteresis);
    RUN_TEST(test_single_spike_ignored);
    RUN_TEST(test_invalid_sample_resets_debounce);
    RUN_TEST(test_motion_holdoff);
}
