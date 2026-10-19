/* BOOT-Taste: Entprellung, kurz / 3 s / 10 s */
#include "unity.h"
#include "tests.h"
#include "btn_logic.h"
#include "config.h"

#define STEP BTN_POLL_MS

/* Pegel 'pressed' für ms anlegen, Ereignisse zählen; liefert letztes Ereignis */
typedef struct { int n[BTN_EV_VERY_LONG + 1]; btn_event_t last; uint32_t last_t; } evlog_t;

static void hold(btn_logic_t *b, uint32_t *t, bool pressed, uint32_t ms, evlog_t *log)
{
    for (uint32_t end = *t + ms; *t < end; *t += STEP) {
        btn_event_t e = btn_logic_update(b, *t, pressed);
        if (e != BTN_EV_NONE) { log->n[e]++; log->last = e; log->last_t = *t; }
    }
}

static void test_short_press(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, false);
    TEST_ASSERT_FALSE(btn_logic_busy(&b));
    hold(&b, &t, true, 200, &l);
    TEST_ASSERT_TRUE(btn_logic_busy(&b));
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_SHORT]);          /* erst beim Loslassen */
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_SHORT]);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_LONG] + l.n[BTN_EV_LONG_HINT] + l.n[BTN_EV_VERY_LONG]);
    TEST_ASSERT_FALSE(btn_logic_busy(&b));
}

static void test_bounce_ignored(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, false);
    for (int i = 0; i < 5; i++) {                          /* Prellen kürzer als BTN_DEBOUNCE_MS */
        hold(&b, &t, true, 10, &l);
        hold(&b, &t, false, 10, &l);
    }
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_SHORT]);
    /* Prellen beim Drücken, dann stabil: genau ein Druck */
    hold(&b, &t, true, 10, &l);
    hold(&b, &t, false, 10, &l);
    hold(&b, &t, true, 300, &l);
    hold(&b, &t, false, 10, &l);
    hold(&b, &t, true, 10, &l);
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_SHORT]);
}

static void test_long_press(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, false);
    hold(&b, &t, true, BTN_LONG_MS + 200, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_LONG_HINT]);
    TEST_ASSERT_TRUE(l.last_t >= BTN_LONG_MS && l.last_t <= BTN_LONG_MS + 2 * BTN_DEBOUNCE_MS);
    hold(&b, &t, true, 2000, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_LONG_HINT]);       /* nur einmal */
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_LONG]);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_SHORT]);
}

static void test_just_below_long_is_short(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, false);
    hold(&b, &t, true, BTN_LONG_MS - 100, &l);
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_SHORT]);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_LONG_HINT]);
}

static void test_very_long_press(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, false);
    hold(&b, &t, true, BTN_VERY_LONG_MS + 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_VERY_LONG]);       /* sofort, ohne Loslassen */
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_LONG_HINT]);
    hold(&b, &t, true, 5000, &l);
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_VERY_LONG]);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_LONG] + l.n[BTN_EV_SHORT]);   /* Loslassen erzeugt nichts */
}

static void test_pressed_at_init_ignored(void)
{
    btn_logic_t b; evlog_t l = { 0 }; uint32_t t = 0;
    btn_logic_init(&b, t, true);
    hold(&b, &t, true, BTN_VERY_LONG_MS + 500, &l);
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(0, l.n[BTN_EV_SHORT] + l.n[BTN_EV_LONG] + l.n[BTN_EV_LONG_HINT] + l.n[BTN_EV_VERY_LONG]);
    hold(&b, &t, true, 100, &l);                           /* danach normal */
    hold(&b, &t, false, 100, &l);
    TEST_ASSERT_EQUAL_INT(1, l.n[BTN_EV_SHORT]);
}

void run_btn_logic_tests(void)
{
    RUN_TEST(test_short_press);
    RUN_TEST(test_bounce_ignored);
    RUN_TEST(test_long_press);
    RUN_TEST(test_just_below_long_is_short);
    RUN_TEST(test_very_long_press);
    RUN_TEST(test_pressed_at_init_ignored);
}
