/* Prozent <-> Schritte, Ränder, Begrenzung, Endpunkte */
#include "unity.h"
#include "tests.h"
#include "axis.h"

static void test_uncalibrated(void)
{
    axis_t a = { .pos = 123 };
    axis_clear(&a);
    TEST_ASSERT_FALSE(a.calibrated);
    TEST_ASSERT_EQUAL_INT32(123, axis_percent_to_steps(&a, 50));   /* keine Fahrt */
    TEST_ASSERT_EQUAL_UINT8(0, axis_steps_to_percent(&a, 5000));
    TEST_ASSERT_EQUAL_INT32(99999, axis_clamp(&a, 99999));
    TEST_ASSERT_FALSE(axis_at_limit(&a, 0, -1));
    TEST_ASSERT_EQUAL_INT32(0, axis_travel(&a));
}

static void test_percent_with_margins(void)
{
    axis_t a = { 0 };
    axis_set_endpoints(&a, 0, 10000, 15, 0);    /* 1,5 % oben, 0 unten (wie LIFT) */
    TEST_ASSERT_TRUE(a.calibrated);
    TEST_ASSERT_EQUAL_INT32(150, a.margin_top);
    TEST_ASSERT_EQUAL_INT32(0, a.margin_bottom);
    TEST_ASSERT_EQUAL_INT32(150, axis_percent_to_steps(&a, 0));
    TEST_ASSERT_EQUAL_INT32(10000, axis_percent_to_steps(&a, 100));
    TEST_ASSERT_EQUAL_INT32(5075, axis_percent_to_steps(&a, 50));
    TEST_ASSERT_EQUAL_INT32(10000, axis_percent_to_steps(&a, 200));   /* > 100 = 100 */
    TEST_ASSERT_EQUAL_UINT8(0, axis_steps_to_percent(&a, 0));
    TEST_ASSERT_EQUAL_UINT8(0, axis_steps_to_percent(&a, 150));
    TEST_ASSERT_EQUAL_UINT8(50, axis_steps_to_percent(&a, 5075));
    TEST_ASSERT_EQUAL_UINT8(100, axis_steps_to_percent(&a, 10000));
    TEST_ASSERT_EQUAL_UINT8(100, axis_steps_to_percent(&a, 12000));
    TEST_ASSERT_EQUAL_UINT8(0, axis_steps_to_percent(&a, -500));
}

static void test_roundtrip(void)
{
    axis_t a = { 0 };
    axis_set_endpoints(&a, 0, 4200, 15, 15);
    for (int p = 0; p <= 100; p++)
        TEST_ASSERT_EQUAL_UINT8(p, axis_steps_to_percent(&a, axis_percent_to_steps(&a, (uint8_t)p)));
}

static void test_reversed_direction(void)
{
    axis_t a = { 0 };
    axis_set_endpoints(&a, 0, -8000, 0, 0);    /* Encoder zählt nach unten negativ */
    TEST_ASSERT_EQUAL_INT32(8000, axis_travel(&a));
    TEST_ASSERT_EQUAL_INT32(-4000, axis_percent_to_steps(&a, 50));
    TEST_ASSERT_EQUAL_UINT8(25, axis_steps_to_percent(&a, -2000));
    TEST_ASSERT_EQUAL_INT32(-8000, axis_limit_lo(&a));
    TEST_ASSERT_EQUAL_INT32(0, axis_limit_hi(&a));
    TEST_ASSERT_EQUAL_INT32(0, axis_clamp(&a, 300));
    TEST_ASSERT_EQUAL_INT32(-8000, axis_clamp(&a, -9000));
}

static void test_limits(void)
{
    axis_t a = { 0 };
    axis_set_endpoints(&a, 0, 1000, 0, 0);
    TEST_ASSERT_TRUE(axis_at_limit(&a, 1000, 1));
    TEST_ASSERT_TRUE(axis_at_limit(&a, 1005, 1));
    TEST_ASSERT_FALSE(axis_at_limit(&a, 999, 1));
    TEST_ASSERT_TRUE(axis_at_limit(&a, 0, -1));
    TEST_ASSERT_FALSE(axis_at_limit(&a, 0, 1));
    TEST_ASSERT_FALSE(axis_at_limit(&a, 500, 0));
    TEST_ASSERT_EQUAL_INT32(1000, axis_clamp(&a, 5000));
    TEST_ASSERT_EQUAL_INT32(0, axis_clamp(&a, -5));
}

static void test_margins_do_not_overlap(void)
{
    axis_t a = { 0 };
    axis_set_endpoints(&a, 0, 100, 800, 800);
    TEST_ASSERT_TRUE(a.margin_top + a.margin_bottom <= 100);
    TEST_ASSERT_EQUAL_INT32(axis_percent_to_steps(&a, 0), axis_percent_to_steps(&a, 100));
    axis_clear(&a);
    TEST_ASSERT_FALSE(a.calibrated);
}

void run_axis_tests(void)
{
    RUN_TEST(test_uncalibrated);
    RUN_TEST(test_percent_with_margins);
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_reversed_direction);
    RUN_TEST(test_limits);
    RUN_TEST(test_margins_do_not_overlap);
}
