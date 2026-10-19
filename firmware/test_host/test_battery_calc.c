/* Akku-Kennlinie, Teiler, NTC, Unterspannungssperre */
#include "unity.h"
#include "tests.h"
#include "battery_calc.h"
#include "config.h"

static void test_divider(void)
{
    TEST_ASSERT_EQUAL_UINT32(3700, batt_calc_vbat_mv(1850));
    TEST_ASSERT_EQUAL_UINT32(0, batt_calc_vbat_mv(0));
}

static void test_percent_curve_points(void)
{
    TEST_ASSERT_EQUAL_UINT8(200, batt_calc_percent_x2(4200));
    TEST_ASSERT_EQUAL_UINT8(120, batt_calc_percent_x2(3800));
    TEST_ASSERT_EQUAL_UINT8(80, batt_calc_percent_x2(3700));
    TEST_ASSERT_EQUAL_UINT8(20, batt_calc_percent_x2(3500));
    TEST_ASSERT_EQUAL_UINT8(0, batt_calc_percent_x2(3300));
}

static void test_percent_interpolation_and_clamp(void)
{
    TEST_ASSERT_EQUAL_UINT8(100, batt_calc_percent_x2(3750));   /* Mitte 40..60 % = 50 % */
    TEST_ASSERT_EQUAL_UINT8(50, batt_calc_percent(3750));
    TEST_ASSERT_EQUAL_UINT8(80, batt_calc_percent(4000));       /* Mitte 60..100 % */
    TEST_ASSERT_EQUAL_UINT8(200, batt_calc_percent_x2(4350));   /* Laden: über 4,2 V */
    TEST_ASSERT_EQUAL_UINT8(0, batt_calc_percent_x2(3000));
    TEST_ASSERT_EQUAL_UINT8(5, batt_calc_percent(3400));        /* Mitte 0..10 % */
}

static void test_percent_monotonic(void)
{
    uint8_t last = 0;
    for (uint32_t mv = 3000; mv <= 4400; mv += 5) {
        uint8_t p = batt_calc_percent_x2(mv);
        TEST_ASSERT_TRUE(p >= last);
        TEST_ASSERT_TRUE(p <= 200);
        last = p;
    }
}

static void test_ntc(void)
{
    float t = -99.0f;
    TEST_ASSERT_EQUAL(BATT_NTC_OK, batt_calc_ntc(NTC_VREF_MV / 2, &t));   /* R = 10k -> 25 °C */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, t);
    TEST_ASSERT_EQUAL(BATT_NTC_OK, batt_calc_ntc(1000, &t));              /* kleinere Spannung = wärmer */
    TEST_ASSERT_TRUE(t > 25.0f);
    TEST_ASSERT_EQUAL(BATT_NTC_OK, batt_calc_ntc(2500, &t));
    TEST_ASSERT_TRUE(t < 25.0f);
    TEST_ASSERT_EQUAL(BATT_NTC_OPEN, batt_calc_ntc(3100, &t));            /* ADC sättigt ~3,1 V */
    TEST_ASSERT_EQUAL(BATT_NTC_SHORT, batt_calc_ntc(10, &t));
    TEST_ASSERT_EQUAL(BATT_NTC_OK, batt_calc_ntc(1650, NULL));            /* NULL erlaubt */
}

static void test_lockout_hysteresis(void)
{
    TEST_ASSERT_FALSE(batt_calc_lockout(false, 3300, false));   /* genau 3,3 V: noch ok */
    TEST_ASSERT_TRUE(batt_calc_lockout(false, 3299, false));
    TEST_ASSERT_TRUE(batt_calc_lockout(true, 3500, false));     /* Hysterese: bleibt */
    TEST_ASSERT_TRUE(batt_calc_lockout(true, BATT_LOCKOUT_RELEASE_MV, true));   /* lädt: bleibt */
    TEST_ASSERT_FALSE(batt_calc_lockout(true, BATT_LOCKOUT_RELEASE_MV, false));
    TEST_ASSERT_FALSE(batt_calc_lockout(false, 4300, true));
}

void run_battery_calc_tests(void)
{
    RUN_TEST(test_divider);
    RUN_TEST(test_percent_curve_points);
    RUN_TEST(test_percent_interpolation_and_clamp);
    RUN_TEST(test_percent_monotonic);
    RUN_TEST(test_ntc);
    RUN_TEST(test_lockout_hysteresis);
}
