/* Zigbee-Attributwerte und Backoff (main/zb_logic.c) */
#include <math.h>
#include "unity.h"
#include "tests.h"
#include "zb_logic.h"
#include "config.h"

static void test_position_attr(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, zbl_position_attr(false, 40));       /* unkalibriert = 0 (0xFF zeigt ZHA als -155 %) */
    TEST_ASSERT_EQUAL_UINT8(0, zbl_position_attr(true, 0));         /* 0 % = ganz offen */
    TEST_ASSERT_EQUAL_UINT8(100, zbl_position_attr(true, 100));     /* 100 % = ganz zu */
    TEST_ASSERT_EQUAL_UINT8(100, zbl_position_attr(true, 180));
}

static void test_battery_attrs(void)
{
    TEST_ASSERT_EQUAL_UINT8(37, zbl_battery_voltage_attr(3700));    /* 37 = 3,7 V */
    TEST_ASSERT_EQUAL_UINT8(37, zbl_battery_voltage_attr(3749));
    TEST_ASSERT_EQUAL_UINT8(38, zbl_battery_voltage_attr(3750));
    TEST_ASSERT_EQUAL_UINT8(42, zbl_battery_voltage_attr(4200));
    TEST_ASSERT_EQUAL_UINT8(0, zbl_battery_voltage_attr(0));
    TEST_ASSERT_EQUAL_UINT8(255, zbl_battery_voltage_attr(60000));
    TEST_ASSERT_EQUAL_UINT8(200, zbl_battery_percent_attr(200));    /* 200 = 100 % */
    TEST_ASSERT_EQUAL_UINT8(200, zbl_battery_percent_attr(250));
    TEST_ASSERT_EQUAL_UINT8(81, zbl_battery_percent_attr(81));
}

static void test_temperature_attr(void)
{
    TEST_ASSERT_EQUAL_INT16(2137, zbl_temperature_attr(true, 21.37f));
    TEST_ASSERT_EQUAL_INT16(-500, zbl_temperature_attr(true, -5.0f));
    TEST_ASSERT_EQUAL_INT16(0, zbl_temperature_attr(true, 0.0f));
    TEST_ASSERT_EQUAL_INT16((int16_t)0x8000, zbl_temperature_attr(false, 25.0f));
    TEST_ASSERT_EQUAL_INT16((int16_t)0x8000, zbl_temperature_attr(true, NAN));
    TEST_ASSERT_EQUAL_INT16(32767, zbl_temperature_attr(true, 1000.0f));
    TEST_ASSERT_EQUAL_INT16(-27315, zbl_temperature_attr(true, -400.0f));
}

static void test_config_status_and_mode(void)
{
    /* Online | Lift/Tilt Closed Loop | Lift/Tilt Encoder (+ Operational) */
    TEST_ASSERT_EQUAL_HEX8(0x7A, zbl_config_status(false));
    TEST_ASSERT_EQUAL_HEX8(0x7B, zbl_config_status(true));
    TEST_ASSERT_EQUAL_HEX8(0x00, zbl_mode_attr(false, false));      /* nie Wartungsmodus */
    TEST_ASSERT_EQUAL_HEX8(0x02, zbl_mode_attr(true, false));       /* Bit 1 Kalibriermodus */
    TEST_ASSERT_EQUAL_HEX8(0x03, zbl_mode_attr(true, true));
}

static void test_backoff(void)
{
    TEST_ASSERT_EQUAL_UINT32(1000, zbl_backoff_ms(0));
    TEST_ASSERT_EQUAL_UINT32(5000, zbl_backoff_ms(1));
    TEST_ASSERT_EQUAL_UINT32(15000, zbl_backoff_ms(2));
    TEST_ASSERT_EQUAL_UINT32(60000, zbl_backoff_ms(3));
    TEST_ASSERT_EQUAL_UINT32(60000, zbl_backoff_ms(9));
    TEST_ASSERT_EQUAL_UINT32(300000, zbl_backoff_ms(10));
    TEST_ASSERT_EQUAL_UINT32(300000, zbl_backoff_ms(0xFFFFFFFFu));
    for (uint32_t i = 1; i < 20; i++) TEST_ASSERT_TRUE(zbl_backoff_ms(i) >= zbl_backoff_ms(i - 1));
}

static void test_report_due(void)
{
    TEST_ASSERT_TRUE(zbl_report_due(true, 100, 50, true));          /* Endwert sofort */
    TEST_ASSERT_TRUE(zbl_report_due(false, 100, 0, false));         /* erste Meldung */
    TEST_ASSERT_FALSE(zbl_report_due(false, 1000 + ZB_REPORT_PROGRESS_MIN_MS - 1, 1000, true));
    TEST_ASSERT_TRUE(zbl_report_due(false, 1000 + ZB_REPORT_PROGRESS_MIN_MS, 1000, true));
    TEST_ASSERT_TRUE(zbl_report_due(false, 600, 0xFFFFFE00u, true));   /* Überlauf: 512 + 600 ms */
    TEST_ASSERT_FALSE(zbl_report_due(false, 10, 0xFFFFFE00u, true));   /* 522 ms */
}

void run_zb_logic_tests(void)
{
    RUN_TEST(test_position_attr);
    RUN_TEST(test_battery_attrs);
    RUN_TEST(test_temperature_attr);
    RUN_TEST(test_config_status_and_mode);
    RUN_TEST(test_backoff);
    RUN_TEST(test_report_due);
}
