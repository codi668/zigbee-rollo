/* LIS2DH12 Rohwerte (links-justiert, 10 Bit, ±2 g, 4 mg/Digit) */
#include "unity.h"
#include "tests.h"
#include "lis2dh12_conv.h"

static void test_raw_to_mg(void)
{
    TEST_ASSERT_EQUAL_INT16(0, lis2dh12_raw_to_mg(0x00, 0x00));
    TEST_ASSERT_EQUAL_INT16(1024, lis2dh12_raw_to_mg(0x00, 0x40));    /* 256 Digit = 1 g */
    TEST_ASSERT_EQUAL_INT16(-1024, lis2dh12_raw_to_mg(0x00, 0xC0));
    TEST_ASSERT_EQUAL_INT16(4, lis2dh12_raw_to_mg(0x40, 0x00));       /* kleinste Stufe */
    TEST_ASSERT_EQUAL_INT16(-4, lis2dh12_raw_to_mg(0xC0, 0xFF));
    TEST_ASSERT_EQUAL_INT16(0, lis2dh12_raw_to_mg(0x3F, 0x00));       /* untere 6 Bit ungenutzt */
    TEST_ASSERT_EQUAL_INT16(2044, lis2dh12_raw_to_mg(0xC0, 0x7F));    /* Vollausschlag + */
    TEST_ASSERT_EQUAL_INT16(-2048, lis2dh12_raw_to_mg(0x00, 0x80));   /* Vollausschlag - */
}

static void test_convert_xyz(void)
{
    const uint8_t raw[6] = { 0x00, 0x40, 0x00, 0xC0, 0x40, 0x00 };
    lis2dh12_mg_t v;
    lis2dh12_convert(raw, &v);
    TEST_ASSERT_EQUAL_INT16(1024, v.x);
    TEST_ASSERT_EQUAL_INT16(-1024, v.y);
    TEST_ASSERT_EQUAL_INT16(4, v.z);
}

static void test_register_constants(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x33, LIS2DH12_WHO_AM_I_VALUE);
    TEST_ASSERT_EQUAL_HEX8(0x10, LIS2DH12_CTRL1_ODR_1HZ & 0xF0);      /* ODR = 1 Hz */
    TEST_ASSERT_EQUAL_HEX8(0x07, LIS2DH12_CTRL1_ODR_1HZ & 0x0F);      /* XYZ an, LPen = 0 */
    TEST_ASSERT_EQUAL_HEX8(0xA8, LIS2DH12_REG_OUT_X_L | LIS2DH12_AUTO_INC);
}

void run_lis2dh12_conv_tests(void)
{
    RUN_TEST(test_raw_to_mg);
    RUN_TEST(test_convert_xyz);
    RUN_TEST(test_register_constants);
}
