/* LED-Muster: Pairing schnell, Kalibrierung langsam, Fehler Doppelblitz, Normal aus */
#include "unity.h"
#include "tests.h"
#include "led_pattern.h"
#include "config.h"

static int on_count(led_pattern_t p, uint32_t from, uint32_t to)
{
    int n = 0;
    for (uint32_t t = from; t < to; t++) n += led_pattern_level(p, t);
    return n;
}

/* Anzahl Einschaltflanken in [0, per) */
static int flashes(led_pattern_t p)
{
    uint32_t per = led_pattern_period(p);
    int n = 0;
    bool last = led_pattern_level(p, per - 1);
    for (uint32_t t = 0; t < per; t++) {
        bool l = led_pattern_level(p, t);
        if (l && !last) n++;
        last = l;
    }
    return n;
}

static void test_off_and_on(void)
{
    TEST_ASSERT_EQUAL_INT(0, on_count(LED_PAT_OFF, 0, 5000));
    TEST_ASSERT_EQUAL_INT(5000, on_count(LED_PAT_ON, 0, 5000));
    TEST_ASSERT_EQUAL_UINT32(0, led_pattern_period(LED_PAT_OFF));
}

static void test_pairing_faster_than_calibration(void)
{
    TEST_ASSERT_TRUE(led_pattern_period(LED_PAT_PAIRING) < led_pattern_period(LED_PAT_CALIBRATION));
    TEST_ASSERT_EQUAL_INT(1, flashes(LED_PAT_PAIRING));
    TEST_ASSERT_EQUAL_INT(1, flashes(LED_PAT_CALIBRATION));
    TEST_ASSERT_TRUE(led_pattern_level(LED_PAT_PAIRING, 0));
    TEST_ASSERT_FALSE(led_pattern_level(LED_PAT_PAIRING, LED_FAST_ON_MS));
    TEST_ASSERT_EQUAL_INT(LED_SLOW_ON_MS, on_count(LED_PAT_CALIBRATION, 0, led_pattern_period(LED_PAT_CALIBRATION)));
}

static void test_error_double_flash(void)
{
    TEST_ASSERT_EQUAL_INT(2, flashes(LED_PAT_ERROR));
    uint32_t per = led_pattern_period(LED_PAT_ERROR);
    TEST_ASSERT_EQUAL_INT(2 * LED_ERR_FLASH_MS, on_count(LED_PAT_ERROR, 0, per));
    TEST_ASSERT_FALSE(led_pattern_level(LED_PAT_ERROR, LED_ERR_FLASH_MS));          /* Lücke */
    TEST_ASSERT_TRUE(led_pattern_level(LED_PAT_ERROR, LED_ERR_FLASH_MS + LED_ERR_GAP_MS));
    TEST_ASSERT_FALSE(led_pattern_level(LED_PAT_ERROR, per - 1));                   /* Pause */
    TEST_ASSERT_TRUE(led_pattern_level(LED_PAT_ERROR, per));                        /* periodisch */
}

static void test_invalid_pattern_off(void)
{
    TEST_ASSERT_FALSE(led_pattern_level(LED_PAT_COUNT, 0));
}

void run_led_pattern_tests(void)
{
    RUN_TEST(test_off_and_on);
    RUN_TEST(test_pairing_faster_than_calibration);
    RUN_TEST(test_error_double_flash);
    RUN_TEST(test_invalid_pattern_off);
}
