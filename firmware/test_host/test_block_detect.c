/* Blockiererkennung mit Testdaten */
#include "unity.h"
#include "tests.h"
#include "block_detect.h"

static void test_moving_never_blocks(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    int32_t pos = 0;
    for (uint32_t t = 0; t < 5000; t += 10) {
        pos += 17;                      /* N20 bei 70 %: ~17 Schritte / 10 ms */
        TEST_ASSERT_FALSE(block_det_update(&b, t, pos));
    }
}

static void test_stall_detected_after_window(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    uint32_t t = 0;
    int32_t pos = 0;
    for (; t < 1000; t += 10) {
        pos += 17;
        block_det_update(&b, t, pos);
    }
    uint32_t stall_t = t;
    bool hit = false;
    for (; t < 2000 && !hit; t += 10) hit = block_det_update(&b, t, pos);
    TEST_ASSERT_TRUE(hit);
    uint32_t dt = (t - 10) - stall_t;
    TEST_ASSERT_TRUE(dt >= 180 && dt <= 220);    /* etwa ein Fenster */
}

static void test_not_before_window_full(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    for (uint32_t t = 0; t < 200; t += 10) TEST_ASSERT_FALSE(block_det_update(&b, t, 0));
    TEST_ASSERT_TRUE(block_det_update(&b, 200, 0));
}

static void test_slow_creep_counts_as_blocked(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    bool hit = false;
    int32_t pos = 0;
    for (uint32_t t = 0; t <= 400 && !hit; t += 10) {
        pos += 0 + (t % 20 == 0);          /* 1 Schritt / 20 ms = 10 pro Fenster < 20 */
        hit = block_det_update(&b, t, pos);
    }
    TEST_ASSERT_TRUE(hit);
}

static void test_reset_and_negative_direction(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    for (uint32_t t = 0; t < 300; t += 10) block_det_update(&b, t, 0);
    block_det_reset(&b);
    TEST_ASSERT_FALSE(block_det_update(&b, 300, 0));   /* nach Reset erst wieder volles Fenster */
    int32_t pos = 0;
    for (uint32_t t = 310; t < 1000; t += 10) {
        pos -= 17;                                       /* Rückwärts = Betrag zählt */
        TEST_ASSERT_FALSE(block_det_update(&b, t, pos));
    }
}

static void test_time_wraparound(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    uint32_t t = 0xFFFFFF00u;
    int32_t pos = 0;
    for (int i = 0; i < 100; i++, t += 10) {
        pos += 17;
        TEST_ASSERT_FALSE(block_det_update(&b, t, pos));
    }
    bool hit = false;
    for (int i = 0; i < 30 && !hit; i++, t += 10) hit = block_det_update(&b, t, pos);
    TEST_ASSERT_TRUE(hit);
}

static void test_irregular_ticks(void)
{
    block_det_t b;
    block_det_init(&b, 200, 20);
    /* ein Takt kommt 300 ms zu spät, Motor dreht trotzdem -> kein Fehlalarm */
    int32_t pos = 0;
    uint32_t t = 0;
    for (; t < 500; t += 10) { pos += 17; TEST_ASSERT_FALSE(block_det_update(&b, t, pos)); }
    t += 300;
    pos += 17 * 30;
    TEST_ASSERT_FALSE(block_det_update(&b, t, pos));
}

void run_block_detect_tests(void)
{
    RUN_TEST(test_moving_never_blocks);
    RUN_TEST(test_stall_detected_after_window);
    RUN_TEST(test_not_before_window_full);
    RUN_TEST(test_slow_creep_counts_as_blocked);
    RUN_TEST(test_reset_and_negative_direction);
    RUN_TEST(test_time_wraparound);
    RUN_TEST(test_irregular_ticks);
}
