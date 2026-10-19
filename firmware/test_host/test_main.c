/* Host-Testlauf: alle Gruppen nacheinander, Exit-Code = Anzahl Fehler */
#include "unity.h"
#include "tests.h"

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();
    run_battery_calc_tests();
    run_axis_tests();
    run_block_detect_tests();
    run_motion_tests();
    run_lis2dh12_conv_tests();
    run_tilt_logic_tests();
    run_btn_logic_tests();
    run_led_pattern_tests();
    run_app_logic_tests();
    run_zb_logic_tests();
    return UNITY_END();
}
