/* Akku-/NTC-Rechnung – reine Logik, Host-getestet (test_host/test_battery.c) */
#include "battery_calc.h"
#include "config.h"
#include <math.h>

typedef struct { uint16_t mv; uint8_t pct; } curve_pt_t;
static const curve_pt_t s_curve[] = BATT_CURVE_POINTS;
#define CURVE_N (sizeof(s_curve) / sizeof(s_curve[0]))

uint32_t batt_calc_vbat_mv(uint32_t pin_mv)
{
    return pin_mv * BATT_DIVIDER_NUM / BATT_DIVIDER_DEN;
}

uint8_t batt_calc_percent_x2(uint32_t mv)
{
    if (mv >= s_curve[0].mv) return s_curve[0].pct * 2;
    if (mv <= s_curve[CURVE_N - 1].mv) return s_curve[CURVE_N - 1].pct * 2;
    for (unsigned i = 1; i < CURVE_N; i++) {
        const curve_pt_t *hi = &s_curve[i - 1], *lo = &s_curve[i];
        if (mv >= lo->mv) {
            /* linear zwischen lo und hi, in halben Prozent, gerundet */
            uint32_t num = (uint32_t)(mv - lo->mv) * (uint32_t)(hi->pct - lo->pct) * 2;
            uint32_t den = hi->mv - lo->mv;
            return (uint8_t)(lo->pct * 2 + (num + den / 2) / den);
        }
    }
    return 0;
}

uint8_t batt_calc_percent(uint32_t mv)
{
    return (uint8_t)((batt_calc_percent_x2(mv) + 1) / 2);
}

batt_ntc_state_t batt_calc_ntc(uint32_t pin_mv, float *temp_c)
{
    if (pin_mv > NTC_OPEN_MV) return BATT_NTC_OPEN;
    if (pin_mv < NTC_SHORT_MV) return BATT_NTC_SHORT;
    float vm = (float)pin_mv;
    float r = NTC_R_FIXED_OHM * vm / ((float)NTC_VREF_MV - vm);
    float inv_t = 1.0f / 298.15f + logf(r / NTC_R25_OHM) / NTC_BETA;
    if (temp_c) *temp_c = 1.0f / inv_t - 273.15f;
    return BATT_NTC_OK;
}

bool batt_calc_lockout(bool locked, uint32_t vbat_mv, bool charging)
{
    if (!locked) return vbat_mv < BATT_LOCKOUT_MV;
    return !(vbat_mv >= BATT_LOCKOUT_RELEASE_MV && !charging);
}
