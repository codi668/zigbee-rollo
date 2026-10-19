/*
 * zb_logic: Umrechnung in Zigbee-Attributwerte, Wiederholungszeiten (reine Logik).
 */
#include "zb_logic.h"
#include "config.h"

uint8_t zbl_position_attr(bool calibrated, uint8_t percent)
{
    if (!calibrated) return ZBL_POS_UNKNOWN;
    return percent > 100 ? 100 : percent;
}

uint8_t zbl_battery_voltage_attr(uint32_t vbat_mv)
{
    uint32_t v = (vbat_mv + 50) / 100;
    return v > 255 ? 255 : (uint8_t)v;
}

uint8_t zbl_battery_percent_attr(uint8_t percent_x2)
{
    return percent_x2 > 200 ? 200 : percent_x2;
}

int16_t zbl_temperature_attr(bool valid, float temp_c)
{
    if (!valid || temp_c != temp_c) return ZBL_TEMP_UNKNOWN;   /* NaN */
    float v = temp_c * 100.0f;
    v += v >= 0 ? 0.5f : -0.5f;
    if (v > 32767.0f) return 32767;
    if (v < -27315.0f) return -27315;      /* ZCL: MinMeasuredValue >= -273,15 °C */
    return (int16_t)v;
}

uint8_t zbl_config_status(bool operational)
{
    return (uint8_t)((operational ? ZBL_CFG_OPERATIONAL : 0) | ZBL_CFG_ONLINE |
                     ZBL_CFG_LIFT_CLOSED_LOOP | ZBL_CFG_TILT_CLOSED_LOOP |
                     ZBL_CFG_LIFT_ENCODER | ZBL_CFG_TILT_ENCODER);
}

uint8_t zbl_mode_attr(bool calibrating, bool reversed)
{
    return (uint8_t)((calibrating ? ZBL_MODE_CALIBRATION : 0) | (reversed ? ZBL_MODE_REVERSED : 0));
}

uint32_t zbl_backoff_ms(uint32_t attempt)
{
    static const uint32_t table[] = { 1000, 5000, 15000 };
    if (attempt < sizeof(table) / sizeof(table[0])) return table[attempt];
    if (attempt < 10) return 60000;
    return 300000;
}

bool zbl_report_due(bool final, uint32_t now_ms, uint32_t last_ms, bool ever_sent)
{
    if (final || !ever_sent) return true;
    return (uint32_t)(now_ms - last_ms) >= ZB_REPORT_PROGRESS_MIN_MS;
}
