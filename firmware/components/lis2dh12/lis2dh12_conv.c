/* LIS2DH12 Rohwert-Umrechnung – reine Logik (test_host/test_tilt.c) */
#include "lis2dh12_conv.h"

int16_t lis2dh12_raw_to_mg(uint8_t lo, uint8_t hi)
{
    /* 16 Bit links-justiert, 10 Bit gültig -> arithmetisch um 6 schieben */
    int16_t v = (int16_t)(((uint16_t)hi << 8) | lo);
    return (int16_t)((v >> 6) * LIS2DH12_MG_PER_LSB_10BIT);
}

void lis2dh12_convert(const uint8_t raw[6], lis2dh12_mg_t *out)
{
    out->x = lis2dh12_raw_to_mg(raw[0], raw[1]);
    out->y = lis2dh12_raw_to_mg(raw[2], raw[3]);
    out->z = lis2dh12_raw_to_mg(raw[4], raw[5]);
}
