/**
 * @file lis2dh12_conv.h
 * @brief LIS2DH12 Registerkonstanten und Rohwert-Umrechnung (reine Logik, Host-getestet).
 */
#pragma once

#include <stdint.h>

#define LIS2DH12_REG_WHO_AM_I    0x0F
#define LIS2DH12_REG_CTRL1       0x20
#define LIS2DH12_REG_CTRL4       0x23
#define LIS2DH12_REG_OUT_X_L     0x28
#define LIS2DH12_WHO_AM_I_VALUE  0x33
#define LIS2DH12_AUTO_INC        0x80   /**< Registeradresse mit Auto-Inkrement */

/** CTRL_REG1: ODR<<4 | LPen(0 = Normal Mode) | Z/Y/X an */
#define LIS2DH12_CTRL1_ODR_1HZ   0x17
#define LIS2DH12_CTRL1_ODR_10HZ  0x27
#define LIS2DH12_CTRL1_POWER_DOWN 0x00
/** CTRL_REG4: BDU = 1, ±2 g, HR = 0 -> Normal Mode 10 Bit */
#define LIS2DH12_CTRL4_VALUE     0x80

#define LIS2DH12_MG_PER_LSB_10BIT 4     /**< ±2 g, Normal Mode: 4 mg/Digit */

/** Beschleunigung in mg */
typedef struct {
    int16_t x, y, z;
} lis2dh12_mg_t;

/** @brief Ein Achsenwert (OUT_L, OUT_H, links-justiert, 10 Bit) in mg. */
int16_t lis2dh12_raw_to_mg(uint8_t lo, uint8_t hi);

/** @brief 6 Bytes ab OUT_X_L in mg umrechnen. */
void lis2dh12_convert(const uint8_t raw[6], lis2dh12_mg_t *out);
