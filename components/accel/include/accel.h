#pragma once
#include <stdbool.h>

/* Test 6: I2C-Scan 0x08..0x77 (SDA GPIO3, SCL GPIO25, externe Pull-ups).
 * PASS, wenn der LIS2DH12 auf 0x18 antwortet. */
bool i2cscan_test(void);
/* Test 7: LIS2DH12 WHO_AM_I (0x33), dann 10 Hz / +-2 g, duration_s lang X/Y/Z in g
 * und Neigung der Z-Achse zur Senkrechten. PASS bei Mittelwert |a| = 0,8 ... 1,2 g. */
bool accel_test(int duration_s);
