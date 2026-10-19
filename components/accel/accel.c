#include <stdio.h>
#include <math.h>
#include "accel.h"
#include "board_pins.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_WHO_AM_I    0x0F
#define REG_CTRL1       0x20
#define REG_CTRL4       0x23
#define REG_OUT_X_L     0x28
#define WHO_AM_I_VALUE  0x33
#define AUTO_INC        0x80
#define I2C_TIMEOUT_MS  100
#define G_PER_LSB_10BIT 0.004f   /* +-2 g, Normal Mode (10 bit): 4 mg/Digit */

static i2c_master_bus_handle_t s_bus;

static bool bus_init(void)
{
    if (s_bus) return true;
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* externe 4,7k vorhanden */
    };
    esp_err_t e = i2c_new_master_bus(&cfg, &s_bus);
    if (e != ESP_OK) {
        printf("I2C: Bus-Init fehlgeschlagen (%s)\n", esp_err_to_name(e));
        s_bus = NULL;
        return false;
    }
    return true;
}

bool i2cscan_test(void)
{
    if (!bus_init()) { printf("I2C-Scan: FAIL\n"); return false; }
    printf("I2C-Scan: SDA GPIO%d, SCL GPIO%d, Adressen 0x08 ... 0x77\n", PIN_I2C_SDA, PIN_I2C_SCL);
    int found = 0;
    bool lis = false;
    for (int addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            printf("  gefunden: 0x%02X%s\n", addr, addr == LIS2DH12_ADDR ? " (LIS2DH12 erwartet)" : "");
            found++;
            if (addr == LIS2DH12_ADDR) lis = true;
        }
    }
    if (found == 0) {
        printf("I2C-Scan: FAIL - kein Geraet gefunden (Pull-ups/SDA/SCL-Leitungen, Versorgung pruefen)\n");
    } else {
        printf("I2C-Scan: %d Geraet(e) -> %s\n", found,
               lis ? "PASS (0x18 vorhanden)" : "FAIL (0x18 fehlt - LIS2DH12 nicht erreichbar, SA0 pruefen: 0x19 waere SA0=1)");
    }
    return lis;
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

bool accel_test(int duration_s)
{
    if (!bus_init()) { printf("Beschleunigung: FAIL (I2C)\n"); return false; }

    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LIS2DH12_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(s_bus, &dcfg, &dev) != ESP_OK) {
        printf("Beschleunigung: FAIL (Geraet konnte nicht angelegt werden)\n");
        return false;
    }

    bool ok = false;
    uint8_t who = 0;
    esp_err_t e = reg_read(dev, REG_WHO_AM_I, &who, 1);
    if (e != ESP_OK) {
        printf("Beschleunigung: FAIL - keine Antwort von 0x%02X (%s)\n", LIS2DH12_ADDR, esp_err_to_name(e));
        goto out;
    }
    printf("Beschleunigung: WHO_AM_I = 0x%02X (erwartet 0x%02X) -> %s\n", who, WHO_AM_I_VALUE,
           who == WHO_AM_I_VALUE ? "ok" : "FALSCH");
    if (who != WHO_AM_I_VALUE) {
        printf("Beschleunigung: FAIL (falscher Baustein?)\n");
        goto out;
    }

    /* CTRL_REG1: ODR 10 Hz (0010), Normal Mode, X/Y/Z an = 0x27
     * CTRL_REG4: BDU an, +-2 g, Normal Mode (10 bit) = 0x80 */
    if (reg_write(dev, REG_CTRL4, 0x80) != ESP_OK || reg_write(dev, REG_CTRL1, 0x27) != ESP_OK) {
        printf("Beschleunigung: FAIL (Konfiguration schreiben)\n");
        goto out;
    }
    vTaskDelay(pdMS_TO_TICKS(150)); /* erste Messung abwarten */

    printf("Beschleunigung: %d s lang bei 10 Hz, +-2 g (Board bitte ruhig halten bzw. langsam kippen)\n", duration_s);
    printf("   t[s]      X[g]      Y[g]      Z[g]     |a|[g]  Neigung Z-Achse zur Senkrechten\n");
    int n = duration_s * 10;
    int valid = 0, errs = 0;
    float sum_mag = 0, min_mag = 1e9f, max_mag = 0;
    for (int i = 0; i < n; i++) {
        uint8_t raw[6];
        if (reg_read(dev, REG_OUT_X_L | AUTO_INC, raw, sizeof(raw)) != ESP_OK) {
            errs++;
        } else {
            /* 16 bit links-justiert, 10 bit nutzbar -> arithmetisch um 6 schieben */
            int16_t x = ((int16_t)((raw[1] << 8) | raw[0])) >> 6;
            int16_t y = ((int16_t)((raw[3] << 8) | raw[2])) >> 6;
            int16_t z = ((int16_t)((raw[5] << 8) | raw[4])) >> 6;
            float gx = x * G_PER_LSB_10BIT, gy = y * G_PER_LSB_10BIT, gz = z * G_PER_LSB_10BIT;
            float mag = sqrtf(gx * gx + gy * gy + gz * gz);
            float tilt = mag > 0.01f ? acosf(fminf(fmaxf(gz / mag, -1.0f), 1.0f)) * 180.0f / (float)M_PI : 0.0f;
            printf("  %5.1f  %8.3f  %8.3f  %8.3f  %8.3f  %6.1f Grad\n", i / 10.0f, gx, gy, gz, mag, tilt);
            sum_mag += mag;
            if (mag < min_mag) min_mag = mag;
            if (mag > max_mag) max_mag = mag;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (valid == 0) {
        printf("Beschleunigung: FAIL (keine gueltigen Messungen, %d I2C-Fehler)\n", errs);
        goto out;
    }
    float avg = sum_mag / valid;
    ok = avg >= 0.8f && avg <= 1.2f;
    printf("Beschleunigung: |a| Mittel %.3f g (min %.3f, max %.3f), %d Messungen, %d I2C-Fehler -> %s\n",
           avg, min_mag, max_mag, valid, errs,
           ok ? "PASS (0,8 ... 1,2 g)" : "FAIL (ausserhalb 0,8 ... 1,2 g - Board bewegt oder Sensor defekt)");

out:
    reg_write(dev, REG_CTRL1, 0x00); /* Power-down */
    i2c_master_bus_rm_device(dev);
    return ok;
}
