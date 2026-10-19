/*
 * LIS2DH12 über i2c_master. Basis: hwtest v0.1.1 components/accel (damals ohne
 * bestückten Sensor nur gegen das Datenblatt geschrieben, nicht auf Hardware getestet).
 */
#include "lis2dh12.h"
#include "board_pins.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "lis2dh12";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_present;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), LIS2DH12_I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, LIS2DH12_I2C_TIMEOUT_MS);
}

static void release(void)
{
    if (s_dev) i2c_master_bus_rm_device(s_dev);
    if (s_bus) i2c_del_master_bus(s_bus);
    s_dev = NULL;
    s_bus = NULL;
    s_present = false;
}

esp_err_t lis2dh12_init(void)
{
    if (s_present) return ESP_OK;
    i2c_master_bus_config_t bcfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   /* externe Pull-ups laut Schaltplan */
    };
    esp_err_t e = i2c_new_master_bus(&bcfg, &s_bus);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "I2C-Bus nicht verfuegbar (%s) - Fenstersensor deaktiviert", esp_err_to_name(e));
        s_bus = NULL;
        return e;
    }
    e = i2c_master_probe(s_bus, LIS2DH12_ADDR, LIS2DH12_I2C_TIMEOUT_MS);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "kein Sensor an 0x%02X (%s) - nicht bestueckt? Fenstersensor deaktiviert",
                 LIS2DH12_ADDR, esp_err_to_name(e));
        release();
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LIS2DH12_ADDR,
        .scl_speed_hz = LIS2DH12_I2C_HZ,
    };
    if ((e = i2c_master_bus_add_device(s_bus, &dcfg, &s_dev)) != ESP_OK) {
        release();
        return e;
    }
    uint8_t who = 0;
    e = reg_read(LIS2DH12_REG_WHO_AM_I, &who, 1);
    if (e != ESP_OK || who != LIS2DH12_WHO_AM_I_VALUE) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (erwartet 0x%02X, %s) - Fenstersensor deaktiviert", who,
                 LIS2DH12_WHO_AM_I_VALUE, esp_err_to_name(e));
        release();
        return ESP_ERR_NOT_FOUND;
    }
    if ((e = reg_write(LIS2DH12_REG_CTRL4, LIS2DH12_CTRL4_VALUE)) != ESP_OK ||
        (e = reg_write(LIS2DH12_REG_CTRL1, LIS2DH12_CTRL1_ODR_1HZ)) != ESP_OK) {
        ESP_LOGW(TAG, "Konfiguration fehlgeschlagen (%s)", esp_err_to_name(e));
        release();
        return e;
    }
    s_present = true;
    ESP_LOGI(TAG, "gefunden (WHO_AM_I 0x33), 1 Hz, +-2 g, 10 Bit");
    return ESP_OK;
}

bool lis2dh12_present(void)
{
    return s_present;
}

esp_err_t lis2dh12_set_rate(lis2dh12_rate_t rate)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    return reg_write(LIS2DH12_REG_CTRL1, rate == LIS2DH12_RATE_10HZ ? LIS2DH12_CTRL1_ODR_10HZ
                                                                     : LIS2DH12_CTRL1_ODR_1HZ);
}

esp_err_t lis2dh12_read_mg(lis2dh12_mg_t *out)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    uint8_t raw[6];
    esp_err_t e = reg_read(LIS2DH12_REG_OUT_X_L | LIS2DH12_AUTO_INC, raw, sizeof(raw));
    if (e == ESP_OK) lis2dh12_convert(raw, out);
    return e;
}

esp_err_t lis2dh12_power_down(void)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    return reg_write(LIS2DH12_REG_CTRL1, LIS2DH12_CTRL1_POWER_DOWN);
}
