/*
 * battery: ADC Oneshot + Kalibrierung, CHRG_N.
 * Basis: hwtest/components/{adc,charger} (auf der Hardware erprobt).
 */
#include "battery.h"
#include "board_pins.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t s_adc;
/* ESP32-H2 kalibriert je Kanal (SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED),
 * deshalb ein Kalibrier-Handle pro Kanal (hwtest nutzte eins für beide). */
static adc_cali_handle_t s_cali_vbat, s_cali_ntc;
static adc_channel_t s_ch_vbat, s_ch_ntc;
static bool s_ready;
static volatile bool s_lockout;

esp_err_t battery_init(void)
{
    gpio_config_t g = {
        .pin_bit_mask = 1ULL << PIN_CHRG_N,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     /* CHRG_N ist Open-Drain */
    };
    esp_err_t e = gpio_config(&g);
    if (e != ESP_OK) return e;

    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(PIN_VBAT_ADC, &unit, &s_ch_vbat) != ESP_OK || unit != ADC_UNIT_1 ||
        adc_oneshot_io_to_channel(PIN_NTC_ADC, &unit, &s_ch_ntc) != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "Akku-/NTC-Pin ist kein ADC1-Kanal");
        return ESP_ERR_INVALID_ARG;
    }
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    if ((e = adc_oneshot_new_unit(&ucfg, &s_adc)) != ESP_OK) return e;
    adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if ((e = adc_oneshot_config_channel(s_adc, s_ch_vbat, &ccfg)) != ESP_OK) return e;
    if ((e = adc_oneshot_config_channel(s_adc, s_ch_ntc, &ccfg)) != ESP_OK) return e;

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .chan = s_ch_vbat,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if ((e = adc_cali_create_scheme_curve_fitting(&cal, &s_cali_vbat)) == ESP_OK) {
        cal.chan = s_ch_ntc;
        e = adc_cali_create_scheme_curve_fitting(&cal, &s_cali_ntc);
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "ADC-Kalibrierung nicht verfuegbar (%s)", esp_err_to_name(e));
        return e;
    }
    s_ready = true;
    ESP_LOGI(TAG, "bereit (ADC1 Kanal %d/%d, %dx Mittelung)", s_ch_vbat, s_ch_ntc, BATT_ADC_SAMPLES);
    return ESP_OK;
}

static esp_err_t read_avg_mv(adc_channel_t ch, adc_cali_handle_t cali, uint32_t *mv_out)
{
    uint32_t sum = 0;
    for (int i = 0; i < BATT_ADC_SAMPLES; i++) {
        int mv;
        esp_err_t e = adc_oneshot_get_calibrated_result(s_adc, cali, ch, &mv);
        if (e != ESP_OK) return e;
        sum += (uint32_t)(mv < 0 ? 0 : mv);
        vTaskDelay(1);
    }
    *mv_out = sum / BATT_ADC_SAMPLES;
    return ESP_OK;
}

esp_err_t battery_measure(battery_status_t *out)
{
    if (!s_ready || !out) return ESP_ERR_INVALID_STATE;
    uint32_t vpin, npin;
    esp_err_t e = read_avg_mv(s_ch_vbat, s_cali_vbat, &vpin);
    if (e == ESP_OK) e = read_avg_mv(s_ch_ntc, s_cali_ntc, &npin);
    if (e != ESP_OK) return e;

    uint32_t vbat = batt_calc_vbat_mv(vpin);
    out->vbat_mv = (uint16_t)vbat;
    out->percent_x2 = batt_calc_percent_x2(vbat);
    out->percent = batt_calc_percent(vbat);
    out->charging = battery_is_charging();
    out->temp_c = 0.0f;
    out->ntc = batt_calc_ntc(npin, &out->temp_c);

    bool was = s_lockout;
    s_lockout = batt_calc_lockout(was, vbat, out->charging);
    out->lockout = s_lockout;
    if (s_lockout != was)
        ESP_LOGW(TAG, "Unterspannungssperre %s (%lu mV)", s_lockout ? "AKTIV" : "aufgehoben", (unsigned long)vbat);
    return ESP_OK;
}

bool battery_is_charging(void)
{
    return gpio_get_level(PIN_CHRG_N) == 0;
}

bool battery_lockout_active(void)
{
    return s_lockout;
}
