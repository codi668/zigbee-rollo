#include <stdio.h>
#include <math.h>
#include "adc_meas.h"
#include "board_pins.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AVG_SAMPLES 16

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_ch_vbat, s_ch_ntc;
static bool s_ready;

bool adc_meas_init(void)
{
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(PIN_VBAT_ADC, &unit, &s_ch_vbat) != ESP_OK || unit != ADC_UNIT_1) {
        printf("ADC: GPIO%d ist kein ADC1-Kanal!\n", PIN_VBAT_ADC);
        return false;
    }
    printf("ADC: GPIO%d = ADC1 Kanal %d\n", PIN_VBAT_ADC, s_ch_vbat);
    if (adc_oneshot_io_to_channel(PIN_NTC_ADC, &unit, &s_ch_ntc) != ESP_OK || unit != ADC_UNIT_1) {
        printf("ADC: GPIO%d ist kein ADC1-Kanal!\n", PIN_NTC_ADC);
        return false;
    }
    printf("ADC: GPIO%d = ADC1 Kanal %d\n", PIN_NTC_ADC, s_ch_ntc);

    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) return false;

    adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_oneshot_config_channel(s_adc, s_ch_vbat, &ccfg) != ESP_OK) return false;
    if (adc_oneshot_config_channel(s_adc, s_ch_ntc, &ccfg) != ESP_OK) return false;

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t e = adc_cali_create_scheme_curve_fitting(&cal, &s_cali);
    if (e != ESP_OK) {
        printf("ADC: Kalibrierung (curve fitting) nicht verfuegbar (%s)\n", esp_err_to_name(e));
        return false;
    }
    s_ready = true;
    return true;
}

static int read_avg_mv(adc_channel_t ch)
{
    if (!s_ready) return -1;
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
        int mv;
        if (adc_oneshot_get_calibrated_result(s_adc, s_cali, ch, &mv) != ESP_OK) return -1;
        sum += mv;
        vTaskDelay(1);
    }
    return (int)(sum / AVG_SAMPLES);
}

int adc_meas_vbat_pin_mv(void) { return read_avg_mv(s_ch_vbat); }
int adc_meas_ntc_pin_mv(void)  { return read_avg_mv(s_ch_ntc); }

bool vbat_test(void)
{
    int mv = adc_meas_vbat_pin_mv();
    if (mv < 0) {
        printf("Akku: FAIL (ADC-Fehler / nicht initialisiert)\n");
        return false;
    }
    float v = mv / 1000.0f * VBAT_DIVIDER;
    bool ok = v >= 3.0f && v <= 4.25f;
    printf("Akku: ADC-Pin %d mV (Mittel aus %d) -> Akkuspannung %.3f V -> %s\n",
           mv, AVG_SAMPLES, v,
           ok ? "PASS (3,0 ... 4,25 V)"
              : (v < 3.0f ? "FAIL (unter 3,0 V: Akku leer/nicht angeschlossen, Teiler offen?)"
                          : "FAIL (ueber 4,25 V: Teiler/Lader pruefen)"));
    return ok;
}

bool ntc_test(void)
{
    int mv = adc_meas_ntc_pin_mv();
    if (mv < 0) {
        printf("NTC: FAIL (ADC-Fehler / nicht initialisiert)\n");
        return false;
    }
    float vm = mv / 1000.0f;
    printf("NTC: Mittelpunkt %d mV (Annahme 3V3 = %.2f V)\n", mv, NTC_VREF_V);
    /* ADC (12 dB) saettigt kalibriert bei ca. 3,1 V, daher Schwelle 3,0 V statt VREF-0,1 V */
    if (vm > 3.0f) {
        printf("NTC: FAIL - Spannung ~3V3: NTC offen/nicht bestueckt oder Leiterbahn unterbrochen\n");
        return false;
    }
    if (vm < 0.05f) {
        printf("NTC: FAIL - Spannung ~0 V: Kurzschluss gegen GND (NTC/Leitung) oder Pull-up fehlt\n");
        return false;
    }
    float r = NTC_R_FIXED_OHM * vm / (NTC_VREF_V - vm);
    float inv_t = 1.0f / 298.15f + logf(r / NTC_R25_OHM) / NTC_BETA;
    float temp_c = 1.0f / inv_t - 273.15f;
    bool ok = temp_c >= 5.0f && temp_c <= 45.0f;
    printf("NTC: R = %.0f Ohm, Temperatur = %.1f Grad C (Beta %.0f) -> %s\n",
           r, temp_c, NTC_BETA, ok ? "PASS (5 ... 45 Grad C)" : "FAIL (ausserhalb 5 ... 45 Grad C)");
    return ok;
}
