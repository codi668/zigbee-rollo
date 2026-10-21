/*
 * DRV8833-Treiber: LEDC-PWM, Slow/Fast Decay, Bremse, nSLEEP, nFAULT-Interrupt.
 * Basis: hwtest/components/{drv,motor} (auf der Hardware erprobt).
 */
#include "drv8833.h"
#include "board_pins.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drv8833";

#define PWM_MODE   LEDC_LOW_SPEED_MODE
#define PWM_TIMER  LEDC_TIMER_0
#define PWM_FULL   (1u << MOTOR_PWM_RES_BITS)   /* 1024 = dauerhaft HIGH (LEDC erlaubt 0…2^bits) */

/* je Kanal zwei LEDC-Kanäle: IN1, IN2 */
static const int s_pins[DRV8833_CH_COUNT][2] = {
    { PIN_M1_IN1, PIN_M1_IN2 },
    { PIN_M2_IN1, PIN_M2_IN2 },
};
static const ledc_channel_t s_ledc[DRV8833_CH_COUNT][2] = {
    { LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    { LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
};

static bool s_ready;
static volatile bool s_enabled;
static volatile int64_t s_enable_us;       /* Zeitpunkt des letzten nSLEEP-HIGH (Blanking) */
static drv8833_fault_cb_t s_fault_cb;
static void *s_fault_ctx;

static void IRAM_ATTR nfault_isr(void *arg)
{
    (void)arg;
    /* Blanking: unmittelbar nach dem Einschalten kommt beim ersten Anlauf nach dem Boot ein
     * kurzer nFAULT-Puls (gemessen: nach 1-2 ms wieder HIGH, Motor steht noch). Er würde die
     * erste Fahrt abbrechen. Im Fenster wird die Flanke ignoriert; bleibt nFAULT LOW, fängt
     * die Abfrage im Regeltakt den Fehler nach dem Fenster ab (drv8833_in_blanking). */
    if ((esp_timer_get_time() - s_enable_us) < (int64_t)DRV8833_FAULT_BLANK_MS * 1000) return;
    /* Sofort alle Brückenausgänge hochohmig – unabhängig von jeder Task */
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    s_enabled = false;
    gpio_intr_disable(PIN_DRV_NFAULT);
    if (s_fault_cb) s_fault_cb(s_fault_ctx);
}

static esp_err_t set_raw(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t e = ledc_set_duty(PWM_MODE, ch, duty);
    if (e == ESP_OK) e = ledc_update_duty(PWM_MODE, ch);
    return e;
}

static esp_err_t set_pair(drv8833_ch_t ch, uint32_t in1, uint32_t in2)
{
    esp_err_t e1 = set_raw(s_ledc[ch][0], in1);
    esp_err_t e2 = set_raw(s_ledc[ch][1], in2);
    return e1 != ESP_OK ? e1 : e2;
}

esp_err_t drv8833_init(drv8833_fault_cb_t cb, void *ctx)
{
    s_fault_cb = cb;
    s_fault_ctx = ctx;

    /* nSLEEP zuerst LOW festnageln */
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PIN_DRV_NSLEEP,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t e = gpio_config(&out);
    if (e != ESP_OK) return e;
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    s_enabled = false;

    /* nFAULT: externer Pull-up, KEIN interner (würde Bestückungsfehler verdecken) */
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_DRV_NFAULT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    e = gpio_config(&in);
    if (e != ESP_OK) return e;
    gpio_intr_disable(PIN_DRV_NFAULT);
    e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;   /* INVALID_STATE = schon installiert */
    e = gpio_isr_handler_add(PIN_DRV_NFAULT, nfault_isr, NULL);
    if (e != ESP_OK) return e;

    ledc_timer_config_t t = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RES_BITS,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    e = ledc_timer_config(&t);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "LEDC-Timer: %s", esp_err_to_name(e));
        return e;
    }
    for (int c = 0; c < DRV8833_CH_COUNT; c++) {
        for (int k = 0; k < 2; k++) {
            ledc_channel_config_t cc = {
                .gpio_num = s_pins[c][k],
                .speed_mode = PWM_MODE,
                .channel = s_ledc[c][k],
                .timer_sel = PWM_TIMER,
                .duty = 0,
                .hpoint = 0,
            };
            e = ledc_channel_config(&cc);
            if (e != ESP_OK) return e;
        }
    }
    s_ready = true;
    ESP_LOGI(TAG, "bereit: %d kHz, %d Bit, %s Decay, nSLEEP LOW", MOTOR_PWM_FREQ_HZ / 1000,
             MOTOR_PWM_RES_BITS, MOTOR_SLOW_DECAY ? "Slow" : "Fast");
    return ESP_OK;
}

/* Ton: beide IN-Pins auf LEDC-Timer 1 (Hörfrequenz), IN2 um eine halbe Periode verschoben. Dadurch wechseln sich
 * Pulse "vorwärts" und "rückwärts" ab: Mittelwert 0, der Motor bleibt stehen und schwingt nur. */
static bool s_tone_timer_ready;

esp_err_t drv8833_tone(drv8833_ch_t ch, uint32_t freq_hz, uint16_t duty_permille)
{
    if (!s_ready || ch >= DRV8833_CH_COUNT || !s_enabled) return ESP_ERR_INVALID_STATE;
    if (freq_hz < BEEP_FREQ_MIN_HZ || freq_hz > BEEP_FREQ_MAX_HZ || duty_permille == 0) return ESP_ERR_INVALID_ARG;
    if (duty_permille > BEEP_DUTY_MAX_PCT * 10) duty_permille = BEEP_DUTY_MAX_PCT * 10;
    esp_err_t e;
    if (!s_tone_timer_ready) {
        ledc_timer_config_t t = {
            .speed_mode = PWM_MODE,
            .timer_num = LEDC_TIMER_1,
            .duty_resolution = MOTOR_PWM_RES_BITS,
            .freq_hz = freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        e = ledc_timer_config(&t);
        if (e != ESP_OK) return e;
        s_tone_timer_ready = true;
    } else {
        e = ledc_set_freq(PWM_MODE, LEDC_TIMER_1, freq_hz);
        if (e != ESP_OK) return e;
    }
    uint32_t d = ((uint32_t)duty_permille * PWM_FULL + 500) / 1000;
    /* Kanal nur auf den Ton-Timer umhängen (kein erneutes ledc_channel_config: das warnt bei jedem Aufruf
     * "GPIO ... is not usable", weil der Pin schon reserviert ist) */
    for (int k = 0; k < 2; k++) {
        e = ledc_bind_channel_timer(PWM_MODE, s_ledc[ch][k], LEDC_TIMER_1);
        if (e != ESP_OK) return e;
        e = ledc_set_duty_with_hpoint(PWM_MODE, s_ledc[ch][k], d, k ? (PWM_FULL / 2) : 0);
        if (e != ESP_OK) return e;
        e = ledc_update_duty(PWM_MODE, s_ledc[ch][k]);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

esp_err_t drv8833_tone_stop(drv8833_ch_t ch)
{
    if (!s_ready || ch >= DRV8833_CH_COUNT) return ESP_ERR_INVALID_ARG;
    esp_err_t e1 = ESP_OK;
    for (int k = 0; k < 2; k++) {
        esp_err_t e = ledc_set_duty_with_hpoint(PWM_MODE, s_ledc[ch][k], 0, 0);
        if (e == ESP_OK) e = ledc_update_duty(PWM_MODE, s_ledc[ch][k]);
        if (e == ESP_OK) e = ledc_bind_channel_timer(PWM_MODE, s_ledc[ch][k], PWM_TIMER);   /* zurück auf den Fahr-PWM-Timer */
        if (e != ESP_OK) e1 = e;
    }
    return e1;
}

esp_err_t drv8833_enable(bool on)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!on) {
        drv8833_all_off();
        return ESP_OK;
    }
    if (s_enabled) return ESP_OK;
    s_enable_us = esp_timer_get_time();
    gpio_set_level(PIN_DRV_NSLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(DRV8833_WAKE_MS));
    if (drv8833_fault_active()) {
        ESP_LOGE(TAG, "nFAULT LOW direkt nach dem Aufwachen -> Treiber bleibt aus");
        drv8833_all_off();
        return ESP_ERR_INVALID_STATE;
    }
    s_enabled = true;
    s_enable_us = esp_timer_get_time();
    /* Der Pin hat NEGEDGE dauerhaft eingestellt: Flanken, die bei gesperrtem Interrupt
     * (nSLEEP-Übergänge, Einschalten) kamen, bleiben im Statusregister stehen und würden
     * beim Freigeben sofort einen Schein-Fehler auslösen (erster Anlauf nach dem Boot:
     * "nFAULT" nach 10 ms). Deshalb vorher löschen; der Pegel wurde oben schon geprüft. */
    gpio_ll_clear_intr_status(&GPIO, 1UL << PIN_DRV_NFAULT);
    gpio_intr_enable(PIN_DRV_NFAULT);
    return ESP_OK;
}

bool drv8833_is_enabled(void)
{
    return s_enabled;
}

bool drv8833_in_blanking(void)
{
    return (esp_timer_get_time() - s_enable_us) < (int64_t)DRV8833_FAULT_BLANK_MS * 1000;
}

bool drv8833_fault_active(void)
{
    return gpio_get_level(PIN_DRV_NFAULT) == 0;
}

esp_err_t drv8833_drive(drv8833_ch_t ch, bool forward, uint16_t duty_permille)
{
    if (!s_ready || ch >= DRV8833_CH_COUNT) return ESP_ERR_INVALID_ARG;
    const uint16_t max_pm = MOTOR_DUTY_MAX_PCT * 10;
    if (duty_permille > max_pm) duty_permille = max_pm;
    if (duty_permille == 0) return drv8833_coast(ch);

    uint32_t d = ((uint32_t)duty_permille * PWM_FULL + 500) / 1000;
#if MOTOR_SLOW_DECAY
    /* Slow Decay: treibender Pin fest HIGH, anderer Pin invertierte PWM
     * (LOW-Anteil = Antrieb, HIGH-Anteil = Bremsen/Rezirkulation). */
    uint32_t inv = PWM_FULL - d;
    return forward ? set_pair(ch, PWM_FULL, inv) : set_pair(ch, inv, PWM_FULL);
#else
    /* Fast Decay: treibender Pin PWM, anderer LOW (wie hwtest) */
    return forward ? set_pair(ch, d, 0) : set_pair(ch, 0, d);
#endif
}

esp_err_t drv8833_brake(drv8833_ch_t ch)
{
    if (!s_ready || ch >= DRV8833_CH_COUNT) return ESP_ERR_INVALID_ARG;
    return set_pair(ch, PWM_FULL, PWM_FULL);
}

esp_err_t drv8833_coast(drv8833_ch_t ch)
{
    if (!s_ready || ch >= DRV8833_CH_COUNT) return ESP_ERR_INVALID_ARG;
    return set_pair(ch, 0, 0);
}

void drv8833_all_off(void)
{
    gpio_intr_disable(PIN_DRV_NFAULT);
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    s_enabled = false;
    if (s_ready) {
        for (int c = 0; c < DRV8833_CH_COUNT; c++) drv8833_coast((drv8833_ch_t)c);
    }
}
