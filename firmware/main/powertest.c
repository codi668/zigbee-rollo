/*
 * Leistungstest (nur mit -DROLLO_PT=1): Messung des Stroms am Akku ohne USB, daher ohne Konsole.
 * Jeder Zustand läuft PT_STATE_MS, dann wird neu gestartet (Zustand im RTC-Speicher) bzw. in Deep Sleep
 * gegangen. Die LED blinkt zu Beginn so oft, wie die Zustandsnummer ist (danach aus).
 *
 *  1  Funk dauernd empfangsbereit (Standard-Firmware)
 *  2  Sleepy End Device (Funk schläft zwischen den Polls), CPU aktiv
 *  3  Sleepy + automatischer Light Sleep
 *  4  ohne Funk, CPU aktiv
 *  5  ohne Funk, Light Sleep
 *  6  Deep Sleep (Boden: Wandler, Teiler, Encoder, ESP im Tiefschlaf)
 *  7  Sleepy + DRV8833 wach, Motor steht
 *  8  Sleepy + Motor 2 fährt hin und her (Anlauf/Betrieb)
 *  9  Ende (Deep Sleep, bis der Akku abgesteckt wird)
 */
#include "powertest.h"
#include "board_pins.h"
#include "config.h"
#include "drv8833.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ctrl.h"

static const char *TAG = "powertest";

#define PT_MAGIC  0x50540A01u
#define PT_LAST   9

static RTC_NOINIT_ATTR uint32_t s_magic;
static RTC_NOINIT_ATTR uint8_t  s_idx;

static void led_set(bool on) { gpio_set_level(PIN_LED, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL); }

static void led_blink(int n)
{
    gpio_config_t o = { .pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    led_set(false);
    vTaskDelay(pdMS_TO_TICKS(600));
    for (int i = 0; i < n; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set(false);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void next_state_and_restart(void *arg)
{
    (void)arg;
    s_idx++;
    esp_restart();
}

static void configure_pm(void)
{
    esp_pm_config_t pm = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    esp_err_t e = esp_pm_configure(&pm);
    if (e != ESP_OK) ESP_LOGE(TAG, "esp_pm_configure: %s", esp_err_to_name(e));
}

void pt_begin(pt_cfg_t *c)
{
    if (s_magic != PT_MAGIC || s_idx < 1 || s_idx > PT_LAST) {
        s_magic = PT_MAGIC;
        s_idx = 1;
    }
    *c = (pt_cfg_t){ .idx = s_idx, .full = true, .zigbee = true, .name = "?" };
    switch (s_idx) {
    case 1: c->name = "Funk RX dauernd"; break;
    case 2: c->sleepy = true; c->name = "Sleepy, CPU aktiv"; break;
    case 3: c->sleepy = true; c->pm = true; c->name = "Sleepy + Light Sleep"; break;
    case 4: c->full = false; c->zigbee = false; c->name = "ohne Funk, CPU aktiv"; break;
    case 5: c->full = false; c->zigbee = false; c->pm = true; c->name = "ohne Funk, Light Sleep"; break;
    case 6: c->full = false; c->zigbee = false; c->name = "Deep Sleep"; break;
    case 7: c->sleepy = true; c->drv_awake = true; c->name = "Sleepy + DRV8833 wach"; break;
    case 8: c->sleepy = true; c->motor = true; c->name = "Sleepy + Motor faehrt"; break;
    default: c->full = false; c->zigbee = false; c->name = "Ende (Deep Sleep)"; break;
    }
    led_blink(c->idx);
    ESP_LOGW(TAG, "Zustand %u: %s (%d s)", c->idx, c->name, PT_STATE_MS / 1000);
    if (c->idx <= 5 || c->idx == 7 || c->idx == 8) {
        static esp_timer_handle_t t;
        const esp_timer_create_args_t a = { .callback = next_state_and_restart, .name = "pt_next" };
        esp_timer_create(&a, &t);
        esp_timer_start_once(t, (uint64_t)PT_STATE_MS * 1000);
    }
}

void pt_run_minimal(const pt_cfg_t *c)
{
    /* nur nötige Pins: Motortreiber aus (nSLEEP LOW), kein Funk */
    gpio_config_t o = { .pin_bit_mask = 1ULL << PIN_DRV_NSLEEP, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    gpio_set_level(PIN_DRV_NSLEEP, 0);
    if (c->idx == 6 || c->idx >= PT_LAST) {
        if (c->idx == 6) {
            s_idx = 7;                                   /* nach dem Aufwachen weiter mit Zustand 7 */
            esp_sleep_enable_timer_wakeup((uint64_t)PT_STATE_MS * 1000);
        } else {
            s_idx = PT_LAST;                             /* Ende: bis zum Abstecken schlafen */
        }
        esp_deep_sleep_start();                          /* nSLEEP bleibt über den Pull-down im DRV8833 LOW */
    }
    if (c->pm) configure_pm();
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));            /* Zustand 4/5: der Timer aus pt_begin() startet neu */
}

static void motor_task(void *arg)
{
    (void)arg;
    int32_t d = 2100;                                    /* halbe Umdrehung */
    for (;;) {
        motor_ctrl_move_by(MOTOR_TILT, d, 0);
        d = -d;
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

void pt_after_init(const pt_cfg_t *c)
{
    if (c->drv_awake) {
        esp_err_t e = drv8833_enable(true);              /* nSLEEP HIGH, kein PWM */
        ESP_LOGW(TAG, "DRV8833 wach: %s", esp_err_to_name(e));
    }
    if (c->motor) xTaskCreate(motor_task, "pt_motor", 3072, NULL, 4, NULL);
    if (c->pm) {
        vTaskDelay(pdMS_TO_TICKS(8000));                 /* erst dem Funk Zeit zum Verbinden */
        configure_pm();
    }
}
