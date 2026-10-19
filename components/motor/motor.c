#include <stdio.h>
#include "motor.h"
#include "board_pins.h"
#include "drv.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_MAX         ((1 << 10) - 1)

/* je Motor zwei Kanaele: IN1, IN2 */
static const int s_pins[2][2] = {
    { PIN_M1_IN1, PIN_M1_IN2 },
    { PIN_M2_IN1, PIN_M2_IN2 },
};
static const ledc_channel_t s_ch[2][2] = {
    { LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    { LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
};
static bool s_ready;

static void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, ch, duty);
    ledc_update_duty(PWM_MODE, ch);
}

void motor_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) {
        printf("Motor: LEDC-Timer-Init fehlgeschlagen\n");
        return;
    }
    for (int m = 0; m < 2; m++) {
        for (int k = 0; k < 2; k++) {
            ledc_channel_config_t c = {
                .gpio_num = s_pins[m][k],
                .speed_mode = PWM_MODE,
                .channel = s_ch[m][k],
                .timer_sel = PWM_TIMER,
                .duty = 0,
                .hpoint = 0,
            };
            ledc_channel_config(&c);
        }
    }
    s_ready = true;
}

void motor_stop(void)
{
    if (s_ready) {
        for (int m = 0; m < 2; m++)
            for (int k = 0; k < 2; k++)
                set_duty(s_ch[m][k], 0);
    }
    drv_enable(false);
}

motor_result_t motor_run(int id, int dir, int duty_pct, int max_ms, motor_poll_cb_t cb, void *ctx)
{
    if (!s_ready || (id != 1 && id != 2) || (dir != MOTOR_FWD && dir != MOTOR_BACK)) return MOTOR_RUN_ERROR;
    if (duty_pct < 1) duty_pct = 1;
    if (duty_pct > MOTOR_DUTY_MAX) duty_pct = MOTOR_DUTY_MAX;
    if (max_ms < 1) max_ms = 1;
    if (max_ms > MOTOR_RUN_MAX_MS) max_ms = MOTOR_RUN_MAX_MS;

    int m = id - 1;
    ledc_channel_t pwm_ch = dir == MOTOR_FWD ? s_ch[m][0] : s_ch[m][1];

    drv_enable(true);
    if (drv_fault()) { /* schon vor dem Start Fehler */
        motor_stop();
        return MOTOR_RUN_FAULT;
    }

    motor_result_t res = MOTOR_RUN_OK;
    int64_t t0 = esp_timer_get_time();
    set_duty(pwm_ch, (PWM_MAX * duty_pct) / 100);
    for (;;) {
        if (drv_fault()) { res = MOTOR_RUN_FAULT; break; }
        int elapsed = (int)((esp_timer_get_time() - t0) / 1000);
        if (elapsed >= max_ms) break;
        if (cb && cb(elapsed, ctx)) { res = MOTOR_RUN_ABORT; break; }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motor_stop(); /* immer: Duty 0 (IN-Pins LOW), nSLEEP LOW */
    return res;
}

const char *motor_result_str(motor_result_t r)
{
    switch (r) {
    case MOTOR_RUN_OK:    return "Zeit abgelaufen";
    case MOTOR_RUN_ABORT: return "vorzeitig gestoppt";
    case MOTOR_RUN_FAULT: return "FEHLER: nFAULT LOW";
    default:              return "ungueltige Parameter";
    }
}

bool motor_test(int id, int dir, int duty_pct, int ms)
{
    if (duty_pct > MOTOR_DUTY_MAX) duty_pct = MOTOR_DUTY_MAX;
    if (ms > MOTOR_RUN_MAX_MS) ms = MOTOR_RUN_MAX_MS;
    printf("Motor %d: %s, Duty %d %% (max %d %%), %d ms (max %d ms), %d kHz PWM\n", id,
           dir == MOTOR_FWD ? "vor" : "zurueck", duty_pct, MOTOR_DUTY_MAX, ms, MOTOR_RUN_MAX_MS,
           MOTOR_PWM_FREQ_HZ / 1000);
    motor_result_t r = motor_run(id, dir, duty_pct, ms, NULL, NULL);
    bool ok = r == MOTOR_RUN_OK;
    printf("Motor %d: gestoppt (IN-Pins LOW, nSLEEP LOW) - %s -> %s\n", id, motor_result_str(r),
           ok ? "PASS" : "FAIL");
    return ok;
}
