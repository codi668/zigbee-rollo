#include <stdio.h>
#include <stdlib.h>
#include "encoder.h"
#include "board_pins.h"
#include "motor.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCNT_LIMIT          32000
#define GLITCH_NS           3000    /* Impulse < 3 us werden verworfen */
#define TEST_RUN_MS         1000
#define TEST_PRINT_MS       200
#define TEST_MIN_COUNTS     20      /* weniger gilt als "dreht nicht" */
#define STALL_GRACE_MS      300     /* Anlauf-Schonfrist */
#define STALL_NOPULSE_MS    200     /* so lange ohne Impuls = blockiert */
#define STALL_DUTY_DEFAULT  30

static pcnt_unit_handle_t s_unit[2];
static const int s_pin_a[2] = { PIN_ENC1_A, PIN_ENC2_A };
static const int s_pin_b[2] = { PIN_ENC1_B, PIN_ENC2_B };
static const int s_ppr_x1[2] = { ENC1_PPR_X1, ENC2_PPR_X1 };

static bool valid_id(int id) { return id == 1 || id == 2; }

static bool enc_unit_init(int i)
{
    gpio_config_t g = {
        .pin_bit_mask = (1ULL << s_pin_a[i]) | (1ULL << s_pin_b[i]),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&g);

    pcnt_unit_config_t ucfg = { .low_limit = -PCNT_LIMIT, .high_limit = PCNT_LIMIT };
    if (pcnt_new_unit(&ucfg, &s_unit[i]) != ESP_OK) return false;

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = GLITCH_NS };
    if (pcnt_unit_set_glitch_filter(s_unit[i], &fcfg) != ESP_OK) return false;

    /* Quadratur x4: je Kanal beide Flanken, Pegel des anderen Kanals bestimmt die Richtung */
    pcnt_chan_config_t ca = { .edge_gpio_num = s_pin_a[i], .level_gpio_num = s_pin_b[i] };
    pcnt_chan_config_t cb = { .edge_gpio_num = s_pin_b[i], .level_gpio_num = s_pin_a[i] };
    pcnt_channel_handle_t cha, chb;
    if (pcnt_new_channel(s_unit[i], &ca, &cha) != ESP_OK) return false;
    if (pcnt_new_channel(s_unit[i], &cb, &chb) != ESP_OK) return false;
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    if (pcnt_unit_enable(s_unit[i]) != ESP_OK) return false;
    pcnt_unit_clear_count(s_unit[i]);
    return pcnt_unit_start(s_unit[i]) == ESP_OK;
}

bool enc_init(void)
{
    for (int i = 0; i < 2; i++) {
        if (!enc_unit_init(i)) {
            printf("Encoder %d: PCNT-Init fehlgeschlagen\n", i + 1);
            return false;
        }
    }
    return true;
}

void enc_clear(int id)
{
    if (valid_id(id)) pcnt_unit_clear_count(s_unit[id - 1]);
}

int enc_count(int id)
{
    int c = 0;
    if (valid_id(id)) pcnt_unit_get_count(s_unit[id - 1], &c);
    return c;
}

int enc_counts_per_rev(int id)
{
    return valid_id(id) ? s_ppr_x1[id - 1] * 4 : 0;
}

/* Umdrehungen pro Minute aus Counts in dt_ms (vorzeichenbehaftet); 0 wenn cpr unbekannt */
static float rpm_from(int id, int counts, int dt_ms)
{
    int cpr = enc_counts_per_rev(id);
    if (cpr <= 0 || dt_ms <= 0) return 0.0f;
    return (float)counts / cpr / (dt_ms / 60000.0f);
}

static void print_cpr_note(int id)
{
    int cpr = enc_counts_per_rev(id);
    if (cpr > 0)
        printf("Encoder %d: %d Counts/Abtriebsumdrehung (x4, PPR_X1 = %d)\n", id, cpr, cpr / 4);
    else
        printf("Encoder %d: Impulse/Umdrehung unbekannt (ENC%d_PPR_X1 = 0, TODO in board_pins.h) - nur Counts, kein rpm\n",
               id, id);
}

typedef struct {
    int id;
    int last_print_ms;
    int last_count;
    bool stall_mode;
    int last_change_ms;
    int stall_last_count;
    bool stalled;
} run_ctx_t;

static bool run_poll(int elapsed_ms, void *vctx)
{
    run_ctx_t *c = vctx;
    int cnt = enc_count(c->id);

    if (c->stall_mode) {
        if (cnt != c->stall_last_count) {
            c->stall_last_count = cnt;
            c->last_change_ms = elapsed_ms;
        }
        if (elapsed_ms >= STALL_GRACE_MS && elapsed_ms - c->last_change_ms >= STALL_NOPULSE_MS) {
            c->stalled = true;
            return true;
        }
    }
    if (elapsed_ms - c->last_print_ms >= TEST_PRINT_MS) {
        int dt = elapsed_ms - c->last_print_ms;
        int d = cnt - c->last_count;
        printf("  t=%4d ms: Counts %7d (%+6d in %d ms)", elapsed_ms, cnt, d, dt);
        if (enc_counts_per_rev(c->id) > 0)
            printf("  %+7.1f rpm Abtrieb", rpm_from(c->id, d, dt));
        printf("\n");
        c->last_print_ms = elapsed_ms;
        c->last_count = cnt;
    }
    return false;
}

static bool one_run(int id, int dir, int duty_pct, int *counts_out)
{
    run_ctx_t ctx = { .id = id };
    enc_clear(id);
    int64_t t0 = esp_timer_get_time();
    motor_result_t r = motor_run(id, dir, duty_pct, TEST_RUN_MS, run_poll, &ctx);
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    vTaskDelay(pdMS_TO_TICKS(100)); /* Ausrollen abwarten, Nachlaeufer mitzaehlen */
    int total = enc_count(id);
    *counts_out = total;
    printf("  Ergebnis %s: %d Counts in %d ms (%s)", dir == MOTOR_FWD ? "vor" : "zurueck", total, ms,
           motor_result_str(r));
    if (enc_counts_per_rev(id) > 0)
        printf(", mittel %+.1f rpm, %+.2f Umdr.", rpm_from(id, total, ms), (float)total / enc_counts_per_rev(id));
    printf("\n");
    return r == MOTOR_RUN_OK;
}

bool enc_test(int id, int duty_pct)
{
    if (!valid_id(id)) { printf("Encoder: ungueltiger Motor/Encoder (1 oder 2)\n"); return false; }
    if (duty_pct <= 0) duty_pct = MOTOR_DUTY_DEFAULT;
    if (duty_pct > MOTOR_DUTY_MAX) duty_pct = MOTOR_DUTY_MAX;
    printf("Encoder %d: Motor %d faehrt 2x %d ms (vor, dann zurueck) mit %d %% Duty\n", id, id, TEST_RUN_MS, duty_pct);
    print_cpr_note(id);

    int cf = 0, cb = 0;
    printf(" Fahrt vor:\n");
    bool ok_f = one_run(id, MOTOR_FWD, duty_pct, &cf);
    vTaskDelay(pdMS_TO_TICKS(300));
    printf(" Fahrt zurueck:\n");
    bool ok_b = one_run(id, MOTOR_BACK, duty_pct, &cb);

    bool moved = abs(cf) >= TEST_MIN_COUNTS && abs(cb) >= TEST_MIN_COUNTS;
    bool reversed = (cf > 0 && cb < 0) || (cf < 0 && cb > 0);
    bool ok = ok_f && ok_b && moved && reversed;
    printf("Encoder %d: vor %+d / zurueck %+d Counts -> %s\n", id, cf, cb,
           ok ? "PASS (Zaehlrichtung kehrt mit der Drehrichtung um)"
              : !(ok_f && ok_b) ? "FAIL (Motorfahrt mit Fehler abgebrochen)"
              : !moved ? "FAIL (zu wenige Impulse: Encoder-Versorgung/A/B-Leitungen, Motor dreht nicht?)"
                       : "FAIL (gleiche Zaehlrichtung in beiden Richtungen - A/B pruefen)");
    if (ok)
        printf("Encoder %d: Hinweis - bei Fahrt 'vor' zaehlt der Encoder %s\n", id,
               cf > 0 ? "positiv" : "negativ (A/B vertauschen, falls positiv gewuenscht)");
    return ok;
}

bool enc_hand_test(int id, int duration_s)
{
    if (!valid_id(id)) { printf("Encoder: ungueltiger Encoder (1 oder 2)\n"); return false; }
    if (duration_s < 1) duration_s = 10;
    motor_stop();
    printf("Encoder %d (Handmodus, Motor aus): %d s lang Abtriebswelle von Hand drehen\n", id, duration_s);
    print_cpr_note(id);
    enc_clear(id);
    int last = 0, min_c = 0, max_c = 0;
    for (int t = 0; t < duration_s * 1000; t += 250) {
        vTaskDelay(pdMS_TO_TICKS(250));
        int c = enc_count(id);
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;
        if (c != last) {
            printf("  t=%5.2f s: Counts %7d", (t + 250) / 1000.0f, c);
            if (enc_counts_per_rev(id) > 0) printf("  (%+.3f Umdr.)", (float)c / enc_counts_per_rev(id));
            printf("\n");
            last = c;
        }
    }
    int c = enc_count(id);
    printf("Encoder %d: Endstand %d Counts (min %d, max %d), Counts/4 = %.1f x1-Impulse\n", id, c, min_c, max_c, c / 4.0f);
    bool ok = min_c != 0 || max_c != 0;
    printf("Encoder %d: %s\n", id,
           ok ? "PASS (Impulse gezaehlt; Vorzeichen = Drehrichtung)"
              : "FAIL (keine Impulse - nicht gedreht oder A/B/Versorgung defekt)");
    return ok;
}

bool stall_test(int id, int duty_pct)
{
    if (!valid_id(id)) { printf("Blockiererkennung: ungueltiger Motor (1 oder 2)\n"); return false; }
    if (duty_pct <= 0) duty_pct = STALL_DUTY_DEFAULT;
    if (duty_pct > MOTOR_DUTY_MAX) duty_pct = MOTOR_DUTY_MAX;
    printf("Blockiererkennung Motor %d: Duty %d %%, max %d ms, Schonfrist %d ms, Stopp bei %d ms ohne Impuls\n",
           id, duty_pct, MOTOR_RUN_MAX_MS, STALL_GRACE_MS, STALL_NOPULSE_MS);
    printf("  (Welle zum Testen kurz festhalten; Motor stoppt dann selbst)\n");
    print_cpr_note(id);

    run_ctx_t ctx = { .id = id, .stall_mode = true };
    enc_clear(id);
    int64_t t0 = esp_timer_get_time();
    motor_result_t r = motor_run(id, MOTOR_FWD, duty_pct, MOTOR_RUN_MAX_MS, run_poll, &ctx);
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    printf("Blockiererkennung Motor %d: nach %d ms gestoppt, %d Counts - ", id, ms, enc_count(id));
    bool ok = r == MOTOR_RUN_OK || (r == MOTOR_RUN_ABORT && ctx.stalled);
    if (ctx.stalled)
        printf("blockiert (letzter Impuls bei %d ms) -> PASS (Blockierung erkannt, Motor gestoppt)\n", ctx.last_change_ms);
    else if (r == MOTOR_RUN_OK)
        printf("nicht blockiert, Zeitlimit erreicht -> PASS (Motor lief mit Impulsen; zum Pruefen der Erkennung Welle festhalten)\n");
    else
        printf("%s -> FAIL\n", motor_result_str(r));
    return ok;
}
