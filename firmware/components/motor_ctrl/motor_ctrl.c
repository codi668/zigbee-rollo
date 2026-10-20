/*
 * motor_ctrl: motor_task, Befehlsqueue, Umsetzung von motion_tick() auf den DRV8833,
 * Ereignisse an die Applikation. Die Fahrlogik selbst steckt in motion.c (Host-getestet).
 */
#include "motor_ctrl.h"
#include "motion.h"
#include "drv8833.h"
#include "encoder.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "motor_ctrl";

#define PCT2PM(p) ((uint16_t)((p) * 10))

/* ---- Parameter je Motor aus config.h ---- */
static const motion_params_t s_params[MOTOR_COUNT] = {
    [MOTOR_LIFT] = {
        .min_duty = PCT2PM(M1_MIN_DUTY_PCT), .kick_duty = PCT2PM(M1_KICK_DUTY_PCT),
        .run_duty = PCT2PM(M1_RUN_DUTY_PCT), .kick_ms = M1_KICK_MS, .kick_steps = M1_KICK_STEPS,
        .ramp_up_ms = M1_RAMP_UP_MS, .ramp_down_ms = M1_RAMP_DOWN_MS,
        .brake_zone_steps = M1_BRAKE_ZONE_STEPS, .stop_tol_steps = M1_STOP_TOL_STEPS,
        .brake_ms = MOTOR_BRAKE_MS, .dir_pause_ms = MOTOR_DIR_PAUSE_MS,
        .block_window_ms = M1_BLOCK_WINDOW_MS, .block_min_steps = M1_BLOCK_MIN_STEPS,
        .wrong_dir_steps = M1_WRONG_DIR_STEPS, .timeout_max_ms = MOTOR_TIMEOUT_MAX_MS,
    },
    [MOTOR_TILT] = {
        .min_duty = PCT2PM(M2_MIN_DUTY_PCT), .kick_duty = PCT2PM(M2_KICK_DUTY_PCT),
        .run_duty = PCT2PM(M2_RUN_DUTY_PCT), .kick_ms = M2_KICK_MS, .kick_steps = M2_KICK_STEPS,
        .ramp_up_ms = M2_RAMP_UP_MS, .ramp_down_ms = M2_RAMP_DOWN_MS,
        .brake_zone_steps = M2_BRAKE_ZONE_STEPS, .stop_tol_steps = M2_STOP_TOL_STEPS,
        .brake_ms = MOTOR_BRAKE_MS, .dir_pause_ms = MOTOR_DIR_PAUSE_MS,
        .block_window_ms = M2_BLOCK_WINDOW_MS, .block_min_steps = M2_BLOCK_MIN_STEPS,
        .wrong_dir_steps = M2_WRONG_DIR_STEPS, .timeout_max_ms = MOTOR_TIMEOUT_MAX_MS,
    },
};
static const uint32_t s_timeout_default[MOTOR_COUNT] = { M1_TIMEOUT_DEFAULT_MS, M2_TIMEOUT_DEFAULT_MS };
static const bool s_dir_invert[MOTOR_COUNT] = { M1_DIR_INVERT, M2_DIR_INVERT };
static const uint16_t s_margin_top[MOTOR_COUNT] = { LIFT_MARGIN_TOP_PERMILLE, TILT_MARGIN_TOP_PERMILLE };
static const uint16_t s_margin_bottom[MOTOR_COUNT] = { LIFT_MARGIN_BOTTOM_PERMILLE, TILT_MARGIN_BOTTOM_PERMILLE };
static const drv8833_ch_t s_ch[MOTOR_COUNT] = { DRV8833_CH_A, DRV8833_CH_B };
static const encoder_id_t s_enc[MOTOR_COUNT] = { ENCODER_1, ENCODER_2 };

_Static_assert(M1_BLOCK_WINDOW_MS / MOTOR_TICK_MS + 2 <= BLOCK_DET_MAX_SAMPLES, "M1_BLOCK_WINDOW_MS zu groß");
_Static_assert(M2_BLOCK_WINDOW_MS / MOTOR_TICK_MS + 2 <= BLOCK_DET_MAX_SAMPLES, "M2_BLOCK_WINDOW_MS zu groß");
_Static_assert(M1_RUN_DUTY_PCT <= MOTOR_DUTY_MAX_PCT && M2_RUN_DUTY_PCT <= MOTOR_DUTY_MAX_PCT, "Duty > Max");
_Static_assert(M1_KICK_DUTY_PCT <= MOTOR_DUTY_MAX_PCT && M2_KICK_DUTY_PCT <= MOTOR_DUTY_MAX_PCT, "Duty > Max");

/* ---- Befehle an die motor_task ---- */
typedef enum { CMD_GOTO, CMD_MOVE_BY, CMD_RUN, CMD_STOP, CMD_STOP_ALL, CMD_FAULT } cmd_type_t;
typedef struct {
    cmd_type_t type;
    motor_id_t id;
    int32_t    value;       /* Ziel, Delta, Richtung; bei STOP: 1 = es waren Befehle offen */
    uint32_t   timeout_ms;
    uint32_t   seq;         /* Stopp-Sequenz beim Absenden (veraltete Fahrbefehle verwerfen) */
} motor_cmd_t;

/* ---- Modulzustand ---- */
static QueueHandle_t s_queue;
static motor_event_cb_t s_cb;
static void *s_cb_ctx;
static esp_pm_lock_handle_t s_pm_lock;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static axis_t s_axis[MOTOR_COUNT];          /* Endpunkte; Zugriff unter s_lock */
static uint8_t s_pending[MOTOR_COUNT];      /* Fahrbefehle in der Queue, unter s_lock */
static uint32_t s_stop_seq[MOTOR_COUNT];    /* +1 bei jedem Stopp, unter s_lock */
static volatile bool s_active[MOTOR_COUNT]; /* von der motor_task gepflegt */
static volatile bool s_fault_latched;       /* nach nFAULT: keine Fahrten bis motor_ctrl_clear_fault() */

/* nur von der motor_task benutzt */
static motion_t s_motion[MOTOR_COUNT];
static uint32_t s_last_progress[MOTOR_COUNT];

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static bool valid(motor_id_t id) { return (unsigned)id < MOTOR_COUNT; }

/* ------------------------------------------------------------------------- */

static void emit(motor_event_type_t type, motor_id_t id, int32_t pos, int32_t goal, uint32_t elapsed)
{
    motor_event_t e = { .type = type, .id = id, .pos = pos, .goal = goal, .elapsed_ms = elapsed };
    if (type != MOTOR_EVT_PROGRESS)
        ESP_LOGI(TAG, "Motor %d: %s, Pos %ld, Ziel %ld, %lu ms", id + 1, motor_event_str(type),
                 (long)pos, (long)goal, (unsigned long)elapsed);
    if (s_cb) s_cb(&e, s_cb_ctx);
}

static motor_event_type_t result_to_evt(motion_result_t r)
{
    switch (r) {
    case MOTION_RES_DONE:      return MOTOR_EVT_DONE;
    case MOTION_RES_LIMIT:     return MOTOR_EVT_LIMIT;
    case MOTION_RES_STOPPED:   return MOTOR_EVT_STOPPED;
    case MOTION_RES_BLOCKED:   return MOTOR_EVT_BLOCKED;
    case MOTION_RES_TIMEOUT:   return MOTOR_EVT_TIMEOUT;
    case MOTION_RES_WRONG_DIR: return MOTOR_EVT_WRONG_DIR;
    default:                   return MOTOR_EVT_FAULT;
    }
}

static void IRAM_ATTR fault_isr_cb(void *ctx)
{
    (void)ctx;
    motor_cmd_t c = { .type = CMD_FAULT };
    BaseType_t woken = pdFALSE;
    if (s_queue) xQueueSendToFrontFromISR(s_queue, &c, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static bool any_active(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        if (motion_is_active(&s_motion[i])) return true;
    return false;
}

static void handle_fault(uint32_t t)
{
    drv8833_all_off();
    /* Sperre setzen und alle noch wartenden Fahrbefehle verwerfen (Sequenz +1),
     * damit der Treiber nach einem Hardwarefehler nicht durch einen schon
     * eingereihten Befehl wieder eingeschaltet wird. */
    bool had_pending[MOTOR_COUNT];
    taskENTER_CRITICAL(&s_lock);
    s_fault_latched = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        had_pending[i] = s_pending[i] != 0;
        s_pending[i] = 0;
        s_stop_seq[i]++;
    }
    taskEXIT_CRITICAL(&s_lock);
    bool any = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motion_is_active(&s_motion[i])) {
            /* Achse stand, hatte aber einen Befehl offen: trotzdem genau ein Abschluss */
            if (had_pending[i]) emit(MOTOR_EVT_FAULT, (motor_id_t)i, encoder_get(s_enc[i]), 0, 0);
            continue;
        }
        any = true;
        uint32_t el = motion_job_elapsed(&s_motion[i], t);
        int32_t goal = s_motion[i].job.has_goal ? s_motion[i].job.goal : 0;
        motion_abort(&s_motion[i], t);
        s_active[i] = false;
        emit(MOTOR_EVT_FAULT, (motor_id_t)i, encoder_get(s_enc[i]), goal, el);
    }
    if (!any) ESP_LOGW(TAG, "nFAULT ohne laufende Fahrt");
}

static void handle_cmd(const motor_cmd_t *c)
{
    uint32_t t = now_ms();
    if (c->type == CMD_FAULT) {
        handle_fault(t);
        return;
    }
    if (c->type == CMD_STOP_ALL || c->type == CMD_STOP) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (c->type == CMD_STOP && (motor_id_t)i != c->id) continue;
            bool had_pending = c->type == CMD_STOP ? c->value != 0 : ((c->value >> i) & 1);
            /* Achse stand, aber ein Fahrbefehl war noch unterwegs (wird jetzt verworfen):
             * trotzdem genau ein Abschluss-Ereignis liefern */
            if (!motion_stop(&s_motion[i], t) && had_pending)
                emit(MOTOR_EVT_STOPPED, (motor_id_t)i, encoder_get(s_enc[i]), 0, 0);
            s_active[i] = motion_is_active(&s_motion[i]);
        }
        return;
    }
    motor_id_t id = c->id;
    if (!valid(id)) return;
    motion_t *m = &s_motion[id];

    /* Fahrbefehl; vor einem späteren Stopp abgeschickt -> verwerfen */
    taskENTER_CRITICAL(&s_lock);
    bool stale = c->seq != s_stop_seq[id];
    if (!stale && s_pending[id]) {
        s_pending[id]--;
        s_active[id] = true;     /* lückenlos "busy": pending -> active im selben Lock */
    }
    axis_t ax = s_axis[id];
    taskEXIT_CRITICAL(&s_lock);
    if (stale) return;

    int32_t pos = encoder_get(s_enc[id]);
    motor_id_t other = id == MOTOR_LIFT ? MOTOR_TILT : MOTOR_LIFT;
    if (s_fault_latched || (!MOTOR_ALLOW_PARALLEL && motion_is_active(&s_motion[other]))) {
        s_active[id] = motion_is_active(m);
        emit(s_fault_latched ? MOTOR_EVT_FAULT : MOTOR_EVT_REJECTED, id, pos, 0, 0);
        return;
    }
    motion_set_limits(m, ax.calibrated, axis_limit_lo(&ax), axis_limit_hi(&ax));
    uint32_t to = c->timeout_ms ? c->timeout_ms : s_timeout_default[id];
    bool was_active = motion_is_active(m);
    switch (c->type) {
    case CMD_GOTO:    motion_goto(m, t, pos, c->value, to); break;
    case CMD_MOVE_BY: motion_goto(m, t, pos, pos + c->value, to); break;
    case CMD_RUN:     motion_run(m, t, pos, c->value >= 0 ? 1 : -1, to); break;
    default: break;
    }
    if (!was_active) s_last_progress[id] = t;
    s_active[id] = motion_is_active(m);
}

static void apply(motor_id_t id, const motion_out_t *o)
{
    switch (o->drive) {
    case MOTION_OUT_DRIVE: {
        bool fwd = (o->dir > 0) != s_dir_invert[id];
        drv8833_drive(s_ch[id], fwd, o->duty);
        break;
    }
    case MOTION_OUT_BRAKE:
        drv8833_brake(s_ch[id]);
        break;
    default:
        drv8833_coast(s_ch[id]);
        break;
    }
}

/* ---- Messspur (Konsole "trace"): Position/Duty alle 20 ms ins Log, zum Auswerten in tools/rollo-debug.py.
 * Schaltet sich selbst nach der angegebenen Zeit ab, damit eine vergessene Spur den Motor-Task nicht
 * dauerhaft mit Log-Ausgaben belastet. */
static volatile int64_t s_trace_until_us;
static const char *TAG_TR = "tr";

void motor_ctrl_trace(uint32_t seconds)
{
    s_trace_until_us = seconds ? esp_timer_get_time() + (int64_t)seconds * 1000000 : 0;
}

static bool trace_on(void)
{
    return s_trace_until_us && esp_timer_get_time() < s_trace_until_us;
}

static void trace_sample(motor_id_t id, uint16_t duty)
{
    ESP_LOGI(TAG_TR, "%d %lu %ld %u", id + 1, (unsigned long)now_ms(), (long)encoder_get(s_enc[id]), duty);
}

/** Nach Fahrtende noch kurz mitschreiben (Nachlauf/Ausschwingen) */
static void trace_settle(void)
{
    for (int k = 0; k < 20 && trace_on(); k++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        for (int i = 0; i < MOTOR_COUNT; i++) trace_sample((motor_id_t)i, 0);
    }
}

static void motor_task(void *arg)
{
    (void)arg;
    bool running = false;          /* Treiber an, PM-Lock gehalten, WDT angemeldet */
    TickType_t last_wake = xTaskGetTickCount();
    motor_cmd_t c;

    for (;;) {
        /* Im Stillstand blockierend auf Befehle warten, während der Fahrt nur abholen.
         * Solange "running" noch gesetzt ist, NICHT blockieren: erst muss unten der
         * Treiber aus, der PM-Lock frei und die Task beim Watchdog abgemeldet werden
         * (sonst bliebe nSLEEP HIGH und der Task-WDT löste nach 5 s aus). */
        TickType_t wait = (any_active() || running) ? 0 : portMAX_DELAY;
        while (xQueueReceive(s_queue, &c, wait) == pdTRUE) {
            handle_cmd(&c);
            wait = 0;
        }

        if (!any_active()) {
            if (running) {
                drv8833_all_off();
                if (s_pm_lock) esp_pm_lock_release(s_pm_lock);
                esp_task_wdt_delete(NULL);
                running = false;
                ESP_LOGD(TAG, "Stillstand, Treiber aus");
                if (trace_on()) trace_settle();
            }
            continue;
        }

        if (!running) {
            if (s_pm_lock) esp_pm_lock_acquire(s_pm_lock);
            esp_task_wdt_add(NULL);
            running = true;
            last_wake = xTaskGetTickCount();
            if (drv8833_enable(true) != ESP_OK) {
                handle_fault(now_ms());    /* nFAULT schon beim Aufwachen LOW */
                continue;
            }
        }
        esp_task_wdt_reset();

        /* nFAULT zusätzlich zum Interrupt in jedem Takt abfragen */
        if (!drv8833_is_enabled() || (drv8833_fault_active() && !drv8833_in_blanking())) {
            handle_fault(now_ms());
            continue;
        }

        uint32_t t = now_ms();
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motion_t *m = &s_motion[i];
            if (!motion_is_active(m)) continue;
            int32_t pos = encoder_get(s_enc[i]);
            motion_out_t o = motion_tick(m, t, pos);
            apply((motor_id_t)i, &o);
            if (trace_on()) trace_sample((motor_id_t)i, o.drive == MOTION_OUT_DRIVE ? o.duty : 0);
            if (o.result != MOTION_RES_NONE) {
                s_active[i] = motion_is_active(m);   /* Folgeauftrag möglich */
                emit(result_to_evt(o.result), (motor_id_t)i, pos, o.goal, o.elapsed_ms);
            } else if (o.drive == MOTION_OUT_DRIVE && (uint32_t)(t - s_last_progress[i]) >= MOTOR_PROGRESS_MS) {
                s_last_progress[i] = t;
                emit(MOTOR_EVT_PROGRESS, (motor_id_t)i, pos, m->job.has_goal ? m->job.goal : 0,
                     motion_job_elapsed(m, t));
            }
            s_active[i] = motion_is_active(m);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_TICK_MS));
    }
}

/* ------------------------------------------------------------------------- */

esp_err_t motor_ctrl_init(motor_event_cb_t cb, void *ctx)
{
    if (s_queue) return ESP_ERR_INVALID_STATE;
    s_cb = cb;
    s_cb_ctx = ctx;
    s_queue = xQueueCreate(MOTOR_QUEUE_LEN, sizeof(motor_cmd_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    esp_err_t e = drv8833_init(fault_isr_cb, NULL);
    if (e != ESP_OK) return e;
    e = encoder_init();
    if (e != ESP_OK) {
        drv8833_all_off();
        return e;
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motion_init(&s_motion[i], &s_params[i]);
        axis_clear(&s_axis[i]);
        s_axis[i].pos = 0;
        s_pending[i] = 0;
        s_active[i] = false;
    }

    e = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "motor", &s_pm_lock);
    if (e != ESP_OK) {
        s_pm_lock = NULL;   /* ohne CONFIG_PM_ENABLE: kein Light Sleep, Lock unnötig */
        ESP_LOGW(TAG, "PM-Lock nicht verfuegbar (%s)", esp_err_to_name(e));
    }

    if (xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK, NULL, MOTOR_TASK_PRIO, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "bereit (Takt %d ms, Motoren aus)", MOTOR_TICK_MS);
    return ESP_OK;
}

static esp_err_t send_motion(motor_cmd_t *c)
{
    if (!s_queue || !valid(c->id)) return ESP_ERR_INVALID_ARG;
    if (s_fault_latched) return ESP_ERR_INVALID_STATE;    /* erst motor_ctrl_clear_fault() */
    motor_id_t other = c->id == MOTOR_LIFT ? MOTOR_TILT : MOTOR_LIFT;
    if (!MOTOR_ALLOW_PARALLEL && motor_ctrl_is_busy(other)) return ESP_ERR_INVALID_STATE;
    taskENTER_CRITICAL(&s_lock);
    s_pending[c->id]++;
    c->seq = s_stop_seq[c->id];
    taskEXIT_CRITICAL(&s_lock);
    if (xQueueSend(s_queue, c, pdMS_TO_TICKS(20)) != pdTRUE) {
        taskENTER_CRITICAL(&s_lock);
        if (c->seq == s_stop_seq[c->id] && s_pending[c->id]) s_pending[c->id]--;
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t motor_ctrl_goto(motor_id_t id, int32_t goal_steps, uint32_t timeout_ms)
{
    motor_cmd_t c = { .type = CMD_GOTO, .id = id, .value = goal_steps, .timeout_ms = timeout_ms };
    return send_motion(&c);
}

esp_err_t motor_ctrl_goto_percent(motor_id_t id, uint8_t percent, uint32_t timeout_ms)
{
    if (!valid(id)) return ESP_ERR_INVALID_ARG;
    axis_t ax;
    motor_ctrl_get_axis(id, &ax);
    if (!ax.calibrated) return ESP_ERR_INVALID_STATE;
    return motor_ctrl_goto(id, axis_percent_to_steps(&ax, percent), timeout_ms);
}

esp_err_t motor_ctrl_move_by(motor_id_t id, int32_t delta_steps, uint32_t timeout_ms)
{
    motor_cmd_t c = { .type = CMD_MOVE_BY, .id = id, .value = delta_steps, .timeout_ms = timeout_ms };
    return send_motion(&c);
}

esp_err_t motor_ctrl_run(motor_id_t id, motor_dir_t dir, uint32_t timeout_ms)
{
    motor_cmd_t c = { .type = CMD_RUN, .id = id, .value = dir, .timeout_ms = timeout_ms };
    return send_motion(&c);
}

esp_err_t motor_ctrl_stop(motor_id_t id)
{
    if (!s_queue || !valid(id)) return ESP_ERR_INVALID_ARG;
    motor_cmd_t c = { .type = CMD_STOP, .id = id };
    /* Stopp hat Vorrang: an den Queue-Anfang; vorher abgeschickte Fahrbefehle
     * werden über die Sequenznummer verworfen */
    taskENTER_CRITICAL(&s_lock);
    s_stop_seq[id]++;
    c.value = s_pending[id] ? 1 : 0;
    s_pending[id] = 0;
    taskEXIT_CRITICAL(&s_lock);
    if (xQueueSendToFront(s_queue, &c, pdMS_TO_TICKS(50)) != pdTRUE) {
        /* Queue klemmt: Notaus direkt am Treiber (beide Achsen), die Task meldet dann FAULT */
        ESP_LOGE(TAG, "Stopp Motor %d nicht zustellbar -> Treiber aus", id + 1);
        drv8833_all_off();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void motor_ctrl_stop_all(void)
{
    if (!s_queue) return;
    motor_cmd_t c = { .type = CMD_STOP_ALL };
    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_stop_seq[i]++;
        if (s_pending[i]) c.value |= 1 << i;
        s_pending[i] = 0;
    }
    taskEXIT_CRITICAL(&s_lock);
    if (xQueueSendToFront(s_queue, &c, pdMS_TO_TICKS(50)) != pdTRUE) {
        /* Queue klemmt: Notaus direkt am Treiber, die Task meldet dann FAULT */
        ESP_LOGE(TAG, "Stopp nicht zustellbar -> Treiber aus");
        drv8833_all_off();
    }
}

bool motor_ctrl_fault_latched(void)
{
    return s_fault_latched;
}

esp_err_t motor_ctrl_clear_fault(void)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    if (motor_ctrl_any_busy()) return ESP_ERR_INVALID_STATE;
    if (drv8833_fault_active() && drv8833_is_enabled()) return ESP_ERR_INVALID_STATE;
    s_fault_latched = false;
    ESP_LOGI(TAG, "nFAULT quittiert");
    return ESP_OK;
}

bool motor_ctrl_is_busy(motor_id_t id)
{
    if (!valid(id)) return false;
    taskENTER_CRITICAL(&s_lock);
    bool b = s_pending[id] > 0 || s_active[id];
    taskEXIT_CRITICAL(&s_lock);
    return b;
}

bool motor_ctrl_any_busy(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        if (motor_ctrl_is_busy((motor_id_t)i)) return true;
    return false;
}

int32_t motor_ctrl_get_position(motor_id_t id)
{
    return valid(id) ? encoder_get(s_enc[id]) : 0;
}

esp_err_t motor_ctrl_set_position(motor_id_t id, int32_t pos)
{
    if (!valid(id)) return ESP_ERR_INVALID_ARG;
    if (motor_ctrl_is_busy(id)) return ESP_ERR_INVALID_STATE;
    encoder_set(s_enc[id], pos);
    return ESP_OK;
}

esp_err_t motor_ctrl_set_endpoints(motor_id_t id, int32_t end_top, int32_t end_bottom)
{
    if (!valid(id) || end_top == end_bottom) return ESP_ERR_INVALID_ARG;
    if (motor_ctrl_is_busy(id)) return ESP_ERR_INVALID_STATE;
    taskENTER_CRITICAL(&s_lock);
    axis_set_endpoints(&s_axis[id], end_top, end_bottom, s_margin_top[id], s_margin_bottom[id]);
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "Motor %d: Endpunkte oben %ld / unten %ld", id + 1, (long)end_top, (long)end_bottom);
    return ESP_OK;
}

esp_err_t motor_ctrl_clear_calibration(motor_id_t id)
{
    if (!valid(id)) return ESP_ERR_INVALID_ARG;
    if (motor_ctrl_is_busy(id)) return ESP_ERR_INVALID_STATE;
    taskENTER_CRITICAL(&s_lock);
    axis_clear(&s_axis[id]);
    taskEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void motor_ctrl_get_axis(motor_id_t id, axis_t *out)
{
    if (!valid(id) || !out) return;
    taskENTER_CRITICAL(&s_lock);
    *out = s_axis[id];
    taskEXIT_CRITICAL(&s_lock);
    out->pos = encoder_get(s_enc[id]);
}

const char *motor_event_str(motor_event_type_t t)
{
    switch (t) {
    case MOTOR_EVT_DONE:      return "Ziel erreicht";
    case MOTOR_EVT_LIMIT:     return "Endpunkt erreicht";
    case MOTOR_EVT_STOPPED:   return "gestoppt";
    case MOTOR_EVT_BLOCKED:   return "blockiert";
    case MOTOR_EVT_TIMEOUT:   return "Timeout";
    case MOTOR_EVT_WRONG_DIR: return "Richtungsfehler";
    case MOTOR_EVT_FAULT:     return "nFAULT";
    case MOTOR_EVT_REJECTED:  return "abgelehnt";
    case MOTOR_EVT_PROGRESS:  return "Fortschritt";
    default:                  return "?";
    }
}
