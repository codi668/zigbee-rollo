/*
 * app_state: app_queue + app_task. Führt die Aktionen des Zustandsautomaten
 * (app_logic.c, reine Logik) auf motor_ctrl, storage, ui, sensor_task und
 * zb_device aus.
 */
#include "app_state.h"
#include <inttypes.h>
#include <string.h>
#include "axis.h"
#include "config.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sensor_task.h"
#include "zb_device.h"
#include "zb_logic.h"

static const char *TAG = "app";

/* ---- Nachrichten der app_queue ---- */
typedef enum { MSG_IN = 0, MSG_BATTERY, MSG_NET, MSG_IDENTIFY } msg_kind_t;

typedef struct {
    uint8_t kind;   /* msg_kind_t */
    uint8_t src;    /* app_src_t (nur Befehle) */
    union {
        app_in_t         in;
        battery_status_t batt;
        app_net_t        net;
        bool             identify;
    };
} app_msg_t;

static QueueHandle_t s_queue;          /* = app_queue: Befehle, Tasten, Akku, Netz */
/* Motorereignisse in einer eigenen Queue, damit ein volles app_queue (Befehlsflut,
 * blockierte app_task) Abschluss-Ereignisse (DONE/BLOCKED/FAULT ...) nie verdrängt.
 * Darin nur Abschluss-Ereignisse (je Auftrag genau eins, Aufträge kommen nur von der
 * app_task) und höchstens eine PROGRESS-Marke je Achse (zusammengefasst, Position im
 * Latch) -> APP_MOTOR_QUEUE_LEN reicht immer. FIFO hält PROGRESS vor dem Abschluss. */
static QueueHandle_t s_motor_queue;
static QueueSetHandle_t s_queue_set;
static portMUX_TYPE s_prog_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_prog_pos[APP_AXIS_COUNT];       /* unter s_prog_lock */
static bool s_prog_pending[APP_AXIS_COUNT];      /* Marke liegt in s_motor_queue */

/* ---- nur von der app_task benutzt ---- */
static app_ctx_t s_ctx;
static app_net_t s_net = APP_NET_OFFLINE;
static bool s_identify;
static bool s_zb_operational, s_zb_calibrating, s_zb_status_sent;
static int64_t s_idle_since_us;        /* Erkennung verlorener Motorereignisse */

/* ---- Momentaufnahme (app_task schreibt, andere lesen) ---- */
static portMUX_TYPE s_snap_lock = portMUX_INITIALIZER_UNLOCKED;
static app_snapshot_t s_snap;

static const char *const s_axis_name[APP_AXIS_COUNT] = { "Heben", "Wenden" };

static const char *axis_str(uint8_t a) { return a < APP_AXIS_COUNT ? s_axis_name[a] : "?"; }

static const char *cmd_str(app_cmd_t c)
{
    switch (c) {
    case APP_CMD_UP:            return "auf";
    case APP_CMD_DOWN:          return "ab";
    case APP_CMD_STOP:          return "stopp";
    case APP_CMD_GOTO:          return "goto";
    case APP_CMD_CAL_START:     return "kalibrieren";
    case APP_CMD_CAL_SAVE:      return "unten speichern";
    case APP_CMD_CAL_ABORT:     return "Kalibrierung abbrechen";
    case APP_CMD_JOG:           return "tippen";
    case APP_CMD_ACK:           return "quittieren";
    case APP_CMD_MODE_WRITE:    return "Mode schreiben";
    case APP_CMD_FACTORY_RESET: return "Werkseinstellung";
    case APP_CMD_LIMIT_TOP:     return "Limit oben";
    case APP_CMD_LIMIT_BOTTOM:  return "Limit unten";
    case APP_CMD_LIMIT_SET:     return "Limit setzen (Weg)";
    case APP_CMD_LIMIT_CLEAR:   return "Limit loeschen";
    case APP_CMD_MARGIN:        return "Randabstand";
    default:                    return "?";
    }
}

static const char *src_str(uint8_t s)
{
    return s == APP_SRC_ZIGBEE ? "Zigbee" : s == APP_SRC_CONSOLE ? "Konsole" : "intern";
}

/* ------------------------------------------------------------------------- */
/* Ausgaben                                                                   */

static void report_axis(uint8_t axis, bool final)
{
    axis_t ax;
    motor_ctrl_get_axis((motor_id_t)axis, &ax);
    if (!ax.calibrated && s_ctx.calibrated[axis]) {
        /* Referenzfahrt: motor_ctrl kennt gerade keine Endpunkte, letzten Wert stehen lassen */
        return;
    }
    uint8_t pct = ax.calibrated ? axis_steps_to_percent(&ax, ax.pos) : 0;
    uint8_t attr = zbl_position_attr(ax.calibrated, pct);
    zb_device_set_position(axis, attr, final);
    if (final) {
        if (ax.calibrated)
            ESP_LOGI(TAG, "%s: %u %% (Pos %" PRId32 " von %" PRId32 ")", axis_str(axis), attr, ax.pos, ax.end_bottom);
        else
            ESP_LOGI(TAG, "%s: unkalibriert (Pos %" PRId32 ")", axis_str(axis), ax.pos);
    }
}

static void update_outputs(void)
{
    ui_set_led(app_logic_led(&s_ctx, s_net == APP_NET_JOINING, s_identify));
    /* ZB_CFG_OPERATIONAL_ALWAYS: siehe config.h (ungeklärt, ob der Stack Befehle bei 0 abweist) */
    bool op = ZB_CFG_OPERATIONAL_ALWAYS ? true : app_logic_operational(&s_ctx);
    bool cal = app_logic_calibrating(&s_ctx);
    if (!s_zb_status_sent || op != s_zb_operational || cal != s_zb_calibrating) {
        s_zb_operational = op;
        s_zb_calibrating = cal;
        s_zb_status_sent = true;
        zb_device_set_status(op, cal);
    }
    taskENTER_CRITICAL(&s_snap_lock);
    s_snap.st = s_ctx.st;
    s_snap.err = s_ctx.err;
    s_snap.err_axis = s_ctx.err_axis;
    s_snap.cal_step = s_ctx.cal_step;
    s_snap.cal_axis = s_ctx.cal_axis;
    for (int i = 0; i < APP_AXIS_COUNT; i++) {
        s_snap.calibrated[i] = s_ctx.calibrated[i];
        s_snap.needs_homing[i] = s_ctx.needs_homing[i];
        s_snap.bottom[i] = s_ctx.bottom[i];
    }
    s_snap.net = s_net;
    taskEXIT_CRITICAL(&s_snap_lock);
}

/* ------------------------------------------------------------------------- */
/* Aktionen ausführen                                                         */

static void storage_check(const char *what, esp_err_t e)
{
    if (e != ESP_OK) ESP_LOGE(TAG, "NVS %s: %s", what, esp_err_to_name(e));
}

/** Fahrbefehl ausführen; bei Fehler Ersatz-Ereignis für die FSM liefern */
static bool exec_motion(const app_act_t *a, app_in_t *fb)
{
    motor_id_t id = (motor_id_t)a->axis;
    esp_err_t e;
    sensor_lock();   /* keine Akku-Messung zwischen Prüfung und Fahrtbeginn */
    switch (a->type) {
    case APP_ACT_RUN:
        e = motor_ctrl_run(id, a->a < 0 ? MOTOR_DIR_UP : MOTOR_DIR_DOWN, (uint32_t)a->b);
        break;
    case APP_ACT_GOTO_PCT:
        e = motor_ctrl_goto_percent(id, (uint8_t)a->a, 0);
        break;
    default:   /* APP_ACT_MOVE_BY */
        e = motor_ctrl_move_by(id, a->a, (uint32_t)a->b);
        break;
    }
    sensor_unlock();
    if (e == ESP_OK) return false;
    bool fault = motor_ctrl_fault_latched();
    ESP_LOGW(TAG, "Fahrbefehl %s abgelehnt: %s%s", axis_str(a->axis), esp_err_to_name(e),
             fault ? " (nFAULT-Sperre)" : "");
    /* Die FSM beendet die Fahrt mit dem Ersatz-Ereignis. Lief noch ein alter Auftrag
     * (Ziel sollte ersetzt werden), darf er nicht unbeobachtet weiterfahren. */
    if (motor_ctrl_any_busy()) motor_ctrl_stop_all();
    memset(fb, 0, sizeof(*fb));
    fb->type = APP_IN_MOTOR;
    fb->motor.ev = fault ? APP_MEV_FAULT : APP_MEV_REJECTED;
    fb->motor.axis = a->axis;
    fb->motor.pos = motor_ctrl_get_position(id);
    return true;
}

/** @return true, wenn ein Ersatz-Ereignis in fb steht */
static bool execute(const app_acts_t *acts, app_in_t *fb)
{
    for (int i = 0; i < acts->n; i++) {
        const app_act_t *a = &acts->a[i];
        motor_id_t id = (motor_id_t)a->axis;
        esp_err_t e = ESP_OK;
        switch (a->type) {
        case APP_ACT_RUN:
        case APP_ACT_GOTO_PCT:
        case APP_ACT_MOVE_BY:
            if (exec_motion(a, fb)) return true;
            break;
        case APP_ACT_STOP:
            motor_ctrl_stop_all();
            break;
        case APP_ACT_CLEAR_CAL:
            e = motor_ctrl_clear_calibration(id);
            break;
        case APP_ACT_SET_POS:
            e = motor_ctrl_set_position(id, a->a);
            break;
        case APP_ACT_SET_ENDPOINTS:
            e = motor_ctrl_set_endpoints(id, 0, a->a);
            break;
        case APP_ACT_CLEAR_FAULT:
            if (motor_ctrl_fault_latched() && motor_ctrl_clear_fault() != ESP_OK) {
                ESP_LOGW(TAG, "nFAULT liegt noch an, Quittieren nicht moeglich");
                memset(fb, 0, sizeof(*fb));
                fb->type = APP_IN_MOTOR;
                fb->motor.ev = APP_MEV_FAULT;
                fb->motor.axis = 0;
                fb->motor.pos = motor_ctrl_get_position(MOTOR_LIFT);
                return true;
            }
            break;
        case APP_ACT_NV_MOTION_BEGIN:
            storage_check("motion_begin", storage_motion_begin());
            break;
        case APP_ACT_NV_MOTION_END:
            storage_check("motion_end", storage_motion_end((storage_axis_t)a->axis, a->a));
            break;
        case APP_ACT_NV_SAVE_POS:
            storage_check("save_position", storage_save_position((storage_axis_t)a->axis, a->a));
            break;
        case APP_ACT_NV_SAVE_CAL:
            storage_check("save_calibration", storage_save_calibration((storage_axis_t)a->axis, a->a));
            ESP_LOGI(TAG, "%s: Endpunkt unten %" PRId32 " gespeichert", axis_str(a->axis), a->a);
            break;
        case APP_ACT_SET_MARGIN:
            e = motor_ctrl_set_margins(a->a != 0);
            if (e == ESP_OK) storage_check("save_margin", storage_save_margin(a->a != 0));
            break;
        case APP_ACT_NV_CLEAR_CAL:
            storage_check("clear_calibration", storage_clear_calibration((storage_axis_t)a->axis));
            break;
        case APP_ACT_REPORT:
            report_axis(a->axis, a->a != 0);
            break;
        case APP_ACT_MEASURE:
            sensor_request_measure(BATT_SETTLE_MS);
            break;
        case APP_ACT_FACTORY_RESET:
            ESP_LOGW(TAG, "Werkseinstellung: NVS loeschen, Zigbee-Netz verlassen, Neustart");
            ui_set_led(LED_PAT_ON);
            storage_check("erase_all", storage_erase_all());
            zb_device_factory_reset();   /* kehrt nicht zurück */
            break;
        }
        if (e != ESP_OK)
            ESP_LOGE(TAG, "Aktion %d (%s) fehlgeschlagen: %s", a->type, axis_str(a->axis), esp_err_to_name(e));
    }
    return false;
}

static void log_note(app_note_t n)
{
    if (n == APP_NOTE_NONE || n == APP_NOTE_IGNORED) return;
    if (n == APP_NOTE_REJECTED_STATE || n == APP_NOTE_NOT_CALIBRATED || n == APP_NOTE_BATTERY_EMPTY ||
        n == APP_NOTE_CAL_TOO_SHORT || n == APP_NOTE_MODE_REVERSE)
        ESP_LOGW(TAG, "%s (Zustand %s)", app_logic_note_str(n), app_logic_state_name(s_ctx.st));
    else
        ESP_LOGI(TAG, "%s", app_logic_note_str(n));
}

static void run_step(const app_in_t *in)
{
    app_in_t cur = *in;
    for (int depth = 0; depth < 4; depth++) {
        app_st_t before = s_ctx.st;
        app_cal_t cal_before = s_ctx.cal_step;
        uint8_t cal_axis_before = s_ctx.cal_axis;
        app_acts_t acts;
        app_logic_step(&s_ctx, &cur, &acts);
        log_note(acts.note);
        app_in_t fb;
        bool again = execute(&acts, &fb);

        if (s_ctx.st != before) {
            if (s_ctx.st == APP_ST_ERROR)
                ESP_LOGE(TAG, "Zustand %s -> FEHLER: %s, %s", app_logic_state_name(before),
                         app_logic_err_name(s_ctx.err), axis_str(s_ctx.err_axis));
            else if (s_ctx.st == APP_ST_MOVING)
                ESP_LOGI(TAG, "Zustand %s -> FAHRT (%s)", app_logic_state_name(before), axis_str(s_ctx.mv_axis));
            else
                ESP_LOGI(TAG, "Zustand %s -> %s", app_logic_state_name(before), app_logic_state_name(s_ctx.st));
        }
        if (s_ctx.st == APP_ST_CALIBRATING &&
            (s_ctx.cal_step != cal_before || s_ctx.cal_axis != cal_axis_before || before != APP_ST_CALIBRATING))
            ESP_LOGI(TAG, "Kalibrierung %s: %s", axis_str(s_ctx.cal_axis), app_logic_cal_name(s_ctx.cal_step));
        if (!again) break;
        cur = fb;
    }
    update_outputs();
}

/* ------------------------------------------------------------------------- */
/* app_task                                                                   */

static void handle(const app_msg_t *m)
{
    switch (m->kind) {
    case MSG_IN:
        if (m->in.type == APP_IN_CMD)
            ESP_LOGI(TAG, "Befehl %s: %s %s %" PRId32, src_str(m->src), cmd_str(m->in.cmd.cmd),
                     m->in.cmd.cmd == APP_CMD_CAL_START ? "" : axis_str(m->in.cmd.axis), m->in.cmd.value);
        run_step(&m->in);
        break;
    case MSG_BATTERY: {
        const battery_status_t *b = &m->batt;
        taskENTER_CRITICAL(&s_snap_lock);
        s_snap.battery = *b;
        s_snap.battery_valid = true;
        taskEXIT_CRITICAL(&s_snap_lock);
        zb_device_set_battery(b);
        ESP_LOGI(TAG, "Akku %u mV, %u %%%s%s", b->vbat_mv, b->percent, b->charging ? ", laedt" : "",
                 b->lockout ? ", SPERRE" : "");
        app_in_t in = { .type = APP_IN_BATTERY };
        in.battery.lockout = b->lockout;
        run_step(&in);
        break;
    }
    case MSG_NET:
        if (m->net != s_net)
            ESP_LOGI(TAG, "Netz: %s", m->net == APP_NET_JOINED ? "verbunden" :
                                    m->net == APP_NET_JOINING ? "Pairing (Netzsuche)" : "offline");
        s_net = m->net;
        update_outputs();
        break;
    case MSG_IDENTIFY:
        s_identify = m->identify;
        update_outputs();
        break;
    }
}

/** Fährt die FSM, obwohl motor_ctrl schon lange steht, ging ein Abschluss-Ereignis verloren. */
static void check_lost_event(void)
{
    bool fsm_moving = s_ctx.st == APP_ST_MOVING || (s_ctx.st == APP_ST_CALIBRATING && s_ctx.cal_moving);
    if (!fsm_moving || motor_ctrl_any_busy()) {
        s_idle_since_us = 0;
        return;
    }
    int64_t now = esp_timer_get_time();
    if (!s_idle_since_us) {
        s_idle_since_us = now;
        return;
    }
    if (now - s_idle_since_us < (int64_t)APP_LOST_EVENT_MS * 1000) return;
    s_idle_since_us = 0;
    uint8_t axis = s_ctx.st == APP_ST_MOVING ? s_ctx.mv_axis : s_ctx.cal_axis;
    ESP_LOGW(TAG, "Motorereignis verloren (%s steht) -> als gestoppt behandeln", axis_str(axis));
    app_in_t in = { .type = APP_IN_MOTOR };
    in.motor.ev = APP_MEV_STOPPED;
    in.motor.axis = axis;
    in.motor.pos = motor_ctrl_get_position((motor_id_t)axis);
    run_step(&in);
}

/** Motorereignis aus s_motor_queue; PROGRESS holt die aktuelle Position aus dem Latch */
static void handle_motor(app_in_t *in)
{
    if (in->motor.ev == APP_MEV_PROGRESS && in->motor.axis < APP_AXIS_COUNT) {
        taskENTER_CRITICAL(&s_prog_lock);
        in->motor.pos = s_prog_pos[in->motor.axis];
        s_prog_pending[in->motor.axis] = false;
        taskEXIT_CRITICAL(&s_prog_lock);
    }
    run_step(in);
}

static void app_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    for (;;) {
        QueueSetMemberHandle_t q = xQueueSelectFromSet(s_queue_set, pdMS_TO_TICKS(APP_WDT_FEED_MS));
        if (q == s_motor_queue) {
            app_in_t in;
            if (xQueueReceive(s_motor_queue, &in, 0) == pdTRUE) handle_motor(&in);
        } else if (q == s_queue) {
            app_msg_t m;
            if (xQueueReceive(s_queue, &m, 0) == pdTRUE) handle(&m);
        }
        check_lost_event();
        esp_task_wdt_reset();
    }
}

/* ------------------------------------------------------------------------- */
/* Öffentliche Funktionen                                                     */

esp_err_t app_state_init(void)
{
    if (s_queue) return ESP_ERR_INVALID_STATE;
    s_queue = xQueueCreate(APP_QUEUE_LEN, sizeof(app_msg_t));
    s_motor_queue = xQueueCreate(APP_MOTOR_QUEUE_LEN, sizeof(app_in_t));
    s_queue_set = xQueueCreateSet(APP_QUEUE_LEN + APP_MOTOR_QUEUE_LEN);
    if (!s_queue || !s_motor_queue || !s_queue_set) return ESP_ERR_NO_MEM;
    /* Mitglieder müssen beim Hinzufügen leer sein */
    if (xQueueAddToSet(s_queue, s_queue_set) != pdPASS || xQueueAddToSet(s_motor_queue, s_queue_set) != pdPASS)
        return ESP_ERR_INVALID_STATE;
    app_logic_init(&s_ctx);
    return ESP_OK;
}

esp_err_t app_state_start(const storage_data_t *nv, const battery_status_t *batt)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    app_msg_t m = { .kind = MSG_IN, .src = APP_SRC_INTERNAL };
    m.in.type = APP_IN_BOOT;
    for (int i = 0; i < APP_AXIS_COUNT; i++) {
        m.in.boot.bottom[i] = nv->max[i];
        m.in.boot.pos[i] = nv->pos[i];
    }
    m.in.boot.moving = nv->moving;
    m.in.boot.lockout = batt ? batt->lockout : false;
    if (batt) {
        taskENTER_CRITICAL(&s_snap_lock);
        s_snap.battery = *batt;
        s_snap.battery_valid = true;
        taskEXIT_CRITICAL(&s_snap_lock);
        zb_device_set_battery(batt);   /* vor dem Stack-Start: nur Schattenkopie */
    }
    /* Start-Ereignis zuerst, dann erst Tasten/Befehle */
    if (xQueueSendToFront(s_queue, &m, 0) != pdTRUE) return ESP_ERR_NO_MEM;
    if (xTaskCreate(app_task, "app_task", APP_TASK_STACK, NULL, APP_TASK_PRIO, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static bool post(const app_msg_t *m)
{
    return s_queue && xQueueSend(s_queue, m, 0) == pdTRUE;
}

bool app_post_cmd(app_cmd_t cmd, uint8_t axis, int32_t value, app_src_t src)
{
    app_msg_t m = { .kind = MSG_IN, .src = (uint8_t)src };
    m.in.type = APP_IN_CMD;
    m.in.cmd.cmd = cmd;
    m.in.cmd.axis = axis;
    m.in.cmd.value = value;
    return post(&m);
}

void app_on_motor_event(const motor_event_t *e, void *ctx)
{
    (void)ctx;
    static const app_mev_t map[] = {
        [MOTOR_EVT_DONE] = APP_MEV_DONE,           [MOTOR_EVT_LIMIT] = APP_MEV_LIMIT,
        [MOTOR_EVT_STOPPED] = APP_MEV_STOPPED,     [MOTOR_EVT_BLOCKED] = APP_MEV_BLOCKED,
        [MOTOR_EVT_TIMEOUT] = APP_MEV_TIMEOUT,     [MOTOR_EVT_WRONG_DIR] = APP_MEV_WRONG_DIR,
        [MOTOR_EVT_FAULT] = APP_MEV_FAULT,         [MOTOR_EVT_REJECTED] = APP_MEV_REJECTED,
        [MOTOR_EVT_PROGRESS] = APP_MEV_PROGRESS,
    };
    if ((unsigned)e->type >= sizeof(map) / sizeof(map[0]) || (unsigned)e->id >= APP_AXIS_COUNT) return;
    if (!s_motor_queue) return;
    app_in_t in = { .type = APP_IN_MOTOR };
    in.motor.ev = map[e->type];
    in.motor.axis = (uint8_t)e->id;
    in.motor.pos = e->pos;
    if (e->type == MOTOR_EVT_PROGRESS) {
        /* zusammenfassen: Position merken, Marke nur, wenn keine mehr aussteht */
        taskENTER_CRITICAL(&s_prog_lock);
        s_prog_pos[e->id] = e->pos;
        bool need = !s_prog_pending[e->id];
        s_prog_pending[e->id] = true;
        taskEXIT_CRITICAL(&s_prog_lock);
        if (need && xQueueSend(s_motor_queue, &in, 0) != pdTRUE) {
            taskENTER_CRITICAL(&s_prog_lock);
            s_prog_pending[e->id] = false;
            taskEXIT_CRITICAL(&s_prog_lock);
        }
        return;
    }
    /* Abschluss-Ereignis: Akku erst nach der Erholzeit messen (auch nach FEHLER/Kalibrierung) */
    sensor_hold_off(BATT_SETTLE_MS);
    if (xQueueSend(s_motor_queue, &in, 0) != pdTRUE)
        ESP_LOGE(TAG, "Motor-Queue voll: Motorereignis %s verloren", motor_event_str(e->type));
}

void app_on_ui_event(ui_event_t e, void *ctx)
{
    (void)ctx;
    app_msg_t m = { .kind = MSG_IN, .src = APP_SRC_INTERNAL };
    m.in.type = APP_IN_BTN;
    switch (e) {
    case UI_EVT_BTN_SHORT:     m.in.btn = APP_BTN_SHORT; break;
    case UI_EVT_BTN_LONG_HINT: m.in.btn = APP_BTN_LONG_HINT; break;
    case UI_EVT_BTN_LONG:      m.in.btn = APP_BTN_LONG; break;
    case UI_EVT_BTN_VERY_LONG: m.in.btn = APP_BTN_VERY_LONG; break;
    default: return;
    }
    if (!post(&m)) ESP_LOGE(TAG, "app_queue voll: Tastenereignis verloren");
}

void app_post_battery(const battery_status_t *b)
{
    app_msg_t m = { .kind = MSG_BATTERY };
    m.batt = *b;
    if (!post(&m)) ESP_LOGW(TAG, "app_queue voll: Akku-Messung verworfen");
}

void app_post_net(app_net_t net)
{
    app_msg_t m = { .kind = MSG_NET };
    m.net = net;
    post(&m);
}

void app_post_identify(bool on)
{
    app_msg_t m = { .kind = MSG_IDENTIFY };
    m.identify = on;
    post(&m);
}

void app_state_get_snapshot(app_snapshot_t *out)
{
    taskENTER_CRITICAL(&s_snap_lock);
    *out = s_snap;
    taskEXIT_CRITICAL(&s_snap_lock);
}

void app_state_log_status(void)
{
    app_snapshot_t s;
    app_state_get_snapshot(&s);
    if (s.st == APP_ST_ERROR)
        ESP_LOGI(TAG, "Zustand FEHLER: %s (%s)", app_logic_err_name(s.err), axis_str(s.err_axis));
    else if (s.st == APP_ST_CALIBRATING)
        ESP_LOGI(TAG, "Zustand KALIBRIERUNG: %s, %s", axis_str(s.cal_axis), app_logic_cal_name(s.cal_step));
    else
        ESP_LOGI(TAG, "Zustand %s", app_logic_state_name(s.st));
    for (int i = 0; i < APP_AXIS_COUNT; i++) {
        axis_t ax;
        motor_ctrl_get_axis((motor_id_t)i, &ax);
        if (ax.calibrated)
            ESP_LOGI(TAG, "  %-6s %3u %%  Pos %" PRId32 "  unten %" PRId32 "%s", s_axis_name[i],
                     axis_steps_to_percent(&ax, ax.pos), ax.pos, ax.end_bottom,
                     s.needs_homing[i] ? "  (Referenzfahrt noetig)" : "");
        else
            ESP_LOGI(TAG, "  %-6s unkalibriert  Pos %" PRId32, s_axis_name[i], ax.pos);
    }
    ESP_LOGI(TAG, "  Rand   %s", motor_ctrl_margins_on() ? "an" : "aus");
    if (s.battery_valid) {
        const battery_status_t *b = &s.battery;
        int t10 = (int)(b->temp_c * 10.0f + (b->temp_c >= 0 ? 0.5f : -0.5f));
        int a10 = t10 < 0 ? -t10 : t10;
        if (b->ntc == BATT_NTC_OK)
            ESP_LOGI(TAG, "  Akku   %u mV, %u %%, %s, Sperre %s, NTC %s%d.%d C", b->vbat_mv, b->percent,
                     b->charging ? "laedt" : "laedt nicht", b->lockout ? "AKTIV" : "aus",
                     t10 < 0 ? "-" : "", a10 / 10, a10 % 10);
        else
            ESP_LOGI(TAG, "  Akku   %u mV, %u %%, %s, Sperre %s, NTC %s", b->vbat_mv, b->percent,
                     b->charging ? "laedt" : "laedt nicht", b->lockout ? "AKTIV" : "aus",
                     b->ntc == BATT_NTC_OPEN ? "offen" : "Kurzschluss");
    } else {
        ESP_LOGI(TAG, "  Akku   keine gueltige Messung");
    }
    ESP_LOGI(TAG, "  Netz   %s", s.net == APP_NET_JOINED ? "verbunden" :
                                 s.net == APP_NET_JOINING ? "Pairing" : "offline");
}
