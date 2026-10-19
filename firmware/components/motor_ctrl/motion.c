/* Fahrprofil-Zustandsautomat – reine Logik, auf dem Host getestet (test_host/test_motion.c) */
#include "motion.h"
#include <stddef.h>

#define DUTY_MAX 1000

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
static uint16_t clamp_duty(int32_t d) { return (uint16_t)(d < 0 ? 0 : d > DUTY_MAX ? DUTY_MAX : d); }
static bool elapsed(uint32_t now, uint32_t t0, uint32_t ms) { return (uint32_t)(now - t0) >= ms; }

static bool is_driving(const motion_t *m)
{
    return m->phase == MOTION_KICK || m->phase == MOTION_RAMP || m->phase == MOTION_CRUISE;
}

void motion_init(motion_t *m, const motion_params_t *p)
{
    *m = (motion_t){ 0 };
    m->p = *p;
    m->p.min_duty = clamp_duty(m->p.min_duty);
    m->p.run_duty = clamp_duty(m->p.run_duty);
    m->p.kick_duty = clamp_duty(m->p.kick_duty);
    if (m->p.run_duty < m->p.min_duty) m->p.run_duty = m->p.min_duty;
    if (m->p.kick_duty < m->p.min_duty) m->p.kick_duty = m->p.min_duty;
    if (m->p.kick_steps < 1) m->p.kick_steps = 1;
    if (m->p.stop_tol_steps < 0) m->p.stop_tol_steps = 0;
    if (m->p.timeout_max_ms == 0) m->p.timeout_max_ms = 1;
    block_det_init(&m->blk, m->p.block_window_ms, m->p.block_min_steps);
    m->phase = MOTION_IDLE;
}

void motion_set_limits(motion_t *m, bool enabled, int32_t lo, int32_t hi)
{
    m->limits_on = enabled;
    m->lo = lo < hi ? lo : hi;
    m->hi = lo < hi ? hi : lo;
}

/* Richtung eines Auftrags von pos aus; 0 = schon am Ziel (innerhalb Toleranz) */
static int8_t job_dir(const motion_t *m, const motion_job_t *j, int32_t pos)
{
    if (!j->has_goal) return j->dir;
    int32_t d = j->goal - pos;
    if (abs32(d) <= m->p.stop_tol_steps) return 0;
    return d > 0 ? 1 : -1;
}

static void enter_brake(motion_t *m, uint32_t now, motion_result_t r)
{
    m->phase = MOTION_BRAKE;
    m->phase_t0 = now;
    m->result_pending = r;
}

static void begin_drive(motion_t *m, uint32_t now, int32_t pos)
{
    int8_t d = job_dir(m, &m->job, pos);
    if (d == 0) {
        m->phase = MOTION_FINISH;
        m->result_pending = m->job.reach_result;
        return;
    }
    m->dir = d;
    m->start_pos = pos;
    m->job_t0 = now;
    m->phase = MOTION_KICK;
    m->phase_t0 = now;
    m->duty = m->p.kick_duty;
    block_det_reset(&m->blk);
}

/* aus IDLE: gleich anfahren oder erst die Richtungswechsel-Pause abwarten */
static void begin_or_pause(motion_t *m, uint32_t now, int32_t pos)
{
    m->job_t0 = now;          /* auch für FINISH/PAUSE eine gültige Startzeit */
    int8_t d = job_dir(m, &m->job, pos);
    if (d == 0) {
        m->phase = MOTION_FINISH;
        m->result_pending = m->job.reach_result;
        return;
    }
    if (m->ever_moved && d != m->last_dir && !elapsed(now, m->idle_t0, m->p.dir_pause_ms)) {
        m->phase = MOTION_PAUSE;
        m->phase_t0 = now;
        m->pause_end = m->idle_t0 + m->p.dir_pause_ms;
        return;
    }
    begin_drive(m, now, pos);
}

static void submit(motion_t *m, uint32_t now, int32_t pos, const motion_job_t *j)
{
    switch (m->phase) {
    case MOTION_KICK:
    case MOTION_RAMP:
    case MOTION_CRUISE: {
        int8_t d = job_dir(m, j, pos);
        if (d == m->dir || d == 0) {
            /* gleiche Richtung (oder schon da): Ziel aktualisieren, Timeout neu */
            m->job = *j;
            m->job_t0 = now;
        } else {
            m->next = *j;
            m->next_valid = true;
            m->phase = MOTION_STOPPING;
            m->phase_t0 = now;
            m->stop_from = m->duty;
        }
        break;
    }
    case MOTION_PAUSE:
        m->job = *j;          /* wartet ohnehin, neuer Auftrag ersetzt den alten */
        break;
    case MOTION_STOPPING:
    case MOTION_BRAKE:
    case MOTION_FINISH:
        m->next = *j;
        m->next_valid = true;
        break;
    case MOTION_IDLE:
    default:
        m->job = *j;
        begin_or_pause(m, now, pos);
        break;
    }
}

static uint32_t eff_timeout(const motion_t *m, uint32_t t)
{
    return (t == 0 || t > m->p.timeout_max_ms) ? m->p.timeout_max_ms : t;
}

void motion_goto(motion_t *m, uint32_t now_ms, int32_t pos, int32_t goal, uint32_t timeout_ms)
{
    if (m->limits_on) {
        if (goal < m->lo) goal = m->lo;
        if (goal > m->hi) goal = m->hi;
    }
    motion_job_t j = {
        .has_goal = true, .goal = goal, .dir = 0,
        .timeout_ms = eff_timeout(m, timeout_ms), .reach_result = MOTION_RES_DONE,
    };
    submit(m, now_ms, pos, &j);
}

void motion_run(motion_t *m, uint32_t now_ms, int32_t pos, int8_t dir, uint32_t timeout_ms)
{
    dir = dir >= 0 ? 1 : -1;
    motion_job_t j = { .has_goal = false, .dir = dir,
                       .timeout_ms = eff_timeout(m, timeout_ms), .reach_result = MOTION_RES_LIMIT };
    if (m->limits_on) {
        j.has_goal = true;
        j.goal = dir > 0 ? m->hi : m->lo;
    }
    submit(m, now_ms, pos, &j);
}

bool motion_stop(motion_t *m, uint32_t now_ms)
{
    m->next_valid = false;
    switch (m->phase) {
    case MOTION_KICK:
    case MOTION_RAMP:
    case MOTION_CRUISE:
    case MOTION_STOPPING:
        enter_brake(m, now_ms, MOTION_RES_STOPPED);
        return true;
    case MOTION_BRAKE:
        if (m->result_pending == MOTION_RES_NONE) m->result_pending = MOTION_RES_STOPPED;
        return true;
    case MOTION_PAUSE:
        m->phase = MOTION_FINISH;
        m->result_pending = MOTION_RES_STOPPED;
        return true;
    case MOTION_FINISH:
        return true;
    case MOTION_IDLE:
    default:
        return false;
    }
}

void motion_abort(motion_t *m, uint32_t now_ms)
{
    if (m->phase != MOTION_IDLE && m->phase != MOTION_FINISH && m->phase != MOTION_PAUSE) {
        m->last_dir = m->dir;
        m->ever_moved = true;
    }
    m->phase = MOTION_IDLE;
    m->idle_t0 = now_ms;
    m->next_valid = false;
    m->result_pending = MOTION_RES_NONE;
    m->duty = 0;
}

/* nach Ende eines Auftrags: wartenden Auftrag starten */
static void go_idle_and_continue(motion_t *m, uint32_t now, int32_t pos)
{
    m->phase = MOTION_IDLE;
    m->duty = 0;
    if (m->next_valid) {
        m->next_valid = false;
        m->job = m->next;
        begin_or_pause(m, now, pos);
    }
}

/* Ziel/Fahrzeit des gerade beendeten Auftrags ins Ergebnis, BEVOR ein
 * wartender Folgeauftrag m->job überschreibt */
static void set_result_info(const motion_t *m, uint32_t now, motion_out_t *out)
{
    if (out->result == MOTION_RES_NONE) return;
    out->goal = m->job.has_goal ? m->job.goal : 0;
    out->elapsed_ms = now - m->job_t0;
}

motion_out_t motion_tick(motion_t *m, uint32_t now, int32_t pos)
{
    motion_out_t out = { .drive = MOTION_OUT_OFF, .dir = m->dir, .duty = 0, .result = MOTION_RES_NONE };

    switch (m->phase) {
    case MOTION_IDLE:
        return out;

    case MOTION_FINISH:
        out.result = m->result_pending;
        m->result_pending = MOTION_RES_NONE;
        set_result_info(m, now, &out);
        go_idle_and_continue(m, now, pos);
        return out;

    case MOTION_PAUSE:
        if ((int32_t)(now - m->pause_end) >= 0) begin_drive(m, now, pos);
        if (m->phase == MOTION_FINISH) return motion_tick(m, now, pos);
        if (!is_driving(m)) return out;
        break;   /* gleich im selben Takt anfahren */

    case MOTION_BRAKE:
        if (!elapsed(now, m->phase_t0, m->p.brake_ms)) {
            out.drive = MOTION_OUT_BRAKE;
            return out;
        }
        out.result = m->result_pending;
        m->result_pending = MOTION_RES_NONE;
        set_result_info(m, now, &out);
        m->last_dir = m->dir;
        m->idle_t0 = now;
        m->ever_moved = true;
        go_idle_and_continue(m, now, pos);
        return out;      /* dieser Takt: aus; ein Folgeauftrag fährt im nächsten Takt */

    case MOTION_STOPPING: {
        /* Auch beim Abbremsen vor einem Richtungswechsel gelten Timeout und harte
         * Endpunkte: sofort bremsen statt noch ramp_down_ms weiterzufahren. */
        if (elapsed(now, m->job_t0, m->job.timeout_ms) ||
            (m->limits_on && ((m->dir > 0 && pos >= m->hi) || (m->dir < 0 && pos <= m->lo)))) {
            enter_brake(m, now, MOTION_RES_NONE);
            out.drive = MOTION_OUT_BRAKE;
            return out;
        }
        uint32_t t = now - m->phase_t0;
        int32_t span = (int32_t)m->stop_from - m->p.min_duty;
        int32_t d = m->p.ramp_down_ms ? (int32_t)m->stop_from - (int32_t)((int64_t)span * t / m->p.ramp_down_ms)
                                      : m->p.min_duty;
        if (span <= 0 || t >= m->p.ramp_down_ms || d <= m->p.min_duty) {
            enter_brake(m, now, MOTION_RES_NONE);
            out.drive = MOTION_OUT_BRAKE;
            return out;
        }
        m->duty = clamp_duty(d);
        out.drive = MOTION_OUT_DRIVE;
        out.duty = m->duty;
        return out;
    }

    default:
        break;
    }

    /* ---- KICK / RAMP / CRUISE: gemeinsame Abbruchbedingungen ---- */
    if (elapsed(now, m->job_t0, m->job.timeout_ms)) {
        enter_brake(m, now, MOTION_RES_TIMEOUT);
        out.drive = MOTION_OUT_BRAKE;
        return out;
    }
    if ((pos - m->start_pos) * m->dir <= -m->p.wrong_dir_steps && m->p.wrong_dir_steps > 0) {
        enter_brake(m, now, MOTION_RES_WRONG_DIR);
        out.drive = MOTION_OUT_BRAKE;
        return out;
    }
    int32_t rest = 0;
    if (m->job.has_goal) {
        rest = (m->job.goal - pos) * m->dir;
        if (rest <= m->p.stop_tol_steps) {                       /* erreicht oder überfahren */
            enter_brake(m, now, m->job.reach_result);
            out.drive = MOTION_OUT_BRAKE;
            return out;
        }
    }
    if (m->limits_on && ((m->dir > 0 && pos >= m->hi) || (m->dir < 0 && pos <= m->lo))) {
        enter_brake(m, now, m->job.has_goal ? m->job.reach_result : MOTION_RES_LIMIT);
        out.drive = MOTION_OUT_BRAKE;
        return out;
    }

    /* ---- Phasen ---- */
    if (m->phase == MOTION_KICK) {
        if (abs32(pos - m->start_pos) >= m->p.kick_steps) {
            m->phase = MOTION_RAMP;          /* dreht: auf min_duty zurück, dann Rampe */
            m->phase_t0 = now;
        } else if (elapsed(now, m->phase_t0, m->p.kick_ms)) {
            enter_brake(m, now, MOTION_RES_BLOCKED);
            out.drive = MOTION_OUT_BRAKE;
            return out;
        } else {
            m->duty = m->p.kick_duty;
        }
    }
    if (m->phase == MOTION_RAMP) {
        uint32_t t = now - m->phase_t0;
        if (m->p.ramp_up_ms == 0 || t >= m->p.ramp_up_ms || m->p.run_duty <= m->p.min_duty) {
            m->phase = MOTION_CRUISE;
            m->phase_t0 = now;
            block_det_reset(&m->blk);        /* Blockiererkennung erst jetzt scharf */
        } else {
            int32_t span = (int32_t)m->p.run_duty - m->p.min_duty;
            m->duty = clamp_duty(m->p.min_duty + (int32_t)((int64_t)span * t / m->p.ramp_up_ms));
        }
    }
    if (m->phase == MOTION_CRUISE) {
        m->duty = m->p.run_duty;
        if (m->p.block_min_steps > 0 && block_det_update(&m->blk, now, pos)) {
            enter_brake(m, now, MOTION_RES_BLOCKED);
            out.drive = MOTION_OUT_BRAKE;
            return out;
        }
    }

    /* Bremszone vor dem Ziel (nicht im Kick, der muss den Motor erst losbrechen) */
    if (m->job.has_goal && m->phase != MOTION_KICK && m->p.brake_zone_steps > 0 &&
        rest < m->p.brake_zone_steps) {
        int32_t cap = (int32_t)((int64_t)m->p.run_duty * rest / m->p.brake_zone_steps);
        if (cap < m->p.min_duty) cap = m->p.min_duty;
        if (m->duty > cap) m->duty = (uint16_t)cap;
    }

    out.drive = MOTION_OUT_DRIVE;
    out.dir = m->dir;
    out.duty = m->duty;
    return out;
}

bool motion_is_active(const motion_t *m)
{
    return m->phase != MOTION_IDLE;
}

uint32_t motion_job_elapsed(const motion_t *m, uint32_t now_ms)
{
    return now_ms - m->job_t0;
}

const char *motion_result_str(motion_result_t r)
{
    switch (r) {
    case MOTION_RES_NONE:      return "-";
    case MOTION_RES_DONE:      return "Ziel erreicht";
    case MOTION_RES_LIMIT:     return "Endpunkt erreicht";
    case MOTION_RES_STOPPED:   return "gestoppt";
    case MOTION_RES_BLOCKED:   return "blockiert";
    case MOTION_RES_TIMEOUT:   return "Timeout";
    case MOTION_RES_WRONG_DIR: return "Richtungsfehler";
    default:                   return "?";
    }
}
