/* Kipp-Erkennung – reine Logik, Host-getestet (test_host/test_tilt.c) */
#include "tilt_logic.h"
#include "config.h"
#include <math.h>

#define RAD2DEG 57.29577951308232f

float tilt_angle_deg(const lis2dh12_mg_t *a, const lis2dh12_mg_t *b)
{
    float ax = a->x, ay = a->y, az = a->z, bx = b->x, by = b->y, bz = b->z;
    float ma = sqrtf(ax * ax + ay * ay + az * az);
    float mb = sqrtf(bx * bx + by * by + bz * bz);
    if (ma < WIN_MIN_MAG_MG || mb < WIN_MIN_MAG_MG) return -1.0f;
    float c = (ax * bx + ay * by + az * bz) / (ma * mb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * RAD2DEG;
}

void tilt_det_init(tilt_det_t *t)
{
    *t = (tilt_det_t){ .state = WINDOW_UNKNOWN, .cand = WINDOW_UNKNOWN, .last_angle = -1.0f };
}

void tilt_det_set_ref(tilt_det_t *t, const lis2dh12_mg_t *ref)
{
    t->ref = *ref;
    t->has_ref = true;
    t->state = WINDOW_UNKNOWN;
    t->cand = WINDOW_UNKNOWN;
    t->cnt = 0;
}

void tilt_det_note_motion(tilt_det_t *t, uint32_t now_ms)
{
    t->motion_seen = true;
    t->last_motion_ms = now_ms;
    t->cnt = 0;      /* Entprellung neu beginnen */
}

bool tilt_det_holdoff(const tilt_det_t *t, uint32_t now_ms)
{
    return t->motion_seen && (uint32_t)(now_ms - t->last_motion_ms) < WIN_MOTION_HOLDOFF_MS;
}

bool tilt_det_update(tilt_det_t *t, const lis2dh12_mg_t *v)
{
    if (!t->has_ref) return false;
    float ang = tilt_angle_deg(&t->ref, v);
    if (ang < 0.0f) {          /* ungültige Messung: Entprellung neu */
        t->cnt = 0;
        return false;
    }
    t->last_angle = ang;

    window_state_t want;
    if (ang > WIN_TILT_ON_DEG) want = WINDOW_TILTED;
    else if (ang < WIN_TILT_OFF_DEG) want = WINDOW_CLOSED;
    else want = t->state == WINDOW_UNKNOWN ? WINDOW_CLOSED : t->state;   /* Hysterese-Band */

    if (want == t->state) {
        t->cnt = 0;
        return false;
    }
    if (want != t->cand) {
        t->cand = want;
        t->cnt = 0;
    }
    if (++t->cnt >= WIN_DEBOUNCE_COUNT) {
        t->state = want;
        t->cnt = 0;
        return true;
    }
    return false;
}
