/* Achsen-Umrechnungen – reine Logik, auf dem Host getestet (test_host/test_axis.c) */
#include "axis.h"

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

void axis_set_endpoints(axis_t *a, int32_t end_top, int32_t end_bottom,
                        uint16_t margin_top_permille, uint16_t margin_bottom_permille)
{
    a->end_top = end_top;
    a->end_bottom = end_bottom;
    int32_t travel = abs32(end_bottom - end_top);
    int64_t mt = (int64_t)travel * margin_top_permille / 1000;
    int64_t mb = (int64_t)travel * margin_bottom_permille / 1000;
    /* Ränder dürfen sich nicht überlappen */
    if (mt + mb > travel) {
        mt = travel / 2;
        mb = travel - mt;
    }
    a->margin_top = (int32_t)mt;
    a->margin_bottom = (int32_t)mb;
    a->calibrated = true;
}

void axis_clear(axis_t *a)
{
    a->end_top = 0;
    a->end_bottom = 0;
    a->margin_top = 0;
    a->margin_bottom = 0;
    a->calibrated = false;
}

int32_t axis_travel(const axis_t *a)
{
    return a->calibrated ? abs32(a->end_bottom - a->end_top) : 0;
}

int32_t axis_limit_lo(const axis_t *a)
{
    return a->end_top < a->end_bottom ? a->end_top : a->end_bottom;
}

int32_t axis_limit_hi(const axis_t *a)
{
    return a->end_top > a->end_bottom ? a->end_top : a->end_bottom;
}

int32_t axis_clamp(const axis_t *a, int32_t goal)
{
    if (!a->calibrated) return goal;
    int32_t lo = axis_limit_lo(a), hi = axis_limit_hi(a);
    if (goal < lo) return lo;
    if (goal > hi) return hi;
    return goal;
}

/* Richtung oben -> unten: +1 oder -1 */
static int32_t sense(const axis_t *a)
{
    return a->end_bottom >= a->end_top ? 1 : -1;
}

int32_t axis_percent_to_steps(const axis_t *a, uint8_t percent)
{
    if (!a->calibrated) return a->pos;
    if (percent > 100) percent = 100;
    int32_t s = sense(a);
    int32_t top_use = a->end_top + s * a->margin_top;
    int32_t usable = axis_travel(a) - a->margin_top - a->margin_bottom;
    if (usable < 0) usable = 0;
    int64_t off = ((int64_t)usable * percent + 50) / 100;
    return top_use + s * (int32_t)off;
}

uint8_t axis_steps_to_percent(const axis_t *a, int32_t pos)
{
    if (!a->calibrated) return 0;
    int32_t s = sense(a);
    int32_t usable = axis_travel(a) - a->margin_top - a->margin_bottom;
    if (usable <= 0) return 0;
    int64_t from_top = (int64_t)(pos - a->end_top) * s - a->margin_top;
    if (from_top <= 0) return 0;
    if (from_top >= usable) return 100;
    return (uint8_t)((from_top * 100 + usable / 2) / usable);
}

bool axis_at_limit(const axis_t *a, int32_t pos, int dir)
{
    if (!a->calibrated) return false;
    if (dir > 0) return pos >= axis_limit_hi(a);
    if (dir < 0) return pos <= axis_limit_lo(a);
    return false;
}
