/* Tasten-Logik – reine Logik, Host-getestet (test_host/test_btn_logic.c) */
#include "btn_logic.h"
#include "config.h"

void btn_logic_init(btn_logic_t *b, uint32_t now_ms, bool pressed)
{
    *b = (btn_logic_t){ .raw = pressed, .raw_t = now_ms, .stable = pressed, .armed = !pressed };
}

btn_event_t btn_logic_update(btn_logic_t *b, uint32_t now, bool pressed)
{
    if (pressed != b->raw) {
        b->raw = pressed;
        b->raw_t = now;
    }
    /* Flanke übernehmen, wenn der Rohpegel lange genug stabil ist */
    if (b->raw != b->stable && (uint32_t)(now - b->raw_t) >= BTN_DEBOUNCE_MS) {
        b->stable = b->raw;
        if (b->stable) {                          /* gedrückt */
            b->press_t = b->raw_t;                /* Beginn = erste stabile Flanke */
            b->hint_sent = false;
            b->very_long_sent = false;
        } else {                                  /* losgelassen */
            uint32_t held = b->raw_t - b->press_t;
            bool was_armed = b->armed;
            bool was_very_long = b->very_long_sent;
            b->armed = true;
            if (!was_armed || was_very_long) return BTN_EV_NONE;
            return held >= BTN_LONG_MS ? BTN_EV_LONG : BTN_EV_SHORT;
        }
    }
    if (b->stable && b->armed && !b->very_long_sent) {
        uint32_t held = now - b->press_t;
        if (held >= BTN_VERY_LONG_MS) {
            b->very_long_sent = true;
            return BTN_EV_VERY_LONG;
        }
        if (held >= BTN_LONG_MS && !b->hint_sent) {
            b->hint_sent = true;
            return BTN_EV_LONG_HINT;
        }
    }
    return BTN_EV_NONE;
}

bool btn_logic_busy(const btn_logic_t *b)
{
    return b->stable || b->raw != b->stable;
}
