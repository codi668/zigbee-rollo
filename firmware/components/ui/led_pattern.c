/* LED-Muster – reine Logik, Host-getestet (test_host/test_led_pattern.c) */
#include "led_pattern.h"
#include "config.h"

uint32_t led_pattern_period(led_pattern_t p)
{
    switch (p) {
    case LED_PAT_PAIRING:     return LED_FAST_ON_MS + LED_FAST_OFF_MS;
    case LED_PAT_CALIBRATION: return LED_SLOW_ON_MS + LED_SLOW_OFF_MS;
    case LED_PAT_ERROR:       return 2 * LED_ERR_FLASH_MS + LED_ERR_GAP_MS + LED_ERR_PAUSE_MS;
    default:                  return 0;
    }
}

bool led_pattern_level(led_pattern_t p, uint32_t t)
{
    uint32_t per = led_pattern_period(p);
    if (per) t %= per;
    switch (p) {
    case LED_PAT_PAIRING:     return t < LED_FAST_ON_MS;
    case LED_PAT_CALIBRATION: return t < LED_SLOW_ON_MS;
    case LED_PAT_ERROR:
        return t < LED_ERR_FLASH_MS ||
               (t >= LED_ERR_FLASH_MS + LED_ERR_GAP_MS && t < 2 * LED_ERR_FLASH_MS + LED_ERR_GAP_MS);
    case LED_PAT_ON:          return true;
    default:                  return false;
    }
}
