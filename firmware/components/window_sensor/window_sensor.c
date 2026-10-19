/* window_sensor: Ablauf um tilt_logic (rein) und lis2dh12 (I2C) */
#include "window_sensor.h"
#include "lis2dh12.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "window";

#define READ_ERR_LIMIT 3      /* so viele Lesefehler in Folge -> Zustand UNKNOWN */

static tilt_det_t s_det;
static bool s_avail;
static uint8_t s_read_errs;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

esp_err_t window_sensor_init(void)
{
    tilt_det_init(&s_det);
    s_avail = lis2dh12_init() == ESP_OK;
    if (!s_avail) ESP_LOGW(TAG, "kein LIS2DH12 - Fensterzustand bleibt 'unbekannt'");
    return ESP_OK;
}

bool window_sensor_available(void) { return s_avail; }
bool window_sensor_ready(void) { return s_avail && s_det.has_ref; }

void window_sensor_set_reference(const lis2dh12_mg_t *ref)
{
    if (ref) tilt_det_set_ref(&s_det, ref);
}

esp_err_t window_sensor_calibrate(lis2dh12_mg_t *ref_out)
{
    if (!s_avail) return ESP_ERR_NOT_FOUND;
    esp_err_t e = lis2dh12_set_rate(LIS2DH12_RATE_10HZ);
    if (e != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(200));           /* erste Messungen mit neuer Rate abwarten */
    int32_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (int i = 0; i < WIN_REF_SAMPLES; i++) {
        lis2dh12_mg_t v;
        if (lis2dh12_read_mg(&v) == ESP_OK) {
            sx += v.x; sy += v.y; sz += v.z;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(110));
    }
    lis2dh12_set_rate(LIS2DH12_RATE_1HZ);
    if (n == 0) return ESP_FAIL;
    lis2dh12_mg_t ref = { (int16_t)(sx / n), (int16_t)(sy / n), (int16_t)(sz / n) };
    tilt_det_set_ref(&s_det, &ref);
    if (ref_out) *ref_out = ref;
    ESP_LOGI(TAG, "Referenz eingelernt: %d/%d/%d mg (%d Messungen)", ref.x, ref.y, ref.z, n);
    return ESP_OK;
}

window_state_t window_sensor_poll(bool motor_busy, bool *changed)
{
    if (changed) *changed = false;
    if (!s_avail || !s_det.has_ref) return WINDOW_UNKNOWN;
    uint32_t t = now_ms();
    if (motor_busy) {
        tilt_det_note_motion(&s_det, t);
        return s_det.state;
    }
    if (tilt_det_holdoff(&s_det, t)) return s_det.state;

    lis2dh12_mg_t v;
    if (lis2dh12_read_mg(&v) != ESP_OK) {
        if (s_read_errs < 255) s_read_errs++;     /* sättigen: Meldung nur einmal */
        if (s_read_errs == READ_ERR_LIMIT) {
            ESP_LOGW(TAG, "Sensor antwortet nicht mehr -> Zustand unbekannt");
            bool ch = s_det.state != WINDOW_UNKNOWN;
            s_det.state = WINDOW_UNKNOWN;
            s_det.cnt = 0;
            if (changed) *changed = ch;
        }
        return s_det.state;
    }
    if (s_read_errs >= READ_ERR_LIMIT) ESP_LOGI(TAG, "Sensor antwortet wieder");
    s_read_errs = 0;
    bool ch = tilt_det_update(&s_det, &v);
    if (changed) *changed = ch;
    if (ch) ESP_LOGI(TAG, "Fenster %s (%.1f Grad)", window_state_str(s_det.state), s_det.last_angle);
    return s_det.state;
}

window_state_t window_sensor_get_state(void)
{
    return (s_avail && s_det.has_ref) ? s_det.state : WINDOW_UNKNOWN;
}

float window_sensor_last_angle(void)
{
    return s_det.last_angle;
}

const char *window_state_str(window_state_t s)
{
    switch (s) {
    case WINDOW_CLOSED: return "zu";
    case WINDOW_TILTED: return "gekippt";
    default:            return "unbekannt";
    }
}
