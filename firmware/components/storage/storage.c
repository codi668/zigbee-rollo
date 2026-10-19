/*
 * storage: NVS-Wrapper mit RAM-Kopie. Geschrieben wird nur, wenn sich ein Wert
 * ändert; jede öffentliche Schreibfunktion endet mit genau einem nvs_commit().
 */
#include "storage.h"
#include <string.h>
#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "storage";

static const char *const KEY_MAX[STORAGE_AXIS_COUNT] = { "lift_max", "tilt_max" };
static const char *const KEY_POS[STORAGE_AXIS_COUNT] = { "lift_pos", "tilt_pos" };
#define KEY_MOVING "moving"
#define KEY_WREF_X "win_ref_x"
#define KEY_WREF_Y "win_ref_y"
#define KEY_WREF_Z "win_ref_z"

static nvs_handle_t s_nvs;
static SemaphoreHandle_t s_mtx;
static storage_data_t s_data;     /* RAM-Kopie, unter s_mtx */
static bool s_ready;

static bool valid_axis(storage_axis_t a) { return (unsigned)a < STORAGE_AXIS_COUNT; }

/* Fehlender Schlüssel ist kein Fehler: Standardwert behalten */
static esp_err_t get_i32(const char *key, int32_t *v)
{
    esp_err_t e = nvs_get_i32(s_nvs, key, v);
    return e == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : e;
}

static esp_err_t load_all(void)
{
    storage_data_t d = { 0 };
    esp_err_t e = ESP_OK;
    for (int i = 0; i < STORAGE_AXIS_COUNT && e == ESP_OK; i++) {
        e = get_i32(KEY_MAX[i], &d.max[i]);
        if (e == ESP_OK) e = get_i32(KEY_POS[i], &d.pos[i]);
    }
    uint8_t mv = 0;
    if (e == ESP_OK) {
        e = nvs_get_u8(s_nvs, KEY_MOVING, &mv);
        if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
        d.moving = mv != 0;
    }
    if (e == ESP_OK) {
        int16_t x, y, z;
        esp_err_t ex = nvs_get_i16(s_nvs, KEY_WREF_X, &x);
        esp_err_t ey = nvs_get_i16(s_nvs, KEY_WREF_Y, &y);
        esp_err_t ez = nvs_get_i16(s_nvs, KEY_WREF_Z, &z);
        if (ex == ESP_OK && ey == ESP_OK && ez == ESP_OK) {
            d.has_win_ref = true;
            d.win_ref = (storage_vec3_t){ x, y, z };
        } else if (ex != ESP_ERR_NVS_NOT_FOUND && ex != ESP_OK) {
            e = ex;
        }
    }
    if (e == ESP_OK) s_data = d;
    return e;
}

esp_err_t storage_init(void)
{
    if (s_ready) return ESP_OK;
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS-Partition unbrauchbar (%s) -> wird geloescht", esp_err_to_name(e));
        e = nvs_flash_erase();
        if (e == ESP_OK) e = nvs_flash_init();
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(e));
        return e;
    }
    if ((e = nvs_open(STORAGE_NVS_NAMESPACE, NVS_READWRITE, &s_nvs)) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(e));
        return e;
    }
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;
    if ((e = load_all()) != ESP_OK) {
        ESP_LOGE(TAG, "Lesen fehlgeschlagen: %s", esp_err_to_name(e));
        return e;
    }
    s_ready = true;
    ESP_LOGI(TAG, "geladen: lift_max %ld, tilt_max %ld, lift_pos %ld, tilt_pos %ld, moving %d, win_ref %s",
             (long)s_data.max[0], (long)s_data.max[1], (long)s_data.pos[0], (long)s_data.pos[1],
             s_data.moving, s_data.has_win_ref ? "ja" : "nein");
    return ESP_OK;
}

#define LOCK()   xSemaphoreTake(s_mtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_mtx)

esp_err_t storage_load(storage_data_t *out)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!out) return ESP_ERR_INVALID_ARG;
    LOCK();
    *out = s_data;
    UNLOCK();
    return ESP_OK;
}

bool storage_is_calibrated(storage_axis_t axis)
{
    if (!s_ready || !valid_axis(axis)) return false;
    LOCK();
    bool c = s_data.max[axis] != 0;
    UNLOCK();
    return c;
}

/* --- Schreibhelfer (unter LOCK), *dirty wird bei echter Änderung gesetzt --- */
static esp_err_t put_i32(const char *key, int32_t *cache, int32_t v, bool *dirty)
{
    if (*cache == v) return ESP_OK;
    esp_err_t e = nvs_set_i32(s_nvs, key, v);
    if (e == ESP_OK) {
        *cache = v;
        *dirty = true;
    }
    return e;
}

static esp_err_t put_moving(bool v, bool *dirty)
{
    if (s_data.moving == v) return ESP_OK;
    esp_err_t e = nvs_set_u8(s_nvs, KEY_MOVING, v ? 1 : 0);
    if (e == ESP_OK) {
        s_data.moving = v;
        *dirty = true;
    }
    return e;
}

static esp_err_t finish(esp_err_t e, bool dirty, const char *what)
{
    if (e == ESP_OK && dirty) e = nvs_commit(s_nvs);
    if (e != ESP_OK) ESP_LOGE(TAG, "%s: %s", what, esp_err_to_name(e));
    return e;
}

esp_err_t storage_save_calibration(storage_axis_t axis, int32_t max)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!valid_axis(axis) || max == 0) return ESP_ERR_INVALID_ARG;
    bool dirty = false;
    LOCK();
    esp_err_t e = finish(put_i32(KEY_MAX[axis], &s_data.max[axis], max, &dirty), dirty, "Kalibrierung speichern");
    UNLOCK();
    if (e == ESP_OK) ESP_LOGI(TAG, "%s = %ld", KEY_MAX[axis], (long)max);
    return e;
}

esp_err_t storage_clear_calibration(storage_axis_t axis)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!valid_axis(axis)) return ESP_ERR_INVALID_ARG;
    bool dirty = false;
    LOCK();
    esp_err_t e = finish(put_i32(KEY_MAX[axis], &s_data.max[axis], 0, &dirty), dirty, "Kalibrierung loeschen");
    UNLOCK();
    return e;
}

esp_err_t storage_motion_begin(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    bool dirty = false;
    LOCK();
    esp_err_t e = finish(put_moving(true, &dirty), dirty, "moving=1");
    UNLOCK();
    return e;
}

esp_err_t storage_motion_end(storage_axis_t axis, int32_t pos)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!valid_axis(axis)) return ESP_ERR_INVALID_ARG;
    bool dirty = false;
    LOCK();
    esp_err_t e = put_i32(KEY_POS[axis], &s_data.pos[axis], pos, &dirty);
    if (e == ESP_OK) e = put_moving(false, &dirty);
    e = finish(e, dirty, "Fahrtende speichern");
    UNLOCK();
    return e;
}

esp_err_t storage_save_position(storage_axis_t axis, int32_t pos)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!valid_axis(axis)) return ESP_ERR_INVALID_ARG;
    bool dirty = false;
    LOCK();
    esp_err_t e = finish(put_i32(KEY_POS[axis], &s_data.pos[axis], pos, &dirty), dirty, "Position speichern");
    UNLOCK();
    return e;
}

esp_err_t storage_save_win_ref(const storage_vec3_t *ref)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (!ref) return ESP_ERR_INVALID_ARG;
    LOCK();
    esp_err_t e = ESP_OK;
    bool dirty = false;
    if (!s_data.has_win_ref || memcmp(&s_data.win_ref, ref, sizeof(*ref)) != 0) {
        e = nvs_set_i16(s_nvs, KEY_WREF_X, ref->x);
        if (e == ESP_OK) e = nvs_set_i16(s_nvs, KEY_WREF_Y, ref->y);
        if (e == ESP_OK) e = nvs_set_i16(s_nvs, KEY_WREF_Z, ref->z);
        dirty = true;
    }
    e = finish(e, dirty, "Fenster-Referenz speichern");
    if (e == ESP_OK) {
        s_data.win_ref = *ref;
        s_data.has_win_ref = true;
    }
    UNLOCK();
    return e;
}

esp_err_t storage_erase_all(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    LOCK();
    esp_err_t e = nvs_erase_all(s_nvs);
    e = finish(e, true, "Werkseinstellung");
    if (e == ESP_OK) memset(&s_data, 0, sizeof(s_data));
    UNLOCK();
    if (e == ESP_OK) ESP_LOGW(TAG, "alle Werte geloescht (Werkseinstellung)");
    return e;
}
