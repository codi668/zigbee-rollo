/*
 * sensor_task: Akku/NTC alle 10 min und nach Fahrten, nur im Stillstand.
 * SENSOR: nicht bestückt, deaktiviert – Fenstersensor nur mit ROLLO_WINDOW_SENSOR.
 */
#include "sensor_task.h"
#include "app_state.h"
#include "battery.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_ctrl.h"
#ifdef ROLLO_WINDOW_SENSOR
#include "window_sensor.h"
#endif

static const char *TAG = "sensor";

static SemaphoreHandle_t s_mutex;
static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_request_at_us;    /* 0 = keine Anforderung, unter s_lock */
static int64_t s_settle_until_us;  /* vorher keine Messung (Spannung nach Fahrt erholt sich), unter s_lock */

static int64_t now_us(void) { return esp_timer_get_time(); }

void sensor_lock(void)
{
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        ESP_LOGW(TAG, "Mess-Mutex nicht frei, Fahrbefehl trotzdem");
}

void sensor_unlock(void)
{
    if (s_mutex && xSemaphoreGetMutexHolder(s_mutex) == xTaskGetCurrentTaskHandle()) xSemaphoreGive(s_mutex);
}

void sensor_hold_off(uint32_t delay_ms)
{
    int64_t at = now_us() + (int64_t)delay_ms * 1000;
    taskENTER_CRITICAL(&s_lock);
    if (at > s_settle_until_us) s_settle_until_us = at;
    taskEXIT_CRITICAL(&s_lock);
}

void sensor_request_measure(uint32_t delay_ms)
{
    int64_t at = now_us() + (int64_t)delay_ms * 1000;
    taskENTER_CRITICAL(&s_lock);
    /* Spätere Anforderung gewinnt: eine frühere (z. B. Retry) darf die Erholzeit nach
     * dieser Fahrt nicht unterlaufen – sonst Messung im Spannungseinbruch -> falsche
     * Unterspannungssperre, die sich erst nach dem Laden löst. */
    if (!s_request_at_us || at > s_request_at_us) s_request_at_us = at;
    if (at > s_settle_until_us) s_settle_until_us = at;
    taskEXIT_CRITICAL(&s_lock);
    if (s_task) xTaskNotifyGive(s_task);
}

/** @return true = gemessen, false = Motor lief (später erneut) */
static bool measure_once(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    if (motor_ctrl_any_busy()) {
        xSemaphoreGive(s_mutex);
        return false;
    }
    battery_status_t b;
    esp_err_t e = battery_measure(&b);
    xSemaphoreGive(s_mutex);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Akku-Messung: %s", esp_err_to_name(e));
        return true;   /* nicht endlos wiederholen; nächste periodische Messung */
    }
    app_post_battery(&b);
    return true;
}

static void sensor_task(void *arg)
{
    (void)arg;
    int64_t next_periodic = now_us() + (int64_t)SENSOR_BATTERY_PERIOD_MS * 1000;
    for (;;) {
        int64_t t = now_us();
        taskENTER_CRITICAL(&s_lock);
        int64_t req = s_request_at_us;
        int64_t settle = s_settle_until_us;
        taskEXIT_CRITICAL(&s_lock);

        bool due_periodic = t >= next_periodic;
        bool due_request = req && t >= req;
        bool settled = t >= settle;
        if ((due_periodic || due_request) && settled) {
            if (measure_once()) {
                next_periodic = now_us() + (int64_t)SENSOR_BATTERY_PERIOD_MS * 1000;
                taskENTER_CRITICAL(&s_lock);
                if (s_request_at_us && s_request_at_us <= now_us()) s_request_at_us = 0;
                taskEXIT_CRITICAL(&s_lock);
            } else {
                /* Motor lief gerade: Anforderung (überschreibend) auf später verschieben */
                taskENTER_CRITICAL(&s_lock);
                s_request_at_us = now_us() + (int64_t)SENSOR_RETRY_MS * 1000;
                taskEXIT_CRITICAL(&s_lock);
                if (due_periodic) next_periodic = now_us() + (int64_t)SENSOR_BATTERY_PERIOD_MS * 1000;
            }
            continue;
        }

        int64_t wake = next_periodic;
        if (req && req < wake) wake = req;
        if (!settled && settle > wake) wake = settle;   /* fällige Messung nach der Erholzeit */
        int64_t wait_ms = (wake - t) / 1000 + 1;
#ifdef ROLLO_WINDOW_SENSOR
        /* SENSOR: nicht bestückt, deaktiviert – zum Aktivieren ROLLO_WINDOW_SENSOR in config.h.
         * TODO beim Aktivieren: Zustandswechsel an zb_device (IAS Zone, Endpoint 2) melden. */
        if (wait_ms > SENSOR_WINDOW_PERIOD_MS) wait_ms = SENSOR_WINDOW_PERIOD_MS;
        bool changed = false;
        window_state_t ws = window_sensor_poll(motor_ctrl_any_busy(), &changed);
        if (changed) ESP_LOGI(TAG, "Fenster: %s", window_state_str(ws));
#endif
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
    }
}

esp_err_t sensor_task_start(void)
{
    if (s_task) return ESP_ERR_INVALID_STATE;
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    if (xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, &s_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
