/*
 * Töne bei Ereignissen. Laufen in einer kurzlebigen Task, damit app_task nicht ~1 s blockiert.
 * Nach einem nFAULT ist der Treiber ausgeschaltet; wir warten kurz, bis er sich erholt hat, und versuchen
 * den Ton ein paar Mal. Meldet der Treiber den Fehler weiter (nFAULT LOW), bleibt es still.
 */
#include "sounds.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ctrl.h"

static const char *TAG = "sound";

static const motor_note_t SND_FAULT[] = {
    { 880, 150 }, { 0, 50 }, { 880, 150 }, { 0, 50 }, { 660, 150 }, { 0, 50 }, { 440, 450 },
};

static const motor_note_t SND_START[] = {
    { 1047, 110 }, { 1319, 110 }, { 1568, 110 }, { 2093, 260 },
};

static volatile bool s_busy;

static void sound_task(void *arg)
{
    sound_t s = (sound_t)(uintptr_t)arg;
    const motor_note_t *notes = NULL;
    size_t n = 0;
    if (s == SOUND_FAULT) {
        notes = SND_FAULT;
        n = sizeof(SND_FAULT) / sizeof(SND_FAULT[0]);
    } else if (s == SOUND_START) {
        notes = SND_START;
        n = sizeof(SND_START) / sizeof(SND_START[0]);
    }
    esp_err_t e = ESP_ERR_INVALID_ARG;
    for (int attempt = 0; notes && attempt < 4; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(attempt == 0 ? 350 : 600));   /* Treiber nach dem Fehler erholen lassen */
        e = motor_ctrl_tune(MOTOR_TILT, notes, n, BEEP_DUTY_DEFAULT_PCT * 10);
        if (e == ESP_OK || e != ESP_ERR_INVALID_STATE) break;
    }
    const char *name = (s == SOUND_START) ? "Startton" : "Fehlerton";
    if (e == ESP_OK) ESP_LOGI(TAG, "%s gespielt", name);
    else ESP_LOGW(TAG, "%s nicht moeglich: %s (nFAULT liegt weiter an oder Motor beschaeftigt)", name, esp_err_to_name(e));
    s_busy = false;
    vTaskDelete(NULL);
}

void sound_play_async(sound_t s)
{
    if (s_busy) return;
    s_busy = true;
    if (xTaskCreate(sound_task, "sound", 3072, (void *)(uintptr_t)s, 3, NULL) != pdPASS) s_busy = false;
}
