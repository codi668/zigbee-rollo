/*
 * Zigbee-Rollosteuerung – Start (Teil 2).
 *
 * Reihenfolge:
 *   1. Banner, Reset-Grund (Brownout-Warnung)
 *   2. storage (NVS: Kalibrierung, Position, moving-Flag)
 *   3. app_state_init: app_queue anlegen (Callbacks der folgenden Module legen dort ab)
 *   4. motor_ctrl (DRV8833 + Encoder + motor_task) – Motoren bleiben AUS (nSLEEP LOW),
 *      bis die Applikation einen Befehl gibt
 *   5. battery + erste Messung (Motoren stehen sicher) -> Unterspannungssperre vor
 *      dem ersten Fahrbefehl bekannt
 *   6. ui (Taste, LED)
 *   7. sensor_task (Akku alle 10 min / nach Fahrten), zb_device_init (Radio)
 *   8. app_state_start: Start-Ereignis NEUSTART -> BEREIT/UNKALIBRIERT/AKKU_LEER, app_task
 *      (beim Task-Watchdog angemeldet; motor_task meldet sich während Fahrten an)
 *   9. zb_task (Zigbee-Stack), Debug-Konsole
 *
 * Brownout-Erkennung: CONFIG_ESP_BROWNOUT_DET=y (sdkconfig.defaults).
 */
#include <stdio.h>
#include "app_state.h"
#include "battery.h"
#include "board_pins.h"
#include "config.h"
#include "console_cmds.h"
#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ctrl.h"
#include "sensor_task.h"
#include "storage.h"
#include "ui.h"
#include "zb_device.h"
#ifdef ROLLO_POWERTEST
#include "powertest.h"
#endif
#ifdef ROLLO_LIGHT_SLEEP
#include "driver/gpio.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#endif
#ifdef ROLLO_WINDOW_SENSOR
#include "window_sensor.h"
#endif

static const char *TAG = "main";

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "Einschalten";
    case ESP_RST_SW:        return "Software";
    case ESP_RST_PANIC:     return "Panic";
    case ESP_RST_INT_WDT:   return "Interrupt-WDT";
    case ESP_RST_TASK_WDT:  return "Task-WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep";
    default:                return "sonstiger";
    }
}

/** Fehler beim Start: loggen, weitermachen (Motoren bleiben in jedem Fall aus) */
static bool check(const char *what, esp_err_t e)
{
    if (e == ESP_OK) return true;
    ESP_LOGE(TAG, "[FAIL] %s: %s", what, esp_err_to_name(e));
    return false;
}

#ifdef ROLLO_LIGHT_SLEEP
/* Nach Zigbee-SDK-Beispiel "light sleep": automatischer Light Sleep, CPU-Takt fest. */
static void power_save_init(void);

static void power_save_timer_cb(void *arg)
{
    (void)arg;
    power_save_init();
}

static void power_save_init(void)
{
    esp_pm_config_t pm = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    check("esp_pm_configure (Light Sleep)", esp_pm_configure(&pm));
    /* Taste weckt nur per Pegel (Flanken-Interrupts wecken nicht). ui.c sperrt dafür im
     * Light-Sleep-Betrieb den Pin-Interrupt in der ISR und gibt ihn nach dem Loslassen
     * wieder frei (sonst Interrupt-Sturm bei Pegel-Interrupt). UNGETESTET. */
    check("gpio_wakeup_enable (Taste)", gpio_wakeup_enable(PIN_BOOT_BTN, GPIO_INTR_LOW_LEVEL));
    check("esp_sleep_enable_gpio_wakeup", esp_sleep_enable_gpio_wakeup());
    ESP_LOGW(TAG, "Light Sleep AKTIV (ROLLO_LIGHT_SLEEP): USB-Konsole kann im Schlaf abbrechen");
}
#endif

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGI(TAG, "=== Zigbee-Rollosteuerung %s (ESP-IDF %s) ===", app->version, esp_get_idf_version());
    ESP_LOGI(TAG, "Reset-Grund: %s", reset_reason_str(rr));
    if (rr == ESP_RST_BROWNOUT)
        ESP_LOGW(TAG, "Brownout! Versorgung pruefen (USB-Kabel/Hub, Akku angesteckt?) / Anlaufstrom (KICK_DUTY) reduzieren");
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_PANIC)
        ESP_LOGW(TAG, "Neustart nach Absturz/Watchdog – Log davor pruefen");
#ifdef ROLLO_POWERTEST
    /* Leistungstest-Variante: Zustand aus dem RTC-Speicher, Zustände 4–6 und 9 ohne normale Applikation */
    pt_cfg_t pt;
    pt_begin(&pt);
    if (!pt.full) pt_run_minimal(&pt);
    zb_device_set_sleepy(pt.sleepy);
#endif

    /* 2. Speicher */
    storage_data_t nv = { 0 };
    if (check("storage_init (NVS)", storage_init())) storage_load(&nv);
    ESP_LOGI(TAG, "NVS: Heben %s (unten %ld, Pos %ld), Wenden %s (unten %ld, Pos %ld)%s",
             nv.max[0] ? "kalibriert" : "unkalibriert", (long)nv.max[0], (long)nv.pos[0],
             nv.max[1] ? "kalibriert" : "unkalibriert", (long)nv.max[1], (long)nv.pos[1],
             nv.moving ? ", moving = 1 (Neustart waehrend Fahrt -> Referenzfahrt)" : "");

    /* 3. Applikation (Queue) */
    ESP_ERROR_CHECK(app_state_init());

    /* 4. Motoren – bleiben aus */
    check("motor_ctrl_init", motor_ctrl_init(app_on_motor_event, NULL));
    motor_ctrl_set_margins(nv.margin_on);   /* vor dem Start-Ereignis, das die Endpunkte setzt */

    /* 5. Akku: erste Messung vor jedem Fahrbefehl */
    battery_status_t batt;
    bool batt_ok = check("battery_init", battery_init()) && check("battery_measure", battery_measure(&batt));
    if (!batt_ok) ESP_LOGW(TAG, "Akku unbekannt: Unterspannungssperre erst nach der naechsten Messung aktiv");
    else if (batt.vbat_mv > BATT_SUSPECT_HIGH_MV)
        ESP_LOGW(TAG, "Akkuspannung %u mV ungewoehnlich hoch (> %d mV): Akku angesteckt? Ohne Akku bricht 3V3 beim Funkstart ein (Brownout)",
                 batt.vbat_mv, BATT_SUSPECT_HIGH_MV);

    /* 6. Taste, LED */
    check("ui_init", ui_init(app_on_ui_event, NULL));

#ifdef ROLLO_WINDOW_SENSOR
    /* SENSOR: nicht bestückt, deaktiviert – zum Aktivieren ROLLO_WINDOW_SENSOR in config.h */
    check("window_sensor_init", window_sensor_init());
    if (window_sensor_available() && nv.has_win_ref) {
        lis2dh12_mg_t ref = { nv.win_ref.x, nv.win_ref.y, nv.win_ref.z };
        window_sensor_set_reference(&ref);
    }
#else
    /* SENSOR: nicht bestückt, deaktiviert – zum Aktivieren ROLLO_WINDOW_SENSOR in config.h einkommentieren */
    ESP_LOGI(TAG, "Fenstersensor: deaktiviert (ROLLO_WINDOW_SENSOR)");
#endif

#ifdef ROLLO_LIGHT_SLEEP
    {
        /* Light Sleep erst nach ROLLO_LIGHT_SLEEP_DELAY_MS (USB-Konsole fällt dann weg) */
        static esp_timer_handle_t t;
        const esp_timer_create_args_t a = { .callback = power_save_timer_cb, .name = "ls_delay" };
        if (check("ls_delay timer", esp_timer_create(&a, &t)))
            check("ls_delay start", esp_timer_start_once(t, (uint64_t)ROLLO_LIGHT_SLEEP_DELAY_MS * 1000));
        ESP_LOGW(TAG, "Light Sleep wird in %d s aktiviert (USB-Konsole faellt dann weg)", ROLLO_LIGHT_SLEEP_DELAY_MS / 1000);
    }
#endif

    /* 7. Messungen, Radio */
    if (rr == ESP_RST_BROWNOUT) {
        /* Reset wegen Spannungseinbruch: erst warten, damit die Versorgung sich erholt und keine
         * Reset-Schleife entsteht (jeder Neustart startet das Funkmodul = Stromspitze). */
        ESP_LOGW(TAG, "Nach Brownout %d ms warten, bevor das Funkmodul startet", BROWNOUT_BACKOFF_MS);
        vTaskDelay(pdMS_TO_TICKS(BROWNOUT_BACKOFF_MS));
    }
    check("sensor_task_start", sensor_task_start());
#ifdef ROLLO_POWERTEST
    if (pt.zigbee)
#endif
    check("zb_device_init (Radio)", zb_device_init());

    /* 8. Zustandsautomat */
    ESP_ERROR_CHECK(app_state_start(&nv, batt_ok ? &batt : NULL));

    /* 9. Zigbee, Konsole */
#ifdef ROLLO_POWERTEST
    if (pt.zigbee)
#endif
    check("zb_device_start", zb_device_start());
    check("Konsole", console_cmds_start());

    vTaskDelay(pdMS_TO_TICKS(300));   /* NEUSTART verarbeiten lassen */
#ifdef ROLLO_POWERTEST
    pt_after_init(&pt);
#endif
    app_state_log_status();
#ifdef ROLLO_DEBUG_CONSOLE
    ESP_LOGI(TAG, "Debug-Konsole: 'help' fuer Befehle (status, up, down, stop, goto, jog, cal, ack, batt)");
#endif
}
