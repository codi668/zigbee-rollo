/*
 * Zigbee-Rollosteuerung – Platzhalter-main (Teil 1).
 *
 * Initialisiert alle Module von unten nach oben und gibt einen Selbsttest über
 * USB-Serial/JTAG aus. Es wird KEIN Motor bewegt: motor_ctrl startet mit
 * nSLEEP LOW und bekommt hier keinen Fahrbefehl.
 *
 * Bedienung im Platzhalter (zum Prüfen von Taste und LED):
 *   kurz drücken     -> nächstes LED-Muster (aus, Pairing, Kalibrierung, Fehler)
 *   3 s / 10 s       -> nur Log-Ausgabe (Bedeutung kommt mit app_state in Teil 2)
 *
 * Teil 2 ersetzt diese Datei durch main.c + app_state.c + zb_device.c.
 */
#include <inttypes.h>
#include <stdio.h>
#include "battery.h"
#include "config.h"
#include "drv8833.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ctrl.h"
#include "storage.h"
#include "ui.h"
#ifdef ROLLO_WINDOW_SENSOR
#include "window_sensor.h"
#endif

static const char *TAG = "main";

static int s_fail;   /* nur in dieser Datei: Anzahl fehlgeschlagener Selbsttest-Schritte */

static void check(const char *what, esp_err_t e)
{
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "[ OK ] %s", what);
    } else {
        ESP_LOGE(TAG, "[FAIL] %s: %s", what, esp_err_to_name(e));
        s_fail++;
    }
}

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "Einschalten";
    case ESP_RST_SW:       return "Software";
    case ESP_RST_PANIC:    return "Panic";
    case ESP_RST_INT_WDT:  return "Interrupt-WDT";
    case ESP_RST_TASK_WDT: return "Task-WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "Brownout";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_JTAG:     return "JTAG";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep";
    default:               return "sonstiger";
    }
}

/* Callbacks: nur loggen (Teil 2: Ereignis in die app_queue legen) */
static void on_motor_event(const motor_event_t *e, void *ctx)
{
    (void)ctx;
    if (e->type == MOTOR_EVT_PROGRESS)
        ESP_LOGI(TAG, "Motor %d: Pos %" PRId32, e->id + 1, e->pos);
}

static void on_ui_event(ui_event_t e, void *ctx)
{
    (void)ctx;
    if (e == UI_EVT_BTN_SHORT) {
        led_pattern_t next = (led_pattern_t)((ui_get_led() + 1) % LED_PAT_ON);  /* ON auslassen */
        ui_set_led(next);
        static const char *const names[] = { "aus", "Pairing", "Kalibrierung", "Fehler" };
        ESP_LOGI(TAG, "LED-Muster: %s", names[next]);
    } else {
        ESP_LOGI(TAG, "Taste '%s' (Platzhalter: keine Aktion)", ui_event_str(e));
    }
}

static void print_battery(void)
{
    battery_status_t b;
    esp_err_t e = battery_measure(&b);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Akku-Messung: %s", esp_err_to_name(e));
        return;
    }
    const char *ntc = b.ntc == BATT_NTC_OPEN ? "offen" : b.ntc == BATT_NTC_SHORT ? "Kurzschluss" : NULL;
    if (ntc)
        ESP_LOGI(TAG, "Akku %u mV, %u %%, %s, Sperre %s, NTC %s", b.vbat_mv, b.percent,
                 b.charging ? "laedt" : "laedt nicht", b.lockout ? "AKTIV" : "aus", ntc);
    else
        ESP_LOGI(TAG, "Akku %u mV, %u %%, %s, Sperre %s, NTC %.1f C", b.vbat_mv, b.percent,
                 b.charging ? "laedt" : "laedt nicht", b.lockout ? "AKTIV" : "aus", b.temp_c);
}

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "=== Zigbee-Rollosteuerung %s (Teil 1, Platzhalter-main) ===", app->version);
    ESP_LOGI(TAG, "Reset-Grund: %s", reset_reason_str(esp_reset_reason()));

    /* Speicher zuerst: Kalibrierung und moving-Flag */
    check("storage_init (NVS)", storage_init());
    storage_data_t sd;
    if (storage_load(&sd) == ESP_OK) {
        ESP_LOGI(TAG, "NVS: Heben %s (max %" PRId32 ", pos %" PRId32 "), Wenden %s (max %" PRId32 ", pos %" PRId32 ")",
                 sd.max[0] ? "kalibriert" : "unkalibriert", sd.max[0], sd.pos[0],
                 sd.max[1] ? "kalibriert" : "unkalibriert", sd.max[1], sd.pos[1]);
        if (sd.moving)
            ESP_LOGW(TAG, "NVS: moving = 1 -> Neustart waehrend einer Fahrt, Position unsicher (neu referenzieren)");
    }

    /* Motoren: Treiber + Encoder + motor_task, Motoren bleiben AUS */
    check("motor_ctrl_init (DRV8833, Encoder, motor_task)", motor_ctrl_init(on_motor_event, NULL));
    ESP_LOGI(TAG, "DRV8833: nSLEEP %s, nFAULT-Pin %s", drv8833_is_enabled() ? "HIGH (!)" : "LOW (aus)",
             drv8833_fault_active() ? "LOW" : "HIGH");

    check("battery_init (ADC, CHRG_N)", battery_init());
    print_battery();

    check("ui_init (Taste, LED)", ui_init(on_ui_event, NULL));

#ifdef ROLLO_WINDOW_SENSOR
    check("window_sensor_init", window_sensor_init());
    if (window_sensor_available() && sd.has_win_ref) {
        lis2dh12_mg_t ref = { sd.win_ref.x, sd.win_ref.y, sd.win_ref.z };
        window_sensor_set_reference(&ref);
    }
    ESP_LOGI(TAG, "Fenstersensor: %s", window_sensor_available() ? "vorhanden" : "nicht gefunden");
#else
    /* SENSOR: nicht bestückt, deaktiviert – zum Aktivieren ROLLO_WINDOW_SENSOR in config.h einkommentieren */
    ESP_LOGI(TAG, "Fenstersensor: deaktiviert (ROLLO_WINDOW_SENSOR), I2C wird nicht benutzt");
#endif

    if (s_fail == 0) ESP_LOGI(TAG, "Selbsttest: alle Module initialisiert");
    else ESP_LOGE(TAG, "Selbsttest: %d Schritt(e) fehlgeschlagen", s_fail);

    /* Laufende Anzeige: Encoder-Positionen bei Änderung (Welle von Hand drehen),
     * Akku jede Minute (nur im Stillstand). */
    int32_t last[MOTOR_COUNT] = { motor_ctrl_get_position(MOTOR_LIFT), motor_ctrl_get_position(MOTOR_TILT) };
    uint32_t sec = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        sec++;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int32_t p = motor_ctrl_get_position((motor_id_t)i);
            if (p != last[i]) {
                ESP_LOGI(TAG, "Encoder %d: %" PRId32 " Schritte", i + 1, p);
                last[i] = p;
            }
        }
        if (sec % 60 == 0 && !motor_ctrl_any_busy()) print_battery();
    }
}
