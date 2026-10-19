/**
 * @file app_state.h
 * @brief Applikation (Schicht 2): Ereignis-Queue app_queue, app_task, Ausführung
 * des Zustandsautomaten (app_logic) auf motor_ctrl, storage, ui und Zigbee.
 *
 * Alle Quellen legen nur Ereignisse in die app_queue (nicht blockierend) –
 * Zigbee-Callbacks, Taste (ui_task), motor_task, sensor_task, Konsole. Die
 * Arbeit (Motorbefehle, NVS schreiben, Zigbee-Attribute) passiert ausschließlich
 * in der app_task. Motorbefehle der Konsole laufen über denselben Weg wie
 * Zigbee-Befehle, also durch alle Sicherheitsprüfungen des Zustandsautomaten.
 *
 * Die app_task ist beim Task-Watchdog angemeldet (wacht spätestens alle
 * APP_WDT_FEED_MS auf).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "app_logic.h"
#include "battery.h"
#include "motor_ctrl.h"
#include "storage.h"
#include "ui.h"

/** Netzzustand für LED und Diagnose (gemeldet von zb_device) */
typedef enum {
    APP_NET_OFFLINE = 0,   /**< Stack startet / Rejoin läuft */
    APP_NET_JOINING,       /**< Network Steering (Pairing): LED schnell */
    APP_NET_JOINED,
} app_net_t;

/** Herkunft eines Befehls (nur fürs Log) */
typedef enum {
    APP_SRC_ZIGBEE = 0,
    APP_SRC_CONSOLE,
    APP_SRC_INTERNAL,
} app_src_t;

/** Momentaufnahme für Konsole/Log */
typedef struct {
    app_st_t   st;
    app_err_t  err;
    uint8_t    err_axis;
    app_cal_t  cal_step;
    uint8_t    cal_axis;
    bool       calibrated[APP_AXIS_COUNT];
    bool       needs_homing[APP_AXIS_COUNT];
    int32_t    bottom[APP_AXIS_COUNT];
    bool       battery_valid;
    battery_status_t battery;
    app_net_t  net;
} app_snapshot_t;

/**
 * @brief app_queue und Zustand anlegen (vor motor_ctrl_init/ui_init, damit deren
 * Callbacks schon Ereignisse ablegen können).
 */
esp_err_t app_state_init(void);

/**
 * @brief app_task starten und das Start-Ereignis (NVS-Inhalt, erste Akku-Messung)
 * einspeisen: NEUSTART -> BEREIT / UNKALIBRIERT / AKKU_LEER.
 * @param nv    Inhalt des NVS (storage_load)
 * @param batt  erste Akku-Messung oder NULL (Messung fehlgeschlagen -> keine Sperre)
 */
esp_err_t app_state_start(const storage_data_t *nv, const battery_status_t *batt);

/** @brief Befehl einspeisen (aus jeder Task, nicht blockierend). @return false bei voller Queue */
bool app_post_cmd(app_cmd_t cmd, uint8_t axis, int32_t value, app_src_t src);

/** @brief Callback für motor_ctrl_init() (Kontext motor_task). */
void app_on_motor_event(const motor_event_t *e, void *ctx);

/** @brief Callback für ui_init() (Kontext ui_task). */
void app_on_ui_event(ui_event_t e, void *ctx);

/** @brief Akku-Messung einspeisen (sensor_task). */
void app_post_battery(const battery_status_t *b);

/** @brief Netzzustand (zb_device, Kontext zb_task). */
void app_post_net(app_net_t net);

/** @brief Identify an/aus (zb_device). */
void app_post_identify(bool on);

/** @brief Thread-sichere Momentaufnahme. */
void app_state_get_snapshot(app_snapshot_t *out);

/** @brief Zustand, Positionen und Akku ins Log schreiben (aus jeder Task). */
void app_state_log_status(void);
