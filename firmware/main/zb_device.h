/**
 * @file zb_device.h
 * @brief Zigbee-Schicht (Schicht 1): End Device mit Window Covering an Endpoint 1.
 *
 * Endpoint 1, Gerätetyp Window Covering (0x0202), Server-Cluster:
 *   Basic (Hersteller ROLLO_ZB_MANUFACTURER, Modell ROLLO_ZB_MODEL, Batterie),
 *   Identify, Power Configuration (BatteryVoltage, BatteryPercentageRemaining),
 *   Window Covering (Typ 0x08 Tilt Blind Lift+Tilt, CurrentPositionLift/TiltPercentage,
 *   ConfigStatus, Mode), optional Temperature Measurement (NTC, ROLLO_ZB_TEMPERATURE).
 * Endpoint 2 (IAS Zone, Fenstersensor): SENSOR: nicht bestückt, deaktiviert –
 *   nur mit ROLLO_WINDOW_SENSOR.
 *
 * Callbacks des Stacks (zb_task) erzeugen NUR Ereignisse in der app_queue.
 * Die Setter unten werden aus der app_task aufgerufen: Sie merken sich die
 * Werte (Schattenkopie) und schreiben sie zwischen esp_zb_lock_acquire()
 * (mit Timeout ZB_LOCK_TIMEOUT_MS) und esp_zb_lock_release() in den Stack.
 * Vor dem Start des Stacks landen die Werte nur in der Schattenkopie und
 * werden beim Anlegen der Cluster übernommen.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "battery.h"
#include "app_state.h"

/** @brief Radio/Plattform konfigurieren (vor zb_device_start, nach storage_init). */
esp_err_t zb_device_init(void);

/** @brief Sleepy End Device (Funk schläft zwischen den Polls) einschalten; vor zb_device_start().
 *         Standard: aus (nur mit ROLLO_LIGHT_SLEEP an). Für den Leistungstest (powertest.c). */
void zb_device_set_sleepy(bool on);

/** @brief zb_task starten (Stack, Netzbeitritt, Main Loop). */
esp_err_t zb_device_start(void);

/**
 * @brief Position einer Achse setzen.
 * @param axis  APP_AXIS_LIFT / APP_AXIS_TILT
 * @param attr  0…100 oder ZBL_POS_UNKNOWN (unkalibriert)
 * @param final true = Endwert, sofort melden; false = Zwischenstand (max. 1/s melden)
 */
void zb_device_set_position(uint8_t axis, uint8_t attr, bool final);

/** @brief Akku (+ NTC) setzen; aktiv gemeldet höchstens alle ZB_BATT_REPORT_S oder bei Sperrwechsel. */
void zb_device_set_battery(const battery_status_t *b);

/** @brief ConfigStatus (Operational) und Mode (Bit1 Kalibriermodus) setzen. */
void zb_device_set_status(bool operational, bool calibrating);

/**
 * @brief Werkseinstellung Zigbee: Netz verlassen (Leave), zb_storage löschen, Neustart.
 * esp_zb_factory_reset() startet selbst neu (geprüft: zb_nvram_erase() + esp_restart()).
 * Ist der Zigbee-Lock nicht zu bekommen, wird zb_storage direkt gelöscht und
 * neu gestartet (Notfallpfad). Kehrt nie zurück.
 */
void zb_device_factory_reset(void) __attribute__((noreturn));

/** @brief Aktueller Netzzustand. */
app_net_t zb_device_net_state(void);

/** @brief Netzdaten ins Log (PAN, Kanal, Adresse, Parent-LQI), aus jeder Task. */
void zb_device_log_info(void);
