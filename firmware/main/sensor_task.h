/**
 * @file sensor_task.h
 * @brief sensor_task: Akku- und NTC-Messung im Stillstand (Spezifikation "Tasks").
 *
 * Misst alle SENSOR_BATTERY_PERIOD_MS (10 min) und auf Anforderung (nach jeder
 * Fahrt, BATT_SETTLE_MS später) und legt das Ergebnis in die app_queue.
 * Gemessen wird nur, wenn kein Motor läuft. Damit zwischen der Prüfung und der
 * Messung keine Fahrt beginnt, nimmt die app_task vor jedem Fahrbefehl
 * sensor_lock() – die Messung hält denselben Mutex.
 *
 * SENSOR: nicht bestückt, deaktiviert – Fenstersensor (1 s) nur mit ROLLO_WINDOW_SENSOR.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/** @brief Mutex anlegen und sensor_task starten (erste periodische Messung nach 10 min). */
esp_err_t sensor_task_start(void);

/** @brief Messung in delay_ms anfordern (aus jeder Task). */
void sensor_request_measure(uint32_t delay_ms);

/**
 * @brief Keine Messung (auch keine periodische) vor Ablauf von delay_ms, ohne eine
 * Messung anzufordern. Aufruf bei jedem Fahrtende (aus jeder Task, nicht aus ISR).
 */
void sensor_hold_off(uint32_t delay_ms);

/** @brief Vor einem Fahrbefehl: keine Messung gleichzeitig (max. ~40 ms Wartezeit). */
void sensor_lock(void);
void sensor_unlock(void);
