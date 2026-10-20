/**
 * @file zb_logic.h
 * @brief Zigbee-Attributwerte und Wiederholungszeiten (reine Logik, ohne ESP-IDF, Host-getestet).
 *
 * Werte nach ZCL (Zigbee Cluster Library):
 * - Window Covering CurrentPositionLift/TiltPercentage: 0 = ganz offen, 100 = ganz zu,
 *   0xFF (= unbekannt) wird NICHT gemeldet: ZHA rechnet daraus eine Position von -155 %%.
 *   Unkalibriert wird 0 gemeldet. Die Achsen-Konvention (axis.h) ist
 *   identisch: 0 % = oben = offen, deshalb keine Umkehrung.
 * - ConfigStatus (8-Bit-Bitmap): Bit0 Operational, Bit1 Online, Bit2 Befehle umgekehrt,
 *   Bit3/4 Lift/Tilt Closed Loop, Bit5/6 Lift/Tilt Encoder.
 * - Mode (8-Bit-Bitmap): Bit0 Motorrichtung umgekehrt, Bit1 Kalibriermodus,
 *   Bit2 Wartungsmodus, Bit3 LED-Rückmeldung.
 * - Power Configuration BatteryVoltage: 100 mV je Digit; BatteryPercentageRemaining:
 *   0,5 % je Digit (200 = 100 %).
 * - Temperature Measurement MeasuredValue: 0,01 °C, 0x8000 = ungültig.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ZBL_POS_UNKNOWN          0      /**< Meldewert unkalibriert (0xFF wäre für ZHA -155 %) */
#define ZBL_TEMP_UNKNOWN         ((int16_t)0x8000)

#define ZBL_CFG_OPERATIONAL      0x01
#define ZBL_CFG_ONLINE           0x02
#define ZBL_CFG_REVERSED         0x04
#define ZBL_CFG_LIFT_CLOSED_LOOP 0x08
#define ZBL_CFG_TILT_CLOSED_LOOP 0x10
#define ZBL_CFG_LIFT_ENCODER     0x20
#define ZBL_CFG_TILT_ENCODER     0x40

#define ZBL_MODE_REVERSED        0x01
#define ZBL_MODE_CALIBRATION     0x02
#define ZBL_MODE_MAINTENANCE     0x04

/** @brief Positionsattribut: kalibriert -> 0…100 (begrenzt), sonst 0 (ZBL_POS_UNKNOWN). */
uint8_t zbl_position_attr(bool calibrated, uint8_t percent);

/** @brief BatteryVoltage in 100 mV, gerundet, 0…255. */
uint8_t zbl_battery_voltage_attr(uint32_t vbat_mv);

/** @brief BatteryPercentageRemaining 0…200. */
uint8_t zbl_battery_percent_attr(uint8_t percent_x2);

/** @brief MeasuredValue in 0,01 °C (gerundet, begrenzt) oder 0x8000 bei ungültig. */
int16_t zbl_temperature_attr(bool valid, float temp_c);

/** @brief ConfigStatus: Online, Closed Loop + Encoder für beide Achsen, Operational nach Zustand. */
uint8_t zbl_config_status(bool operational);

/** @brief Mode: Bit1 = Kalibriermodus, Bit0 = Richtung umgekehrt. */
uint8_t zbl_mode_attr(bool calibrating, bool reversed);

/**
 * @brief Wartezeit vor dem nächsten Steering-/Rejoin-Versuch (Akku schonen):
 * Versuch 0: 1 s, 1: 5 s, 2: 15 s, 3…9: 60 s, ab 10: 300 s.
 */
uint32_t zbl_backoff_ms(uint32_t attempt);

/**
 * @brief Positionsmeldung jetzt senden? Endwert immer, Zwischenstand frühestens
 * ZB_REPORT_PROGRESS_MIN_MS nach der letzten Meldung (Überlauf-fest).
 */
bool zbl_report_due(bool final, uint32_t now_ms, uint32_t last_ms, bool ever_sent);
