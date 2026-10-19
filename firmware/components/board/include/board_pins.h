/**
 * @file board_pins.h
 * @brief GPIO-Belegung der Zigbee-Rollosteuerplatine Rev 0.1 (ESP32-H2FH4S).
 *
 * Alle Pin-Nummern stehen nur hier; ein Wechsel auf Rev 0.2 braucht nur
 * Änderungen in dieser Datei. Einstellwerte (Schwellen, PWM, Zeiten) stehen
 * in config.h.
 *
 * GPIO26/27 (USB-Serial/JTAG) NIE anfassen.
 */
#pragma once

#include "driver/gpio.h"

/* ---------------- DRV8833 Motortreiber ---------------- */
#define PIN_DRV_NSLEEP      GPIO_NUM_0   /**< Ausgang, LOW = Treiber aus (Standard, interner Pull-down im DRV8833) */
#define PIN_DRV_NFAULT      GPIO_NUM_2   /**< Eingang, externer 10k Pull-up, LOW = Fehler. KEIN interner Pull-up! */

#define PIN_M1_IN1          GPIO_NUM_10  /**< AIN1, Motor 1 Heben (JGA25-370) */
#define PIN_M1_IN2          GPIO_NUM_11  /**< AIN2 */
#define PIN_M2_IN1          GPIO_NUM_12  /**< BIN1, Motor 2 Wenden (N20) */
#define PIN_M2_IN2          GPIO_NUM_22  /**< BIN2 */

/* ---------------- Encoder (interne Pull-ups, externe nicht bestückt) ---------------- */
#define PIN_ENC1_A          GPIO_NUM_13  /**< JGA25-370, PCNT Unit 0 */
#define PIN_ENC1_B          GPIO_NUM_14
#define PIN_ENC2_A          GPIO_NUM_23  /**< N20, PCNT Unit 1 */
#define PIN_ENC2_B          GPIO_NUM_24

/* ---------------- Analog (ADC1 Oneshot) ---------------- */
#define PIN_VBAT_ADC        GPIO_NUM_1   /**< ADC1_CH0, Teiler 100k/100k */
#define PIN_NTC_ADC         GPIO_NUM_4   /**< ADC1_CH3, 10k gegen 3V3, NTC 10k gegen GND */

/* ---------------- Lader / LED / Taste ---------------- */
#define PIN_CHRG_N          GPIO_NUM_5   /**< TP4056 CHRG, Open-Drain, interner Pull-up, LOW = lädt */
#define PIN_LED             GPIO_NUM_8   /**< LED gegen 3V3 -> aktiv LOW (Strapping-Pin) */
#define LED_ACTIVE_LEVEL    0
#define PIN_BOOT_BTN        GPIO_NUM_9   /**< BOOT-Taste, 10k Pull-up, gedrückt = LOW (Strapping-Pin) */
#define BTN_ACTIVE_LEVEL    0

/* ---------------- I2C / LIS2DH12 (auf Rev 0.1 NICHT bestückt) ---------------- */
#define PIN_I2C_SDA         GPIO_NUM_3
#define PIN_I2C_SCL         GPIO_NUM_25
#define LIS2DH12_ADDR       0x18         /**< SA0 = 0 */
