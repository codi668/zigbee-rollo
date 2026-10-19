/*
 * Zentrale Pinbelegung und Board-Konstanten
 * Zigbee-Rollosteuerplatine mit ESP32-H2FH4S
 *
 * GPIO26/27 (USB-Serial/JTAG) NICHT anfassen.
 */
#pragma once

#include "driver/gpio.h"

/* ---------------- DRV8833 Motortreiber ---------------- */
#define PIN_DRV_NSLEEP      GPIO_NUM_0   /* Ausgang, LOW = Treiber aus (Standard) */
#define PIN_DRV_NFAULT      GPIO_NUM_2   /* Eingang, externer 10k Pull-up, LOW = Fehler */

/* ---------------- Analog ---------------- */
#define PIN_VBAT_ADC        GPIO_NUM_1   /* ADC1_CH0, Teiler 100k/100k */
#define PIN_NTC_ADC         GPIO_NUM_4   /* ADC1_CH3, 10k gegen 3V3, NTC 10k gegen GND */
#define VBAT_DIVIDER        2.0f         /* Akkuspannung = ADC x 2 */

#define NTC_R_FIXED_OHM     10000.0f     /* fester Widerstand gegen 3V3 */
#define NTC_R25_OHM         10000.0f     /* NTC bei 25 Grad C */
#define NTC_BETA            3380.0f
#define NTC_VREF_V          3.3f         /* Annahme: 3V3-Schiene = 3,30 V (wird nicht gemessen) */

/* ---------------- Lader / LED / Taste ---------------- */
#define PIN_CHRG_N          GPIO_NUM_5   /* TP4056 CHRG, Open-Drain, LOW = laedt */
#define PIN_LED             GPIO_NUM_8   /* gegen 3V3 -> aktiv LOW (Strapping-Pin) */
#define LED_ACTIVE_LEVEL    0
#define PIN_BOOT_BTN        GPIO_NUM_9   /* 10k Pull-up, gedrueckt = LOW (Strapping-Pin) */

/* ---------------- I2C / LIS2DH12 ---------------- */
#define PIN_I2C_SDA         GPIO_NUM_3   /* externe 4,7k Pull-ups */
#define PIN_I2C_SCL         GPIO_NUM_25
#define LIS2DH12_ADDR       0x18

/* ---------------- Motoren (DRV8833) ---------------- */
#define PIN_M1_IN1          GPIO_NUM_10  /* AIN1 */
#define PIN_M1_IN2          GPIO_NUM_11  /* AIN2 */
#define PIN_M2_IN1          GPIO_NUM_12  /* BIN1 */
#define PIN_M2_IN2          GPIO_NUM_22  /* BIN2 */

#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_DUTY_DEFAULT  50           /* Prozent */
#define MOTOR_DUTY_MAX      100          /* Obergrenze in Prozent */
#define MOTOR_RUN_MAX_MS    60000        /* jede Fahrt max. 60 s (Sicherheitsgrenze) */
#define MOTOR_RUN_DEFAULT_MS 2000        /* Standarddauer ohne Angabe */
#define STALL_RUN_MS        3000         /* Dauer des Blockiertests */

/* ---------------- Encoder ---------------- */
#define PIN_ENC1_A          GPIO_NUM_13  /* JGA25-370 */
#define PIN_ENC1_B          GPIO_NUM_14
#define PIN_ENC2_A          GPIO_NUM_23  /* Waveshare N20 */
#define PIN_ENC2_B          GPIO_NUM_24

/*
 * Impulse pro Abtriebsumdrehung bei x1-Zaehlung (eine Flanke eines Kanals).
 * Gezaehlt wird in der Firmware x4 (Quadratur), also Counts/Umdrehung = PPR_X1 * 4.
 *
 * Encoder 2 (Waveshare N20): 1050 Impulse/Umdrehung bei x1 laut Vorgabe.
 *
 * TODO: Encoder 1 (JGA25-370) - Impulszahl ist NICHT bekannt (haengt von
 *       Getriebeuebersetzung und Encoder-Variante ab, z. B. 11 PPR Motorwelle
 *       mal Untersetzung). 0 = unbekannt: dann werden nur Counts, keine rpm
 *       angezeigt. Wert eintragen, sobald per "enc 1 hand" (Abtriebswelle genau
 *       eine Umdrehung von Hand drehen, Counts/4 ablesen) ermittelt.
 */
#define ENC1_PPR_X1         0            /* TODO: unbekannt */
#define ENC2_PPR_X1         1050
