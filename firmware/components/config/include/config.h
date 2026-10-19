/**
 * @file config.h
 * @brief Zentrale Einstellwerte der Rollo-Firmware (Schwellen, Zeiten, PWM).
 *
 * Alles, was im Test eingestellt werden muss, steht hier und nur hier.
 * Die Datei enthält ausschließlich Zahlen-Makros und KEINE ESP-IDF-Includes,
 * damit die reine Logik (z. B. motion.c, battery_calc.c) auch auf dem PC mit
 * den Host-Tests übersetzt werden kann. Pins stehen getrennt in board_pins.h.
 *
 * Liegt als eigene Header-Komponente `config` unter components/, weil sowohl
 * die Komponenten (Treiber, Services) als auch main/ sie brauchen und eine
 * Komponente nicht von main abhängen kann.
 *
 * Einheiten: Duty in Prozent (0…100), Zeiten in ms, Wege in Encoder-Zählschritten
 * (Quadratur x4), Spannungen in mV.
 */
#pragma once

/* =========================================================================
 * PWM / DRV8833
 * ========================================================================= */
#define MOTOR_PWM_FREQ_HZ        20000   /**< 20 kHz, außerhalb des Hörbereichs */
#define MOTOR_PWM_RES_BITS       10      /**< 10 Bit -> 1024 Stufen */
#define MOTOR_DUTY_MAX_PCT       100     /**< harte Obergrenze für jeden Duty */

/**
 * Abklingmodus der Brücke.
 * 0 = Fast Decay (ein IN-Pin PWM, der andere LOW) – so wurde in hwtest/ gemessen
 *     (Anlauf N20 erst ab 70 % usw.), daher Standard.
 * 1 = Slow Decay (ein IN-Pin fest HIGH, der andere invertierte PWM) laut
 *     Spezifikation, gleichmäßigeres Moment bei kleiner Drehzahl.
 * TODO: Slow Decay auf der Hardware testen; Drehzahl je Duty ändert sich dabei,
 * MIN_DUTY/RUN_DUTY/Blockier-Schwellen danach neu messen.
 */
#define MOTOR_SLOW_DECAY         0

#define DRV8833_WAKE_MS          5       /**< Wartezeit nach nSLEEP HIGH (tWAKE max. 1 ms, wie hwtest großzügig) */

/* =========================================================================
 * motor_task
 * ========================================================================= */
#define MOTOR_TICK_MS            10      /**< Regeltakt der motor_task */
#define MOTOR_TASK_PRIO          6       /**< über app_task, unter Zigbee-Stack nicht nötig */
#define MOTOR_TASK_STACK         4096
#define MOTOR_QUEUE_LEN          8       /**< Befehlsqueue der motor_task */
#define MOTOR_BRAKE_MS           50      /**< Kurzschlussbremse (beide IN HIGH) am Ende jeder Fahrt */
#define MOTOR_DIR_PAUSE_MS       100     /**< Pause vor einem Richtungswechsel */
#define MOTOR_PROGRESS_MS        1000    /**< Abstand der Fortschrittsmeldungen während der Fahrt (Zigbee-Report) */
#define MOTOR_TIMEOUT_MAX_MS     60000   /**< absolute Obergrenze jeder Fahrt (wie hwtest) */
/** 0 = immer nur ein Motor gleichzeitig (Akku/2-A-Grenze, Brownout-Schutz). */
#define MOTOR_ALLOW_PARALLEL     0

/* =========================================================================
 * Motor 1 – Heben (JGA25-370), GPIO10/11, Encoder GPIO13/14
 * NOCH NICHT ANGESCHLOSSEN/GETESTET: alle Werte sind Annahmen (analog Motor 2)
 * und müssen im Test ermittelt werden.
 * ========================================================================= */
#define M1_DIR_INVERT            0       /**< 1 = Drehrichtung tauschen (IN1/IN2), wenn "positiv" falsch herum läuft */
#define M1_MIN_DUTY_PCT          70      /**< TODO messen: kleinster Duty, bei dem der Motor sicher weiterläuft */
#define M1_KICK_DUTY_PCT         100     /**< Anlauf-Kick */
#define M1_KICK_MS               300     /**< max. Kick-Dauer; zählt der Encoder bis dahin nicht -> BLOCKED */
#define M1_KICK_STEPS            8       /**< so viele Zählschritte beenden den Kick (Motor dreht) */
#define M1_RUN_DUTY_PCT          100     /**< Fahr-Duty, TODO im Test einstellen */
#define M1_RAMP_UP_MS            300     /**< Rampe MIN -> RUN nach dem Kick */
#define M1_RAMP_DOWN_MS          200     /**< Rampe RUN -> MIN vor einem Richtungswechsel */
#define M1_BRAKE_ZONE_STEPS      200     /**< TODO: ab dieser Restdistanz linear auf MIN runter (CPR unbekannt) */
#define M1_STOP_TOL_STEPS        4       /**< Ziel gilt als erreicht, wenn Rest <= Toleranz (oder überfahren) */
#define M1_BLOCK_WINDOW_MS       200     /**< Blockiererkennung: Zeitfenster T, TODO messen */
#define M1_BLOCK_MIN_STEPS       4       /**< Blockiererkennung: weniger als N Schritte in T = blockiert, TODO messen */
#define M1_WRONG_DIR_STEPS       100     /**< so weit gegen die Sollrichtung gezählt -> Richtungsfehler */
#define M1_TIMEOUT_DEFAULT_MS    30000   /**< Timeout, wenn der Aufrufer 0 übergibt */

/* =========================================================================
 * Motor 2 – Wenden (Waveshare N20), GPIO12/22, Encoder GPIO23/24
 * Gemessen am Akku (~4 V, Fast Decay, hwtest): 20–60 % Duty -> steht (0 Impulse),
 * 70 % -> ~24 rpm, 80 % -> ~38 rpm, 100 % -> ~69–73 rpm (Abtriebswelle).
 * Die Spezifikation nahm Anlauf bei 20 % an – das stimmt nicht, daher
 * Anlauf-Kick mit 100 % bis der Encoder zählt, danach Reduktion auf MIN_DUTY
 * und Rampe auf RUN_DUTY.
 * 70 % = 24 rpm = 1680 Schritte/s ≈ 17 Schritte pro 10-ms-Takt.
 * ========================================================================= */
#define M2_DIR_INVERT            0       /**< "vor" (IN1 PWM) zählt positiv – gemessen */
#define M2_MIN_DUTY_PCT          70      /**< gemessen: darunter steht der N20 */
#define M2_KICK_DUTY_PCT         100
#define M2_KICK_MS               300     /**< wie Schonfrist im hwtest-Blockiertest */
#define M2_KICK_STEPS            20
#define M2_RUN_DUTY_PCT          80      /**< ~38 rpm gemessen; Spezifikations-Richtwert */
#define M2_RAMP_UP_MS            300
#define M2_RAMP_DOWN_MS          200
#define M2_BRAKE_ZONE_STEPS      420     /**< 0,1 Abtriebsumdrehung, TODO am Getriebe einstellen */
#define M2_STOP_TOL_STEPS        10
#define M2_BLOCK_WINDOW_MS       200     /**< hwtest: 200 ms ohne Impuls = blockiert */
#define M2_BLOCK_MIN_STEPS       20      /**< bei MIN_DUTY kommen ~336 Schritte in 200 ms, 20 = sicher blockiert */
#define M2_WRONG_DIR_STEPS       200
#define M2_TIMEOUT_DEFAULT_MS    20000

/* =========================================================================
 * Encoder (PCNT)
 * ========================================================================= */
#define ENC_PCNT_LIMIT           32000   /**< Grenzen des 16-Bit-Zählers, Watchpoints für accum_count */
#define ENC_GLITCH_NS            3000    /**< Glitch-Filter 3 µs (hwtest erprobt) */
/**
 * Impulse pro Abtriebsumdrehung bei x1 (eine Flanke eines Kanals);
 * gezählt wird x4, also Schritte/Umdrehung = PPR_X1 * 4.
 * TODO: ENC1 (JGA25-370) unbekannt -> 0. Ermitteln mit hwtest "enc 1 hand".
 */
#define ENC1_PPR_X1              0
#define ENC2_PPR_X1              1050    /**< N20: 1050 x1 = 4200 Schritte/Umdrehung */
#define ENC1_INVERT              0       /**< 1 = Zählrichtung umdrehen */
#define ENC2_INVERT              0       /**< "vor" zählt positiv – gemessen */

/* =========================================================================
 * Achsen / Endpunkte
 * Sicherheitsabstand zum Anschlag in Promille des Fahrwegs (Spez.: 1–2 %).
 * 0 % wird auf end_top + Rand, 100 % auf end_bottom - Rand abgebildet.
 * ========================================================================= */
#define LIFT_MARGIN_TOP_PERMILLE     15  /**< oben fester Anschlag (Kopfschiene) */
#define LIFT_MARGIN_BOTTOM_PERMILLE  0   /**< unten manuell gesetzt, kein Anschlag */
#define TILT_MARGIN_TOP_PERMILLE     15  /**< TODO: hängt davon ab, ob das Wendegetriebe Anschläge hat */
#define TILT_MARGIN_BOTTOM_PERMILLE  15

/* =========================================================================
 * Akku / NTC (battery)
 * ========================================================================= */
#define BATT_DIVIDER_NUM         2       /**< Teiler 100k/100k: U_Akku = U_ADC * 2 / 1 */
#define BATT_DIVIDER_DEN         1
#define BATT_ADC_SAMPLES         16      /**< Mittelung (beim Laden schwankt der Wert 4,19–4,35 V) */
#define BATT_LOCKOUT_MV          3300    /**< Unterspannungssperre: darunter keine Fahrten */
#define BATT_LOCKOUT_RELEASE_MV  3600    /**< Hysterese: Sperre fällt erst ab hier (und nicht beim Laden) */
/**
 * Prozent-Kennlinie LiPo im Ruhezustand {mV, %}, absteigend sortiert,
 * dazwischen linear interpoliert (Spezifikation).
 */
#define BATT_CURVE_POINTS { {4200, 100}, {3800, 60}, {3700, 40}, {3500, 10}, {3300, 0} }

#define NTC_R_FIXED_OHM          10000.0f  /**< 10k gegen 3V3 */
#define NTC_R25_OHM              10000.0f  /**< NTC 10k bei 25 °C gegen GND */
#define NTC_BETA                 3380.0f
#define NTC_VREF_MV              3300      /**< Annahme 3V3 = 3,30 V (nicht gemessen) */
#define NTC_OPEN_MV              3000      /**< ADC 12 dB sättigt bei ~3,1 V -> darüber "offen" */
#define NTC_SHORT_MV             50        /**< darunter "Kurzschluss" */

/* =========================================================================
 * Fenstersensor (lis2dh12, window_sensor)
 *
 * SENSOR: nicht bestückt, deaktiviert – zum Aktivieren einkommentieren.
 * Zentraler Schalter: nur diese eine Zeile einkommentieren. main/CMakeLists.txt
 * liest die Zeile ebenfalls und zieht dann die Komponenten window_sensor und
 * lis2dh12 (I2C) in den Build. Solange auskommentiert, wird weder I2C noch der
 * Sensor angefasst. (Bewusst ohne Präfix CONFIG_, das ist für Kconfig reserviert.)
 * ========================================================================= */
// #define ROLLO_WINDOW_SENSOR      1

#define LIS2DH12_I2C_HZ          100000
#define LIS2DH12_I2C_TIMEOUT_MS  20      /**< kurz, Sensor ist auf Rev 0.1 nicht bestückt */
#define WIN_TILT_ON_DEG          3.0f    /**< gekippt ab Winkel zur Referenz > 3° */
#define WIN_TILT_OFF_DEG         2.0f    /**< zurück "zu" unter 2° (Hysterese) */
#define WIN_DEBOUNCE_COUNT       3       /**< so viele Messungen in Folge zum Umschalten */
#define WIN_MOTION_HOLDOFF_MS    1000    /**< während und 1 s nach einer Fahrt nicht auswerten */
#define WIN_REF_SAMPLES          8       /**< Mittelung beim Einlernen der Referenzlage */
#define WIN_MIN_MAG_MG           500     /**< Betrag < 0,5 g gilt als ungültige Messung */

/* =========================================================================
 * Bedienung (ui)
 * ========================================================================= */
#define BTN_DEBOUNCE_MS          30
#define BTN_LONG_MS              3000    /**< 3 s: Kalibrierung / untere Endlage speichern */
#define BTN_VERY_LONG_MS         10000   /**< 10 s: Werkseinstellung */
#define BTN_POLL_MS              10      /**< Abtastung nur solange die Taste gedrückt/unentprellt ist */
/* Ablauf: kurz (< 3 s) und lang (3…10 s) werden beim LOSLASSEN gemeldet,
 * 10 s sofort beim Erreichen (Loslassen danach wird ignoriert). Bei 3 s kommt
 * zusätzlich ein Hinweis-Ereignis, damit die Applikation Rückmeldung geben kann. */
#define UI_TASK_PRIO             4
#define UI_TASK_STACK            3072

#define LED_TICK_MS              50      /**< Auflösung der Blinkmuster */
#define LED_FAST_ON_MS           100     /**< Pairing: schnelles Blinken */
#define LED_FAST_OFF_MS          100
#define LED_SLOW_ON_MS           500     /**< Kalibrierung: langsames Blinken */
#define LED_SLOW_OFF_MS          500
#define LED_ERR_FLASH_MS         100     /**< Fehler: Doppelblitz an-aus-an, dann Pause */
#define LED_ERR_GAP_MS           150
#define LED_ERR_PAUSE_MS         1000

/* =========================================================================
 * Messintervalle (für die sensor_task in Teil 2)
 * ========================================================================= */
#define SENSOR_WINDOW_PERIOD_MS  1000
#define SENSOR_BATTERY_PERIOD_MS (10 * 60 * 1000)

/* =========================================================================
 * Speicher (storage, NVS)
 * ========================================================================= */
#define STORAGE_NVS_NAMESPACE    "rollo"   /**< eigener Namespace in der Partition "nvs" (max. 15 Zeichen) */
