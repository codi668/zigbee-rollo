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

/* =========================================================================
 * Teil 2: Applikation (main/app_state, main/app_logic)
 * ========================================================================= */
#define APP_QUEUE_LEN            16      /**< Ereignis-Queue app_queue (Befehle, Tasten, Akku, Netz) */
/** Eigene Queue nur für Motorereignisse: Abschluss-Ereignisse (je Auftrag eins) plus
 *  höchstens eine zusammengefasste PROGRESS-Marke je Achse – kann nicht volllaufen. */
#define APP_MOTOR_QUEUE_LEN      8
#define APP_TASK_PRIO            5
#define APP_TASK_STACK           5120
/** app_task wacht spätestens nach dieser Zeit auf und füttert den Task-Watchdog
 *  (CONFIG_ESP_TASK_WDT_TIMEOUT_S = 5 s). Für die Light-Sleep-Messphase ggf. erhöhen. */
#define APP_WDT_FEED_MS          2000
/** Fahrt gilt als "verloren", wenn die FSM fährt, motor_ctrl aber so lange schon steht
 *  (Ereignis ging z. B. bei voller Queue verloren) -> Ersatz-Ereignis STOPPED. */
#define APP_LOST_EVENT_MS        4000

/* Kalibrierung (Spezifikation "Endpunkte setzen"). Alle Wege in Encoder-Schritten. */
#define CAL_BACKOFF_STEPS_LIFT   50      /**< nach dem Anschlag oben zurückfahren (Seil entlasten). TODO: CPR Motor 1 unbekannt */
#define CAL_BACKOFF_STEPS_TILT   105     /**< 1/40 Umdrehung N20. TODO am Wendegetriebe einstellen */
#define CAL_JOG_STEPS_LIFT       50      /**< Tippbetrieb, TODO (CPR Motor 1 unbekannt) */
#define CAL_JOG_STEPS_TILT       105
#define CAL_MIN_TRAVEL_STEPS_LIFT 200    /**< kürzerer Fahrweg wird beim Speichern abgelehnt (Plausibilität), TODO */
#define CAL_MIN_TRAVEL_STEPS_TILT 200
/** 1 = Wendegetriebe hat unten einen festen Anschlag: Endpunkt unten per Blockieren
 *  (Spezifikation: "im Test prüfen"). 0 = unten manuell wie beim Heben. */
#define TILT_CAL_BOTTOM_BY_BLOCK 0
/** Referenzfahrt oben (bis Blockieren, Position = 0) bei jeder Fahrt auf 0 % /
 *  "Auf" (Spezifikation: "gelegentlich oben neu referenzieren"). */
#define LIFT_REHOME_ON_OPEN      1
#define TILT_REHOME_ON_OPEN      0
/** Plausibilität der Referenzfahrt: blockiert die Achse weiter als
 *  max(HOME_TOL_MIN_STEPS, Fahrweg * HOME_TOL_PERMILLE / 1000) unterhalb von 0,
 *  ist das ein Hindernis und kein Anschlag -> FEHLER. */
#define HOME_TOL_PERMILLE        50
#define HOME_TOL_MIN_STEPS       50

/* sensor_task (Akku/NTC). Periode: SENSOR_BATTERY_PERIOD_MS oben. */
#define SENSOR_TASK_PRIO         3
#define SENSOR_TASK_STACK        3072
#define BATT_SETTLE_MS           2000    /**< nach einer Fahrt so lange warten (Spannung erholt sich), Annahme */
#define SENSOR_RETRY_MS          5000    /**< Motor lief gerade -> später erneut versuchen */

/* =========================================================================
 * Teil 2: Zigbee (main/zb_device)
 * ========================================================================= */
#define ROLLO_ZB_ENDPOINT        1
/** Basic-Cluster. "THOMAS" laut Spezifikation; falls Z2M damit Probleme macht,
 *  auf "ESPRESSIF" umstellen (wie im Join-Test). Max. 32 Zeichen. */
#define ROLLO_ZB_MANUFACTURER    "THOMAS"
#define ROLLO_ZB_MODEL           "RolloH2"
/** Poll-Intervall des End Device beim Parent (ms). Das SDK setzt zed_cfg.keep_alive
 *  zugleich als Keep-Alive und als Long-Poll-Intervall (geprüft in esp-zigbee-lib 1.6.8).
 *  Befehle kommen frühestens beim nächsten Poll an. Startwert laut Spezifikation 1–5 s. */
#define ZB_POLL_INTERVAL_MS      2000
#define ZB_CHANNEL_MASK          0x07FFF800u   /**< alle Kanäle 11–26 (Z2M aktuell Kanal 18) */
#define ZB_TASK_PRIO             5
#define ZB_TASK_STACK            8192
#define ZB_LOCK_TIMEOUT_MS       1500    /**< esp_zb_lock_acquire() außerhalb der zb_task, nie portMAX_DELAY */
#define ZB_REPORT_PROGRESS_MIN_MS 1000   /**< Position während der Fahrt höchstens 1/s melden */
#define ZB_BATT_REPORT_S         10800   /**< Akku spätestens alle 3 h aktiv melden */
#define ZB_BATT_REPORT_MIN_S     3600    /**< Reporting-Konfiguration: frühestens stündlich */
#define ZB_NET_CHECK_MS          (5 * 60 * 1000)  /**< Netzüberwachung: offline -> Rejoin anstoßen */
#define ZB_FACTORY_LEAVE_WAIT_MS 3000    /**< Werkseinstellung: so lange auf das Leave warten */

/** ConfigStatus Bit 0 "Operational": 1 = immer gesetzt (Standard), 0 = echter Zustand
 *  (nur BEREIT/FAHRT mit beiden Achsen kalibriert). Ob ZBOSS Window-Covering-Befehle bei
 *  Operational = 0 abweist, ist nicht dokumentiert (Bibliothek nur als Binary) – mit 1 ist
 *  sicher, dass Stop (= Fehler quittieren) und Auf/Ab im unkalibrierten Zustand ankommen.
 *  TODO: mit Z2M testen, dann ggf. auf 0. */
#define ZB_CFG_OPERATIONAL_ALWAYS 1

/** Temperature-Measurement-Cluster (NTC) an Endpoint 1. Zum Abschalten auskommentieren. */
#define ROLLO_ZB_TEMPERATURE     1

/** Debug-Konsole über USB-Serial/JTAG (status, up, down, goto, cal, ...).
 *  Für die Inbetriebnahme an; zum Abschalten auskommentieren. */
#define ROLLO_DEBUG_CONSOLE      1

/**
 * Light Sleep + Sleepy End Device (Messphase, 2. Semester). STANDARD AUS.
 * Zum Einschalten: diese Zeile einkommentieren UND in sdkconfig.defaults den
 * Block "Light Sleep" einkommentieren, dann sdkconfig löschen und neu bauen.
 * Ungetestet; Hinweise und offene Punkte im README.
 */
// #define ROLLO_LIGHT_SLEEP        1
