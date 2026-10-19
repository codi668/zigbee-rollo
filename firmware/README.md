# Zigbee-Rollosteuerung – Firmware

![Version](https://img.shields.io/badge/version-0.1.0-blue) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5.4-red) ![Target](https://img.shields.io/badge/target-ESP32--H2-green)

Firmware für die eigene Platine Rev 0.1 (ESP32-H2FH4S, DRV8833, 2 Encoder-Motoren, LiPo).
Vorgabe ist `software-uebersicht-rollosteuerung.md` im Projektordner (Schule). Abweichungen davon stehen unten unter „Abweichungen von der Spezifikation“.

**Stand Teil 1:** Treiber und Services sind fertig, dazu ein Platzhalter-`main/`, das alle Module initialisiert und einen Selbsttest ausgibt. Teil 2 bringt `app_state` (Zustandsautomat), `zb_device` (Zigbee) und das echte `main.c`.

## Architektur

```text
Zigbee (Teil 2)   zb_device  ── nur Ereignisse in die app_queue ──┐
Applikation       app_state (Teil 2): Zustandsautomat, % ↔ Schritte, Akku-Sperre
Services          motor_ctrl · battery · storage · ui · (window_sensor, deaktiviert)
Treiber           drv8833 · encoder · (lis2dh12, deaktiviert) · ADC/GPIO in battery/ui
Header            board (board_pins.h) · config (config.h)
```

- Jede Schicht ruft nur die darunterliegende auf. Nach oben gehen Ergebnisse als **Callbacks**, die im Kontext der jeweiligen Task laufen (`motor_task`, `ui_task`). Die Callbacks dürfen nicht blockieren und sollen nur ein Ereignis in die `app_queue` legen.
- Kein Modul hängt von Zigbee ab, und es gibt keine globalen Variablen über Modulgrenzen hinweg (jeder Zustand ist `static`, Zugriff nur über Funktionen).
- Die reine Logik steht in eigenen Dateien ohne ESP-IDF und wird auf dem PC getestet: `motion.c`, `axis.c`, `block_detect.c`, `battery_calc.c`, `btn_logic.c`, `led_pattern.c`, `lis2dh12_conv.c`, `tilt_logic.c`.
- Alle Pins stehen in `components/board/include/board_pins.h`, alle Einstellwerte in `components/config/include/config.h`. `config.h` ist eine eigene Komponente statt `main/config.h`, weil die Komponenten sie brauchen und eine Komponente nicht von `main` abhängen kann.

```text
firmware/
├─ CMakeLists.txt, sdkconfig.defaults, partitions.csv, version.txt
├─ main/                 Platzhalter: Init + Selbsttest (Teil 2: app_state, zb_device)
├─ components/
│  ├─ board/             board_pins.h
│  ├─ config/            config.h (inkl. Schalter ROLLO_WINDOW_SENSOR)
│  ├─ drv8833/           LEDC-PWM 20 kHz/10 Bit, nSLEEP, nFAULT-Interrupt
│  ├─ encoder/           PCNT x4, Glitch-Filter, 32-Bit-Position
│  ├─ motor_ctrl/        motor_task, motion (Fahrprofil), axis, block_detect
│  ├─ battery/           ADC Akku + NTC, CHRG_N, Kennlinie, Unterspannungssperre
│  ├─ storage/           NVS-Wrapper
│  ├─ ui/                BOOT-Taste, LED-Muster
│  ├─ lis2dh12/          I2C-Treiber (deaktiviert, nicht bestückt)
│  └─ window_sensor/     Kipp-Erkennung (deaktiviert)
└─ test_host/            Unity-Tests der reinen Logik (gcc, PC)
```

## Schnittstellen (für Teil 2)

Positionskonvention überall: Encoder-Zählschritte (Quadratur x4), **Endpunkt oben = 0**, positive Schritte führen nach unten. 0 % = oben = offen, 100 % = unten = zu (wie Zigbee Lift).

### motor_ctrl (`motor_ctrl.h`, Basis `axis.h`)

| Funktion | Zweck |
|---|---|
| `motor_ctrl_init(cb, ctx)` | DRV8833 + Encoder + `motor_task` starten; Motoren aus, beide Achsen unkalibriert, Position 0 |
| `motor_ctrl_goto(id, steps, timeout_ms)` | Zielfahrt; kalibriert: Ziel auf Endpunkte begrenzt |
| `motor_ctrl_goto_percent(id, pct, timeout_ms)` | Prozentfahrt mit Sicherheitsrand; `ESP_ERR_INVALID_STATE`, wenn unkalibriert |
| `motor_ctrl_move_by(id, delta, timeout_ms)` | Tippbetrieb (relativ) |
| `motor_ctrl_run(id, MOTOR_DIR_UP/DOWN, timeout_ms)` | Fahrt ohne Ziel bis Blockieren/Stopp/Timeout (unkalibriert, Kalibrierung); kalibriert bis Endpunkt |
| `motor_ctrl_stop(id)` / `motor_ctrl_stop_all()` | Stopp mit Vorrang (Kurzschlussbremse 50 ms) |
| `motor_ctrl_is_busy(id)` / `motor_ctrl_any_busy()` | vom Befehl bis zum Abschluss-Ereignis |
| `motor_ctrl_get_position(id)` / `motor_ctrl_set_position(id, pos)` | Live-Position / Referenzieren (nur im Stillstand) |
| `motor_ctrl_set_endpoints(id, top, bottom)` / `motor_ctrl_clear_calibration(id)` | Kalibrierung setzen/löschen (nur im Stillstand) |
| `motor_ctrl_fault_latched()` / `motor_ctrl_clear_fault()` | Sperre nach nFAULT/Notaus: alle Fahrbefehle liefern `ESP_ERR_INVALID_STATE`, bis die Applikation quittiert (FEHLER → BEREIT) |
| `motor_ctrl_get_axis(id, &axis)` | Kopie `axis_t` (pos, end_top, end_bottom, calibrated, Ränder) |
| `axis_steps_to_percent(&axis, pos)` / `axis_percent_to_steps(&axis, pct)` | Umrechnung für Zigbee-Attribute |

IDs: `MOTOR_LIFT` (Motor 1, JGA25, Encoder 1), `MOTOR_TILT` (Motor 2, N20, Encoder 2).
Befehle legen nur etwas in eine Queue und kehren sofort zurück (aus jeder Task, nicht aus ISR). Mit `MOTOR_ALLOW_PARALLEL = 0` liefert ein Befehl `ESP_ERR_INVALID_STATE`, solange die andere Achse fährt.

Ereignisse `motor_event_t {type, id, pos, goal, elapsed_ms}`: genau ein Abschluss-Ereignis pro „busy“-Phase. Wird ein Auftrag ersetzt, meldet nur der letzte.

| Ereignis | Bedeutung |
|---|---|
| `MOTOR_EVT_DONE` | Ziel erreicht |
| `MOTOR_EVT_LIMIT` | `run()` am Endpunkt angehalten (kalibriert) |
| `MOTOR_EVT_STOPPED` | durch Stopp angehalten |
| `MOTOR_EVT_BLOCKED` | blockiert oder läuft nicht an; bei der Kalibrierung = Anschlag oben |
| `MOTOR_EVT_TIMEOUT` | maximale Fahrzeit überschritten |
| `MOTOR_EVT_WRONG_DIR` | Encoder zählt gegen die Sollrichtung (`Mx_DIR_INVERT`/`ENCx_INVERT` prüfen) |
| `MOTOR_EVT_FAULT` | nFAULT des DRV8833 oder Notaus (Stopp nicht zustellbar). Setzt die Fehlersperre, wartende Befehle werden verworfen |
| `MOTOR_EVT_REJECTED` | verworfen, weil die andere Achse fährt |
| `MOTOR_EVT_PROGRESS` | Zwischenstand ca. jede Sekunde (kein Abschluss), für das Zigbee-Reporting |

Fahrprofil (`motion.c`): Kick mit `KICK_DUTY`, bis `KICK_STEPS` gezählt sind (zählt nichts innerhalb von `KICK_MS` → BLOCKED), dann Rampe `MIN_DUTY` → `RUN_DUTY` in `RAMP_UP_MS`, danach Fahrt mit scharfer Blockiererkennung. Ab `BRAKE_ZONE_STEPS` vor dem Ziel geht der Duty linear bis `MIN_DUTY` herunter, am Ziel folgt die Kurzschlussbremse. Bei einem Richtungswechsel wird erst heruntergerampt, dann gebremst, `MOTOR_DIR_PAUSE_MS` gewartet und neu angefahren.
Während einer Fahrt hält `motor_ctrl` einen PM-Lock (kein Light Sleep) und ist beim Task-Watchdog angemeldet. Die Unterspannungssperre prüft **die Applikation** vor jedem Fahrbefehl (`battery_lockout_active()`).

### battery (`battery.h`, `battery_calc.h`)

| Funktion | Zweck |
|---|---|
| `battery_init()` | ADC1 (GPIO1 Akku, GPIO4 NTC) mit Kalibrierung je Kanal, CHRG_N mit Pull-up |
| `battery_measure(&st)` | 16× gemittelt, ca. 35 ms blockierend; nur im Stillstand aufrufen (`!motor_ctrl_any_busy()`) |
| `battery_is_charging()` | CHRG_N LOW = lädt |
| `battery_lockout_active()` | Sperre der letzten Messung; vor der ersten Messung false, also beim Start einmal messen |

`battery_status_t`: `vbat_mv`, `percent`, `percent_x2` (Zigbee `BatteryPercentageRemaining`), `charging`, `lockout`, `ntc` (OK/OPEN/SHORT), `temp_c`. Zigbee `BatteryVoltage` = `vbat_mv / 100`.
Sperre: setzt unter 3300 mV und löst erst ab 3600 mV, und auch das nur, wenn **nicht geladen** wird (beim Laden ist die Spannung überhöht).

### storage (`storage.h`)

| Funktion | Zweck |
|---|---|
| `storage_init()` | `nvs_flash_init` (bei voller/alter Partition löschen + neu), alles in die RAM-Kopie laden |
| `storage_load(&d)` | `storage_data_t {max[2], pos[2], moving, has_win_ref, win_ref}` |
| `storage_is_calibrated(axis)` | `max != 0` |
| `storage_save_calibration(axis, max)` / `storage_clear_calibration(axis)` | Endpunkt unten (oben ist immer 0) |
| `storage_motion_begin()` | `moving = 1`, **vor** jeder Fahrt |
| `storage_motion_end(axis, pos)` | Position + `moving = 0`, am Ende der Fahrt (ein Commit) |
| `storage_save_position(axis, pos)` | nur Position (z. B. nach dem Referenzieren) |
| `storage_save_win_ref(&v)` | Referenzlage Fenster |
| `storage_erase_all()` | Werkseinstellung: Namespace `rollo` löschen (Zigbee-Netz liegt in `zb_storage`, das löscht Teil 2 mit `esp_zb_factory_reset()`) |

NVS-Schlüssel: `lift_max`, `tilt_max`, `lift_pos`, `tilt_pos`, `moving`, `win_ref_x/_y/_z`. Unveränderte Werte werden nicht neu geschrieben, und während der Fahrt wird nie zyklisch geschrieben. Steht nach einem Neustart `moving = 1`, ist die Position unsicher und muss neu referenziert werden.
Die Funktionen blockieren für einen Commit, deshalb nicht aus Zigbee-Callbacks oder einer ISR aufrufen (aus `app_task` ist es in Ordnung).

### ui (`ui.h`, `btn_logic.h`, `led_pattern.h`)

| Funktion / Ereignis | Zweck |
|---|---|
| `ui_init(cb, ctx)` | LED (GPIO8, aus) und Taste (GPIO9, Interrupt auf beide Flanken) einrichten, `ui_task` starten |
| `ui_set_led(LED_PAT_OFF / PAIRING / CALIBRATION / ERROR / ON)` | Normal aus · Pairing schnell (100/100 ms) · Kalibrierung langsam (500/500 ms) · Fehler Doppelblitz · dauernd an |
| `ui_get_led()`, `ui_button_pressed()` | Abfrage |
| `UI_EVT_BTN_SHORT` | beim Loslassen nach < 3 s |
| `UI_EVT_BTN_LONG_HINT` | 3 s erreicht, Taste noch gedrückt (nur als Rückmeldung) |
| `UI_EVT_BTN_LONG` | beim Loslassen nach 3 bis < 10 s |
| `UI_EVT_BTN_VERY_LONG` | sofort bei 10 s; das Loslassen danach erzeugt nichts mehr |

Entprellung 30 ms. Ist die Taste beim Start schon gedrückt, wird dieser Druck ignoriert. Die Bedeutung der Ereignisse (Stopp/Auf/Ab, Kalibrierung, Werkseinstellung) legt `app_state` fest.

### drv8833 / encoder (nur über motor_ctrl verwenden)

`drv8833_init/enable/drive/brake/coast/all_off`, `drv8833_fault_active()`. Der nFAULT-Interrupt schaltet in der ISR sofort nSLEEP LOW. `encoder_get/set/counts_per_rev()`, wobei `counts_per_rev` = 0 bedeutet, dass der Wert unbekannt ist (Encoder 1).

### window_sensor / lis2dh12 (deaktiviert)

`window_sensor_init/available/ready/set_reference/calibrate/poll/get_state`, Zustände `WINDOW_UNKNOWN/CLOSED/TILTED`. Nur mit `ROLLO_WINDOW_SENSOR` verfügbar, siehe nächsten Abschnitt.

## Sensor deaktiviert

Der LIS2DH12 ist auf Rev 0.1 **nicht bestückt**. Deshalb ist der Fenstersensor in dieser Version abgeschaltet: Die Komponenten `lis2dh12` und `window_sensor` werden nicht gebaut, I2C wird nie initialisiert, und es gibt weder Timeouts noch Logs vom Sensor. Der Code bleibt erhalten, die reine Logik (`lis2dh12_conv`, `tilt_logic`) läuft in den Host-Tests weiter mit.

Wieder einschalten:

1. In `components/config/include/config.h` die Zeile
   `// #define ROLLO_WINDOW_SENSOR      1` einkommentieren.
2. Neu bauen (`idf.py build`). `main/CMakeLists.txt` liest denselben Schalter, nimmt dann `window_sensor` und `lis2dh12` in `REQUIRES` auf und gibt beim Konfigurieren `Rollo: Fenstersensor AKTIV` aus. In `main.c` werden die `#ifdef ROLLO_WINDOW_SENSOR`-Blöcke aktiv.
3. Teil 2: in der `sensor_task` `window_sensor_poll(motor_ctrl_any_busy(), &changed)` jede Sekunde aufrufen, die Referenz mit `storage_save_win_ref()` speichern und nach dem Start mit `window_sensor_set_reference()` laden.

Alle Stellen sind mit `SENSOR: nicht bestückt, deaktiviert – zum Aktivieren einkommentieren` markiert (`grep -rn "SENSOR:"`).

## Konstanten (`config.h`)

| Gruppe | Wichtigste Werte |
|---|---|
| PWM | `MOTOR_PWM_FREQ_HZ` 20 kHz, 10 Bit, `MOTOR_SLOW_DECAY` 0 (Fast Decay, wie gemessen) |
| motor_task | `MOTOR_TICK_MS` 10, `MOTOR_BRAKE_MS` 50, `MOTOR_DIR_PAUSE_MS` 100, `MOTOR_TIMEOUT_MAX_MS` 60000, `MOTOR_ALLOW_PARALLEL` 0 |
| je Motor `M1_*`/`M2_*` | `DIR_INVERT`, `MIN_DUTY_PCT`, `KICK_DUTY_PCT`/`KICK_MS`/`KICK_STEPS`, `RUN_DUTY_PCT`, `RAMP_UP_MS`/`RAMP_DOWN_MS`, `BRAKE_ZONE_STEPS`, `STOP_TOL_STEPS`, `BLOCK_WINDOW_MS`/`BLOCK_MIN_STEPS`, `WRONG_DIR_STEPS`, `TIMEOUT_DEFAULT_MS` |
| Encoder | `ENC_GLITCH_NS` 3000, `ENC1_PPR_X1` 0 (**TODO**), `ENC2_PPR_X1` 1050, `ENCx_INVERT` |
| Achsen | `LIFT_/TILT_MARGIN_TOP/BOTTOM_PERMILLE` (Sicherheitsabstand 1,5 %) |
| Akku | Teiler 2:1, 16 Mittelungen, Sperre 3300/3600 mV, Kennlinie `BATT_CURVE_POINTS`, NTC 10k/B 3380, offen > 3000 mV, Kurzschluss < 50 mV |
| Fenster | `ROLLO_WINDOW_SENSOR` (aus), 3°/2° Hysterese, 3 Messungen, 1 s Sperre nach einer Fahrt |
| ui | Entprellung 30 ms, 3 s / 10 s, LED-Zeiten |
| Messintervalle | Fenster 1 s, Akku 10 min |
| storage | Namespace `rollo` |

Motor 2 (N20) ist gemessen (am Akku, Fast Decay): unter 70 % Duty steht er; 70 % ≈ 24 rpm, 80 % ≈ 38 rpm, 100 % ≈ 69–73 rpm. Bei Motor 1 sind alle Werte Annahmen (noch nicht angeschlossen).

## Bauen

```bash
export IDF_PATH=~/.platformio/packages/framework-espidf IDF_TOOLS_PATH=~/.espressif
. $IDF_PATH/export.sh
cd firmware
idf.py set-target esp32h2      # nur beim ersten Mal
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Der Platzhalter bewegt **keinen Motor**. Er gibt Folgendes aus: Version, Reset-Grund, NVS-Inhalt (Warnung bei `moving = 1`), Zustand von nSLEEP/nFAULT, Akku/NTC/Laden, `[ OK ]`/`[FAIL]` je Modul und danach Encoder-Positionen bei jeder Änderung (Welle von Hand drehen). Ein kurzer Tastendruck schaltet die LED-Muster weiter, 3 s und 10 s werden nur geloggt.

## Testen (PC)

```bash
make -C firmware/test_host test
```

Das kompiliert die reinen Logikdateien mit gcc (`-Wall -Wextra -Werror`) zusammen mit Unity aus `$IDF_PATH/components/unity` und führt 56 Tests aus: Akku-Kennlinie/NTC/Sperre, Achse (Prozent ↔ Schritte, Ränder, Begrenzung), Blockiererkennung, Fahrprofil gegen ein Motormodell (Kick, Rampe, Bremszone, Blockieren, Timeout, Richtungsfehler, Endpunkte, Richtungswechsel, Stopp), LIS2DH12-Umrechnung, Kipp-Logik, Taste und LED-Muster.

## Abweichungen von der Spezifikation

- **Anlauf:** Die Spezifikation nimmt Anlauf bei 20 % an, gemessen sind 70 % (N20). Deshalb gibt es einen Anlauf-Kick (100 %, bis der Encoder zählt), danach geht es auf `MIN_DUTY` zurück und von dort per Rampe auf `RUN_DUTY`. Alle Werte sind pro Motor einstellbar.
- **Decay:** Standard ist Fast Decay, weil alle Messungen damit gemacht wurden. Slow Decay (Spezifikation) lässt sich mit `MOTOR_SLOW_DECAY 1` einschalten, ist aber **ungetestet**.
- **Sicherheitsrand:** 0…100 % wird auf `[end_top + Rand, end_bottom − Rand]` abgebildet (Spezifikation: „1–2 % Abstand“). Prozent ist also nicht exakt `pos / end_bottom`.
- **Fenstersensor** ist komplett deaktiviert (nicht bestückt). Falls er wieder aktiviert wird: Der LIS2DH12 läuft im Normal Mode (10 Bit) mit 1 Hz statt im Low-Power-Modus.
- **config.h** liegt in `components/config/` statt in `main/` (Begründung siehe oben).
- **Zusätzliche Ereignisse:** `WRONG_DIR`, `REJECTED`, `PROGRESS` und `UI_EVT_BTN_LONG_HINT`.
- **nFAULT-Sperre** in motor_ctrl selbst (zusätzlich zum Zustand FEHLER in app_state): nach nFAULT fährt nichts mehr, bis `motor_ctrl_clear_fault()` aufgerufen wird.

## Offene Punkte / TODO

- `ENC1_PPR_X1` (JGA25) messen: `hwtest` → `enc 1 hand`.
- Motor 1 anschließen und alle `M1_*`-Werte messen: Anlauf, Blockier-Schwelle N/T, Bremszone, Drehrichtung.
- Für Motor 2 `BLOCK_*`, `BRAKE_ZONE` und die Ränder am echten Wendegetriebe einstellen. Offen ist auch noch, ob es einen Anschlag hat.
- Slow Decay testen.
- `NTC_VREF_MV` = 3300 ist angenommen, nicht gemessen.
- Light Sleep (Teil 2): Die Taste weckt nur über `gpio_wakeup_enable` (Pegel). Flanken-Interrupts wecken nicht aus dem Light Sleep.
- Die Unterspannungssperre löst während des Ladens nie. Falls gewünscht, in `batt_calc_lockout()` ändern.

## Version

Die Version steht in `version.txt` (SemVer) und landet im App-Descriptor und im Startbanner.
