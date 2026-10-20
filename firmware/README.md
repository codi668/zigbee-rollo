# Zigbee-Rollosteuerung – Firmware

![Version](https://img.shields.io/badge/version-0.2.9-blue) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5.4-red) ![Target](https://img.shields.io/badge/target-ESP32--H2-green)

Firmware für die eigene Platine Rev 0.1 (ESP32-H2FH4S, DRV8833, 2 Encoder-Motoren, LiPo).
Vorgabe ist `software-uebersicht-rollosteuerung.md` im Projektordner (Schule). Abweichungen davon stehen unten unter „Abweichungen von der Spezifikation“.

**Stand 0.2.9:** Beim Anstecken des USB-Kabels gibt die Firmware sofort die Version aus (`=== Zigbee-Rollosteuerung <Version> … ===`), dazu der Konsolenbefehl `version`; der Monitor fragt sie beim Verbinden ab.

**Stand 0.2.8:** Keine Referenzfahrten mehr für Achsen ohne festen Anschlag oben (`TILT_TOP_STOP 0`): Nach einem Fehler (z. B. blockiert) und dem Quittieren fuhr der Motor bei der nächsten Prozentfahrt in der Referenzfahrt bis zum 60-s-Zeitlimit über das Limit hinaus. Außerdem: Endabstand schaltbar (`margin on|off`, 0.2.7), Messspur und neue Monitor-Reiter (0.2.6).

**Stand 0.2.2** (getestet mit ZHA: Kipp-Regler fährt Motor 2); 0.2.1 brachte: Aufbaustufe nur Motor 2 (`ROLLO_TILT_ONLY 1` in `config.h`: Taste, Konsole (ohne Achsangabe) und der Zigbee-Kipp-Regler fahren die Wendeachse; Befehle für „Heben“ (Auf/Zu, Lift-Prozent) werden abgewiesen, Motor 1 wird nie angesteuert), manuelle Endpunkte (`limit …`), unkalibriert meldet Zigbee Position 0 statt 0xFF (ZHA zeigte −155 %), Schein-nFAULT beim ersten Anlauf nach dem Boot behoben (kurzer nFAULT-Puls direkt nach dem Einschalten; `DRV8833_FAULT_BLANK_MS` = 50 ms Blanking im Interrupt und in der Pegelabfrage, auf der Hardware bestätigt). Für beide Motoren `ROLLO_TILT_ONLY 0`.

**Stand 0.2.0 (Teil 2):** Treiber und Services (Teil 1) plus Applikation (`app_state`/`app_logic`: Zustandsautomat, Kalibrierung, Plausibilität), Zigbee End Device (`zb_device`), `sensor_task` (Akku/NTC) und Debug-Konsole. Gebaut, Host-Tests grün, **noch nicht auf der Hardware getestet** – Reihenfolge für den ersten Flash siehe „Inbetriebnahme“.

## Architektur

```text
Zigbee        zb_device (zb_task): Endpoint 1 Window Covering, Callbacks -> nur app_queue
Applikation   app_state (app_task): app_queue, führt app_logic aus (reine Logik, Host-getestet)
              sensor_task: Akku/NTC alle 10 min + nach Fahrten, nur im Stillstand
              console_cmds: Debug-Konsole -> app_queue (gleicher Weg wie Zigbee)
Services      motor_ctrl (motor_task) · battery · storage · ui (ui_task) · (window_sensor, deaktiviert)
Treiber       drv8833 · encoder · (lis2dh12, deaktiviert) · ADC/GPIO in battery/ui
Header        board (board_pins.h) · config (config.h)
```

- Jede Schicht ruft nur die darunterliegende auf. Nach oben gehen Ergebnisse als **Callbacks** (`motor_task`, `ui_task`, `zb_task`), die nur ein Ereignis in die `app_queue` legen und nie blockieren.
- Die ganze Entscheidungslogik steckt in `main/app_logic.c`: Eingang = Ereignis, Ausgang = Liste von Aktionen (Motorbefehl, NVS, Zigbee-Meldung). `app_state.c` führt die Aktionen aus. Dadurch ist der Zustandsautomat samt Kalibrierablauf auf dem PC getestet.
- Kein Modul hängt von Zigbee ab außer `zb_device`, und es gibt keine globalen Variablen über Modulgrenzen hinweg (jeder Zustand ist `static`, Zugriff nur über Funktionen).
- Reine Logik ohne ESP-IDF (Host-Tests): `motion.c`, `axis.c`, `block_detect.c`, `battery_calc.c`, `btn_logic.c`, `led_pattern.c`, `lis2dh12_conv.c`, `tilt_logic.c`, `main/app_logic.c`, `main/zb_logic.c`.
- Alle Pins stehen in `components/board/include/board_pins.h`, alle Einstellwerte in `components/config/include/config.h`. `config.h` ist eine eigene Komponente statt `main/config.h`, weil die Komponenten sie brauchen und eine Komponente nicht von `main` abhängen kann.

```text
firmware/
├─ CMakeLists.txt, sdkconfig.defaults, partitions.csv, version.txt
├─ main/
│  ├─ main.c             Init-Reihenfolge, Tasks starten, Banner
│  ├─ app_state.c/.h     app_queue, app_task, Aktionen ausführen, Status/Log
│  ├─ app_logic.c/.h     Zustandsautomat (reine Logik)
│  ├─ zb_device.c/.h     Zigbee End Device, Datenmodell, Netzlogik
│  ├─ zb_logic.c/.h      Attributwerte, Backoff (reine Logik)
│  ├─ sensor_task.c/.h   Akku/NTC-Messung im Stillstand
│  ├─ console_cmds.c/.h  Debug-Konsole (ROLLO_DEBUG_CONSOLE)
│  └─ idf_component.yml  esp-zigbee-lib 1.6.8, esp-zboss-lib 1.6.4 (fest, wie Join-Test)
├─ components/           board, config, drv8833, encoder, motor_ctrl, battery, storage, ui,
│                        lis2dh12 + window_sensor (deaktiviert)
├─ z2m/rollo_h2.js       Entwurf External Converter (ungetestet)
└─ test_host/            Unity-Tests der reinen Logik (gcc, PC)
```

## Tasks und Queues

| Task | Prio | Stack | Takt | Aufgabe |
|---|---|---|---|---|
| `motor_task` | 6 | 4096 | 10 ms nur während einer Fahrt | Teil 1: Fahrprofil, Blockieren, Timeout, nFAULT, PM-Lock, Task-WDT während der Fahrt |
| `zb_task` | 5 | 8192 | ereignisgesteuert | `esp_zb_stack_main_loop`, Signale, Befehle -> `app_queue` |
| `app_task` | 5 | 5120 | ereignisgesteuert, spätestens alle 2 s | Zustandsautomat, NVS, Attribute; dauerhaft beim Task-WDT (5 s) |
| `ui_task` | 4 | 3072 | nur bei Tastendruck/Blinkmuster | Teil 1: Taste, LED |
| `sensor_task` | 3 | 3072 | 10 min, nach Fahrten (+2 s) | Akku/NTC messen, nur im Stillstand -> `app_queue` |
| Konsole (REPL) | 2 | 4096 | Eingabe | Befehle -> `app_queue` |

- **`app_queue`** (16 Einträge): Befehle (Zigbee/Konsole), Tastenereignisse, Akku-Messungen, Netzzustand, Identify. Das Start-Ereignis (NVS-Inhalt + erste Akku-Messung) wird zuerst eingereiht.
- **Motor-Queue** (`APP_MOTOR_QUEUE_LEN` = 8, eigene Queue, mit `app_queue` in einem Queue-Set): nur Motorereignisse. PROGRESS wird zusammengefasst (Position im Latch, höchstens eine Marke je Achse), Abschluss-Ereignisse kommen je Auftrag genau einmal – die Queue kann nicht volllaufen, BLOCKED/FAULT/DONE gehen nie verloren.
- **Akku nur im Stillstand:** `app_task` nimmt vor jedem Fahrbefehl `sensor_lock()`, die Messung hält denselben Mutex und prüft `motor_ctrl_any_busy()`. So beginnt keine Fahrt mitten in einer Messung.
- **Verlorene Motorereignisse:** Steht `motor_ctrl` länger als `APP_LOST_EVENT_MS` (4 s), während der Zustandsautomat noch „fährt“ (z. B. Queue war voll), wird ein Ersatz-Ereignis „gestoppt“ erzeugt.
- **Zigbee-Lock:** Attribute schreibt nur `zb_device`, außerhalb der `zb_task` immer zwischen `esp_zb_lock_acquire(1500 ms)` und `esp_zb_lock_release()` – nie `portMAX_DELAY`. Bekommt die `app_task` den Lock nicht, wird nur diese Meldung übersprungen (Log-Warnung); der Wert steht in der Schattenkopie und geht beim nächsten Mal mit.

## Zustandsautomat

Zustand, Übergänge und Kalibrierung: `main/app_logic.c` (Tests: `test_host/test_app_logic.c`). Achsen: Heben (Motor 1) und Wenden (Motor 2), immer nur eine fährt (`MOTOR_ALLOW_PARALLEL = 0`).

| Von | Ereignis | Nach | Aktion |
|---|---|---|---|
| NEUSTART | Start, beide Achsen haben Endpunkte im NVS | BEREIT | Endpunkte + Position in `motor_ctrl`; bei `moving = 1` beide Achsen „Referenzfahrt nötig“ |
| NEUSTART | Start, mindestens eine Achse ohne Endpunkte | UNKALIBRIERT | |
| NEUSTART | erste Messung unter 3,3 V | AKKU_LEER | |
| BEREIT/UNKALIBRIERT | Befehl Auf/Ab/Prozent/Tippen | FAHRT | `moving = 1`, Fahrbefehl |
| BEREIT/UNKALIBRIERT | Taste 3 s / Mode-Bit 1 / `cal` | KALIBRIERUNG | |
| FAHRT | Ziel erreicht / Stopp / Endpunkt | BEREIT bzw. UNKALIBRIERT | Position + `moving = 0`, Endwert melden, Akku messen, vorgemerkten Befehl der anderen Achse starten |
| FAHRT | blockiert, Timeout, Richtungsfehler, nFAULT | FEHLER | Motor aus, LED Doppelblitz; danach Referenzfahrt nötig (auch nach nFAULT) |
| FAHRT (Referenz) | Blockieren weit unter 0 bei bekannter Position | FEHLER | Hindernis statt Kopfschiene |
| KALIBRIERUNG | Endpunkte aller gewählten Achsen gespeichert | BEREIT/UNKALIBRIERT | |
| KALIBRIERUNG | Abbruch (Stopp beim Hochfahren, `cal abort`, Mode-Bit 1 löschen vor „unten“) | vorher | alte Endpunkte aus dem NVS zurück, danach Referenzfahrt nötig |
| KALIBRIERUNG | kein Anschlag oben, Blockieren beim Runterfahren, Motor läuft nicht | FEHLER | alte Endpunkte zurück |
| FEHLER | quittieren: Taste kurz / Zigbee Stop / `ack` / `stop` | BEREIT/UNKALIBRIERT/AKKU_LEER | `motor_ctrl_clear_fault()` (scheitert, solange nFAULT anliegt -> bleibt FEHLER) |
| BEREIT/UNKALIBRIERT | Messung unter 3,3 V | AKKU_LEER | |
| FAHRT | Messung unter 3,3 V | AKKU_LEER (nach dem Stopp-Ereignis) | sofort Stopp, Vorgemerktes verworfen |
| KALIBRIERUNG | Messung unter 3,3 V | AKKU_LEER | fährt sie: sofort Stopp, danach Abbruch (kein weiterer Schritt); steht sie: Abbruch |
| AKKU_LEER | Messung: Sperre aufgehoben | BEREIT/UNKALIBRIERT | |
| jeder | Taste 10 s / `reset factory` | Neustart | Motoren aus, NVS `rollo` löschen, Zigbee verlassen |

Regeln:

- **Prozentfahrt** nur für eine kalibrierte Achse. Ist eine Referenzfahrt nötig (nach `moving = 1`, nach Blockieren/Timeout, nach Kalibrierfehler), fährt die Achse zuerst nach oben bis zum Anschlag (Position = 0) und dann aufs Ziel.
- **Heben „Auf“ / 0 %** referenziert jedes Mal oben neu (`LIFT_REHOME_ON_OPEN = 1`, Spezifikation „bei jeder Fahrt auf 0 %“). Wenden nicht (`TILT_REHOME_ON_OPEN = 0`).
- **Unkalibriert:** „Auf“ endet am Anschlag oben (Blockieren = kein Fehler, Position = 0). „Ab“ läuft bis Stopp – **beim Seil gibt es unten keinen Anschlag, also selbst stoppen** (sonst Timeout `M1_TIMEOUT_DEFAULT_MS` = 30 s -> FEHLER). Blockieren nach unten = FEHLER.
- **Zweite Achse während einer Fahrt:** Der Befehl wird vorgemerkt (letzter gewinnt) und nach dem Ende der Fahrt ausgeführt. Stopp verwirft Vorgemerktes.
- **moving-Flag:** `moving = 1` vor jeder Fahrt, Position + `moving = 0` am Ende. Muss danach noch eine Achse referenziert werden, bleibt `moving = 1`.
- **Akku:** Messung beim Start (vor dem ersten Fahrbefehl), alle 10 min und 2 s nach jeder Fahrt, nur im Stillstand.
- **Unterspannungssperre (bitte bestätigen):** setzt unter 3300 mV, löst erst ab 3600 mV **und nur, wenn nicht geladen wird** (Teil-1-Entscheidung in `batt_calc_lockout()`: beim Laden ist die Spannung überhöht). Folge: Während das Ladegerät steckt, bleibt AKKU_LEER bestehen; erst nach Ladeende (CHRG_N HIGH) und der nächsten Messung (spätestens 10 min, oder `batt`) geht es weiter.

### Kalibrierung (Heben, dann Wenden)

1. Start: Taste 3 s halten (LED leuchtet bei 3 s, beim Loslassen langsames Blinken), Zigbee `Mode` = 2 oder Konsole `cal` (`cal lift` / `cal tilt` für eine Achse).
2. Achse fährt hoch bis Blockieren -> Endpunkt oben = 0, dann `CAL_BACKOFF_STEPS_*` zurück (Seil entlasten).
3. Achse fährt runter. An der gewünschten Stelle anhalten: Taste kurz / Zigbee Stop / Konsole `stop`.
4. Tippbetrieb: Taste kurz = `CAL_JOG_STEPS_*` nach unten; Konsole `jog <schritte>` (+ runter, − hoch); Zigbee Auf/Ab = ein Tippschritt.
5. Speichern: Taste 3 s / Zigbee `Mode` = 0 / Konsole `cal save` (auch während der Fahrt nach unten: hält an und speichert). Weniger als `CAL_MIN_TRAVEL_STEPS_*` wird abgelehnt.
6. Danach kommt die nächste Achse (bei `cal` bzw. Taste: Heben, dann Wenden).

Ohne angeschlossenen Motor 1 bricht `cal` (beide Achsen) mit FEHLER ab (Motor läuft beim Zurückfahren nicht an) – für den Test daher `cal tilt` verwenden. Wenden kann mit `TILT_CAL_BOTTOM_BY_BLOCK = 1` auch unten per Blockieren kalibrieren, falls das Getriebe einen Anschlag hat.

## Bedienung

| BOOT-Taste | Ruhe | Fahrt | Kalibrierung | FEHLER |
|---|---|---|---|---|
| kurz | Heben Auf/Ab im Wechsel (erst Auf) | Stopp | runter: anhalten; Tippbetrieb: Tippschritt | quittieren |
| 3 s (beim Loslassen) | Kalibrierung starten | – | Endpunkt unten speichern | – |
| 10 s (sofort) | Werkseinstellung | Werkseinstellung | Werkseinstellung | Werkseinstellung |

| LED (GPIO8) | Bedeutung |
|---|---|
| aus | Normalbetrieb, auch AKKU_LEER (Strom sparen) |
| schnelles Blinken | Pairing (Network Steering läuft) |
| langsames Blinken | Kalibrierung |
| Doppelblitz | FEHLER |
| dauernd an | Taste 3 s erreicht (noch gedrückt), Zigbee Identify, Werkseinstellung läuft |

## Zigbee-Datenmodell

End Device (ZED), natives 802.15.4-Radio, alle Kanäle 11–26 (Z2M: Kanal 18), Poll-Intervall `ZB_POLL_INTERVAL_MS` = 2 s. Das SDK nimmt `zed_cfg.keep_alive` zugleich als Keep-Alive und als Long-Poll-Intervall (im Objektcode der Apache-lizenzierten esp-zigbee-lib 1.6.8 geprüft). Befehle kommen also spätestens nach ~2 s an. RxOnWhenIdle bleibt ohne `ROLLO_LIGHT_SLEEP` auf dem Stack-Standard (wie im Join-Test, wird beim Start geloggt).

**Endpoint 1**, Profil HA, Gerät Window Covering (0x0202), Server-Cluster:

| Cluster | Attribut / Befehl | Wert |
|---|---|---|
| Basic (0x0000) | ManufacturerName / ModelIdentifier | `THOMAS` / `RolloH2` |
| | PowerSource / SWBuildID | Batterie (0x03) / Firmware-Version |
| Identify (0x0003) | Identify | LED dauernd an |
| Power Configuration (0x0001) | BatteryVoltage (0x0020) | 100 mV (37 = 3,7 V), meldbar |
| | BatteryPercentageRemaining (0x0021) | 0,5 % (200 = 100 %), meldbar |
| Window Covering (0x0102) | WindowCoveringType | 0x08 Tilt Blind Lift + Tilt |
| | CurrentPositionLiftPercentage (0x0008) | 0 = ganz offen/oben … 100 = ganz zu/unten, 0xFF = unkalibriert |
| | CurrentPositionTiltPercentage (0x0009) | wie Lift (0 % = Endpunkt „oben“ der Wendeachse) |
| | ConfigStatus (0x0007) | Online, Lift/Tilt Closed Loop + Encoder; Operational siehe `ZB_CFG_OPERATIONAL_ALWAYS` |
| | Mode (0x0017, schreibbar) | Bit 1 = Kalibriermodus (2 schreiben = starten, 0 = speichern/abbrechen); Bit 0 wird nicht unterstützt |
| | UpOpen / DownClose / Stop | Heben auf / ab / Stopp (Stop im FEHLER = quittieren) |
| | GoToLiftPercentage / GoToTiltPercentage | Prozentfahrt |
| | GoToLift/TiltValue | nicht unterstützt (ZCL-Fehler) |
| Temperature Measurement (0x0402) | MeasuredValue | NTC in 0,01 °C, 0x8000 bei offen/Kurzschluss (`ROLLO_ZB_TEMPERATURE`) |

**Endpoint 2** (IAS Zone 0x0402, Fenstersensor): `SENSOR: nicht bestückt, deaktiviert` – nur mit `ROLLO_WINDOW_SENSOR`, dort als Gerüst (Enrollment und Zonenstatus-Meldung TODO).

**Reporting:** Position während der Fahrt höchstens 1/s (Fortschritt aus `motor_ctrl`), Endwert sofort; Akku spätestens alle 3 h, sofort bei Sperrwechsel und nach dem Beitritt. Die Firmware sendet diese Meldungen selbst direkt an den Koordinator (0x0000, Endpoint 1) und legt zusätzlich Standard-Reporting an (Position 1 s … 1 h, Akku 1 h … 3 h), das Z2M beim Konfigurieren überschreiben kann.

**Hersteller-/Modell-ID:** „THOMAS“ wie in der Spezifikation (statt „ESPRESSIF“ im Join-Test), Modell „RolloH2“. Z2M kennt das Gerät nicht; neuere Z2M-Versionen erzeugen dafür automatisch eine Definition aus den Clustern. Falls Z2M damit Probleme macht, in `config.h` `ROLLO_ZB_MANUFACTURER` auf `"ESPRESSIF"` stellen.

**Z2M-Converter:** Entwurf in `z2m/rollo_h2.js` (**ungetestet**, modernExtend `windowCovering` lift+tilt, `battery`, `temperature`). Nur nötig, wenn die automatisch erzeugte Definition nicht reicht.

### Netzlogik (gegenüber dem Join-Test verbessert)

| Fall | Verhalten |
|---|---|
| Werkseinstellung | Network Steering; ohne Erfolg Wiederholung nach 1 s, 5 s, 15 s, dann 60 s, ab dem 10. Versuch alle 5 min (Akku). LED schnell. |
| Neustart im Netz, Koordinator/Parent offline | `DEVICE_REBOOT` mit Fehler -> erneute BDB-Initialisierung (Rejoin) mit demselben Backoff, statt für immer offline zu bleiben (Join-Test). |
| Im Betrieb offline | Netzüberwachung alle 5 min: zweimal hintereinander nicht im Netz und kein Versuch geplant -> Rejoin anstoßen. |
| Leave ohne Rejoin (in Z2M entfernt) | `esp_zb_factory_reset()` -> Neustart -> Pairing. Kalibrierung bleibt. |
| Werkseinstellung (Taste 10 s) | Motoren aus, NVS `rollo` löschen; im Netz: Leave senden, nach Leave-Signal bzw. spätestens 3 s `esp_zb_factory_reset()`. Lock mit Timeout; ist er nicht zu bekommen, wird `zb_storage` direkt gelöscht und neu gestartet. |

`esp_zb_factory_reset()` startet selbst neu: geprüft im Objektcode (`zb_nvram_erase()` gefolgt von `esp_restart()`), wie auch im Header beschrieben. Die ZBOSS-Bibliothek selbst ist nur als Binary vorhanden (Lizenz verbietet Disassemblieren); deren internes Rejoin-Verhalten nach Parent-Verlust ist nicht geprüft, wird aber durch die beiden Rückfallebenen oben abgefangen.

## Debug-Oberfläche (Browser)

`python3 tools/rollo-debug.py` öffnet `http://127.0.0.1:8765` und zeigt Zustand, Position, Akku, Temperatur und Zigbee-Status des Boards live. Knöpfe für Stopp, Fehler quittieren, Auf/Zu, Prozentfahrt, Tippen, Limits setzen und löschen, Kalibrieren und freie Befehle. Reiter rechts:

- **Log** mit Filter und Speichern.
- **Position:** Diagramm von Position und Duty (alle 20 ms) mit Ziel und Limits, auch für eine einzelne Fahrt. Braucht die Messspur (`trace`-Befehl, ab Firmware 0.2.6; das Tool schaltet sie selbst ein, sie schaltet sich nach 60 s von allein ab).
- **Fahrten:** Tabelle je Fahrt mit Ziel, erreichter Position, Abweichung (Schritte und %), Dauer und Ergebnis, CSV-Export. Zeile anklicken zeigt den Verlauf.
- **Akku:** Verlauf von Spannung und Temperatur; Brownouts erscheinen als roter Hinweis oben.
- **Firmware:** „Nur bauen“ und „Bauen + Flashen“ (Port wird dafür automatisch freigegeben und danach wieder geöffnet; nur bei stehendem Motor).

Braucht nur Python 3 und pyserial; der Server lauscht nur auf `127.0.0.1`. Das Programm verbindet sich selbst wieder, wenn das Board neu angesteckt wird. Zum Flashen von Hand vorher **„Port freigeben“** drücken. Das Öffnen des Ports kann den ESP neu starten.

## Debug-Konsole

Über USB-Serial/JTAG (`idf.py monitor`), Prompt `rollo>`, Schalter `ROLLO_DEBUG_CONSOLE` (an). Alle Fahrbefehle gehen in die `app_queue` – genau wie Zigbee-Befehle, also mit Akku-Sperre, Kalibrierprüfung und Fehlerzustand.

| Befehl | Wirkung |
|---|---|
| `status` | Zustand, Positionen (%, Schritte, Endpunkt), Akku/NTC, Netz + Parent-LQI |
| `up [lift\|tilt]` / `down [lift\|tilt]` | Auf / Ab (Standard Heben) |
| `stop` | Stopp; im FEHLER quittieren |
| `goto lift\|tilt <0..100>` | Prozentfahrt (0 = offen/oben) |
| `jog [lift\|tilt] <schritte>` | relativ (+ = runter); in der Kalibrierung Tippbetrieb |
| `version` | Firmware-Version (kommt auch automatisch, sobald ein USB-Host angesteckt wird) |
| `margin [on\|off]` | Sicherheitsabstand an den Endpunkten (1,5 %) ein/aus, nur im Stillstand, im NVS gespeichert. Aus: 0 % / 100 % = die gesetzten Limits. Voreinstellung `ROLLO_MARGIN_DEFAULT` (0 = aus) |
| `cal [all\|lift\|tilt]` / `cal save` / `cal abort` | Kalibrierung (braucht oben einen Anschlag) |
| `limit top` / `limit bottom` / `limit set <n>` / `limit clear` `[lift\|tilt]` | Endpunkte von Hand setzen (nur Stillstand, ohne Anschlag): mit `jog` in die obere Lage fahren, `limit top`, nach unten fahren, `limit bottom` – oder `limit set <n>` (Lage = oben, unten = n Schritte). Mindestweg `CAL_MIN_TRAVEL_STEPS_*`, wird im NVS gespeichert |
| `trace [s\|off]` | Messspur Position/Duty alle 20 ms (nur für `tools/rollo-debug.py`, schaltet sich selbst ab) |
| `ack` | Fehler quittieren |
| `batt` | Akku jetzt messen (Ergebnis im Log) |
| `reset factory` | Werkseinstellung |
| `restart` | Neustart |

## Energiesparen / Light Sleep

Standard: **aus** (`ROLLO_LIGHT_SLEEP` auskommentiert), damit die erste Inbetriebnahme stabil ist. Aktiv sind schon: nSLEEP des DRV8833 nur während Fahrten, PM-Lock während Fahrten (Teil 1), LED aus im Normalbetrieb, `sensor_task` und `ui_task` schlafen ohne Timeout, Poll 2 s.

Einschalten (Messphase, **ungetestet**): in `config.h` `#define ROLLO_LIGHT_SLEEP 1`, in `sdkconfig.defaults` den Block „Light Sleep“ (`CONFIG_FREERTOS_USE_TICKLESS_IDLE`, `CONFIG_IEEE802154_SLEEP_ENABLE`) einkommentieren, `sdkconfig` löschen, neu bauen (ohne die Kconfig-Optionen bricht der Build mit `#error` ab). Dann gilt nach dem SDK-Beispiel „light sleep“: `esp_pm_configure(light_sleep_enable)`, `esp_zb_sleep_enable(true)` vor `esp_zb_init`, `ESP_ZB_COMMON_SIGNAL_CAN_SLEEP` -> `esp_zb_sleep_now()`, `esp_zb_set_rx_on_when_idle(false)` (Sleepy End Device), Taste per `gpio_wakeup_enable(GPIO9, LOW_LEVEL)`. Weil die Taste dann einen Pegel-Interrupt hat, sperrt `ui.c` (nur mit dem Schalter) den Pin-Interrupt in der ISR und gibt ihn nach dem Loslassen wieder frei. Diese Variante baut ohne Warnung, ist aber nicht auf der Hardware erprobt.

## Inbetriebnahme (erster Flash)

Voraussetzung: Motor 1 nicht angeschlossen ist in Ordnung (Lift-Befehle enden nach ~0,3 s mit „blockiert“ -> FEHLER, quittieren mit `ack`).

1. Falls auf dieser Platine der Join-Test lief: in Z2M das alte Gerät „RolloJoinTest“ entfernen und den Flash einmal komplett löschen: `idf.py -p /dev/ttyACM0 erase-flash`. Grund: Die Partition `zb_storage` vom Join-Test (gleiche Partitionstabelle) überlebt ein normales Flashen; die neue Firmware würde sonst mit den alten Netzdaten (Endpoint 10, On/Off) wieder beitreten. Der Teil-1-NVS enthält noch keine Kalibrierung, es geht nichts verloren.
2. Flashen + Monitor: `idf.py -p /dev/ttyACM0 flash monitor`. Erwartet: Banner `Zigbee-Rollosteuerung 0.2.0`, Reset-Grund, NVS-Inhalt, `Zustand UNKALIBRIERT`, Akku-Zeile, Zigbee-Start mit Poll 2 s. Kein Motor darf sich bewegen.
3. `status` -> Zustand, Positionen, Akku plausibel (Akku-mV gegen Multimeter).
4. Wenden unkalibriert: `up tilt`, nach 1–2 s `stop` -> FAHRT -> UNKALIBRIERT, Position im Log. `down tilt`, `stop`.
5. Wenden kalibrieren: `cal tilt` -> fährt hoch bis Blockieren (Hand bereit, im Zweifel `stop`), zurück, runter; `stop` an der gewünschten Stelle, ggf. `jog 200` / `jog -200`, dann `cal save`. Danach `goto tilt 50`, `goto tilt 0`, `goto tilt 100`; `status` prüfen.
6. Taste: kurz = Heben Auf/Ab (ohne Motor 1 -> FEHLER, Doppelblitz), kurz = quittieren; 3 s = Kalibrierung (LED langsam) -> `cal abort`.
7. Zigbee: In Z2M „Permit join“ (LED blinkt schnell bis zum Beitritt). Gerät erscheint als THOMAS RolloH2. Z2M „Tilt“ setzen -> Wenden fährt; Position wird gemeldet (Endwert sofort, sonst ~1/s). Akku-Spannung/-Prozent in Z2M prüfen.
8. Netzlogik: Koordinator (Z2M) stoppen, Gerät neu starten -> Log „Neustart im bestehenden Netz fehlgeschlagen, neuer Versuch in 1 s/5 s/…“; Z2M wieder starten -> „Neustart im bestehenden Netz“.
9. Fehlerfälle: Wendeachse beim Fahren festhalten -> FEHLER blockiert; `ack`; nächster `goto tilt` macht erst eine Referenzfahrt. Neustart während einer Fahrt (`restart`) -> `moving = 1` im Log, nächste Fahrt referenziert.
10. Werkseinstellung zuletzt testen (Taste 10 s): Leave in Z2M, Neustart, Kalibrierung gelöscht.

## Schnittstellen der Services (Teil 1)

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
| app (Teil 2) | Queue 16, `APP_WDT_FEED_MS` 2000, `APP_LOST_EVENT_MS` 4000 |
| Kalibrierung | `CAL_BACKOFF_STEPS_*` 50/105, `CAL_JOG_STEPS_*` 50/105, `CAL_MIN_TRAVEL_STEPS_*` 200, `TILT_CAL_BOTTOM_BY_BLOCK` 0, `LIFT_/TILT_REHOME_ON_OPEN` 1/0, `HOME_TOL_*` 5 % / 50 (alles Schätzwerte) |
| sensor_task | `BATT_SETTLE_MS` 2000 (Annahme), `SENSOR_RETRY_MS` 5000 |
| Zigbee | Endpoint 1, `THOMAS`/`RolloH2`, Poll 2000 ms, alle Kanäle, zb_task 8 KiB, Lock-Timeout 1500 ms, Position max. 1/s, Akku alle 3 h, Netzüberwachung 5 min, `ZB_CFG_OPERATIONAL_ALWAYS` 1 |
| Schalter | `ROLLO_ZB_TEMPERATURE` an, `ROLLO_DEBUG_CONSOLE` an, `ROLLO_LIGHT_SLEEP` aus, `ROLLO_WINDOW_SENSOR` aus |

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

Beim ersten Bauen lädt der Component Manager esp-zigbee-lib 1.6.8 und esp-zboss-lib 1.6.4 nach `managed_components/` (Internet nötig). Nach Änderungen an `sdkconfig.defaults` die Datei `sdkconfig` löschen, sonst gelten die alten Werte weiter. Nach dem Start bewegt sich **kein Motor**, bis ein Befehl kommt (Taste, Zigbee, Konsole).

## Testen (PC)

```bash
make -C firmware/test_host test
```

Das kompiliert die reinen Logikdateien mit gcc (`-Wall -Wextra -Werror`) zusammen mit Unity aus `$IDF_PATH/components/unity` und führt 99 Tests aus: Akku-Kennlinie/NTC/Sperre, Achse (Prozent ↔ Schritte, Ränder, Begrenzung), Blockiererkennung, Fahrprofil gegen ein Motormodell (Kick, Rampe, Bremszone, Blockieren, Timeout, Richtungsfehler, Endpunkte, Richtungswechsel, Stopp), LIS2DH12-Umrechnung, Kipp-Logik, Taste, LED-Muster (Teil 1: 56) sowie Zustandsautomat (37: Start, Fahrten, Referenzfahrt, Plausibilität, moving-Flag, Fehler/Quittieren, Vormerken, Akku-Sperre, kompletter Kalibrierablauf inkl. Abbruch, fehlender Motor) und Zigbee-Attribute/Backoff (6). Jede Aktionsliste wird zusätzlich darauf geprüft, dass Motorbefehle am Ende stehen.

## Abweichungen von der Spezifikation

- **Anlauf:** Die Spezifikation nimmt Anlauf bei 20 % an, gemessen sind 70 % (N20). Deshalb gibt es einen Anlauf-Kick (100 %, bis der Encoder zählt), danach geht es auf `MIN_DUTY` zurück und von dort per Rampe auf `RUN_DUTY`. Alle Werte sind pro Motor einstellbar.
- **Decay:** Standard ist Fast Decay, weil alle Messungen damit gemacht wurden. Slow Decay (Spezifikation) lässt sich mit `MOTOR_SLOW_DECAY 1` einschalten, ist aber **ungetestet**.
- **Sicherheitsrand:** 0…100 % wird auf `[end_top + Rand, end_bottom − Rand]` abgebildet (Spezifikation: „1–2 % Abstand“). Prozent ist also nicht exakt `pos / end_bottom`.
- **Fenstersensor** ist komplett deaktiviert (nicht bestückt). Falls er wieder aktiviert wird: Der LIS2DH12 läuft im Normal Mode (10 Bit) mit 1 Hz statt im Low-Power-Modus.
- **config.h** liegt in `components/config/` statt in `main/` (Begründung siehe oben).
- **Zusätzliche Ereignisse:** `WRONG_DIR`, `REJECTED`, `PROGRESS` und `UI_EVT_BTN_LONG_HINT`.
- **nFAULT-Sperre** in motor_ctrl selbst (zusätzlich zum Zustand FEHLER in app_state): nach nFAULT fährt nichts mehr, bis `motor_ctrl_clear_fault()` aufgerufen wird.

Teil 2:

- **Kalibrierung je Achse:** BEREIT erst, wenn beide Achsen kalibriert sind; im Zustand UNKALIBRIERT sind Prozentfahrten für eine bereits kalibrierte Achse erlaubt (sonst wäre Wenden ohne angeschlossenen Motor 1 nicht testbar).
- **Kalibrierung auch aus BEREIT** (Neukalibrieren ohne Werkseinstellung), nicht nur aus UNKALIBRIERT.
- **`moving = 1` nach dem Neustart:** Spezifikation „nicht kalibriert -> UNKALIBRIERT“. Umgesetzt: Endpunkte bleiben gültig, Zustand BEREIT, aber vor der nächsten Prozentfahrt einer Achse automatisch Referenzfahrt oben (Spezifikation Speicher: „neu referenzieren“). Nach dem Neustart fährt trotzdem nichts von selbst.
- **Zigbee `Mode` Bit 0** (Richtung umkehren) wird nicht unterstützt (gemeldet als 0); die Richtung stellt man mit `Mx_DIR_INVERT`/`ENCx_INVERT` in `config.h` ein.
- **Kalibrierung speichern per Zigbee** über `Mode`-Bit 1 löschen (es gibt dafür keinen eigenen ZCL-Befehl).
- **Fehler quittieren per Zigbee** über den Stop-Befehl.
- **ConfigStatus Operational** standardmäßig immer 1 (`ZB_CFG_OPERATIONAL_ALWAYS`, siehe Offene Punkte).
- **Fenstersensor / Endpoint 2** nicht bestückt, nur als Gerüst hinter `ROLLO_WINDOW_SENSOR`.
- **AKKU_LEER:** LED aus statt Fehlercode (Strom sparen).
- **Akku nach Fahrten** erst 2 s nach dem Stillstand messen (`BATT_SETTLE_MS`, Annahme), damit der Spannungseinbruch die Sperre nicht auslöst.

## Review-Änderungen

Externes KI-Review (Gemini, Oktober 2026), jeder Befund gegen den Code bzw. die Header von esp-zigbee-lib 1.6.8 geprüft.

Übernommen:

- **Unterspannungssperre während der Fahrt** (`app_logic.c`): stoppt jetzt sofort (FAHRT: Stopp + Vormerkungen verwerfen, nach dem Abschluss-Ereignis AKKU_LEER, auch keine Weiterfahrt nach einer Referenzfahrt; KALIBRIERUNG in Bewegung: Stopp, danach Abbruch statt nächstem Schritt). Die Sperre löst sich weiterhin erst nach dem Ladeende. Tests: `test_lockout_during_motion`, `test_lockout_during_homing`, `test_lockout_stops_moving_calibration`, `test_lockout_during_cal_down`.
- **Kalibrier-Abbruch** (`cal_abort`): alte Endpunkte zurück **und** Referenzfahrt nötig (`moving` bleibt 1) – „oben“ kann in der abgebrochenen Kalibrierung an einem Hindernis gesetzt worden sein. Test: `test_calibration_abort_restores_old`.
- **nFAULT** setzt jetzt wie Blockieren/Timeout „Referenzfahrt nötig“ (Überstrom heißt meist Blockieren, die Achse trudelt aus). Test: `test_fault_any_state`.
- **Motorereignisse in eigener Queue** (`app_state.c`, `config.h` `APP_MOTOR_QUEUE_LEN`): PROGRESS zusammengefasst, Abschluss-Ereignisse können nicht mehr durch ein volles `app_queue` verloren gehen (vorher: Ersatz „STOPPED“ nach 4 s, ein Blockieren wäre als normaler Halt gewertet worden). `check_lost_event()` bleibt als letzte Sicherung.
- **Tippen auf unkalibrierter Achse, dann Auf/Ab** (`moving_cmd`): wird RUN statt Prozentfahrt; Prozent wird abgelehnt. Vorher lehnte motor_ctrl die Prozentfahrt ab, die FSM ging in den Ruhezustand, während der Tippauftrag weiterlief. Zusätzlich hält `exec_motion()` bei einem abgelehnten Fahrbefehl laufende Motoren an. Test: `test_uncalibrated_jog_then_up_is_run`.
- **Prozentwerte > 100 / 0xFF** werden abgewiesen statt auf 100 % (= zu) begrenzt: in `zb_device.c` (`ESP_ERR_INVALID_ARG` an den Stack) und zusätzlich in `app_logic.c` (`APP_NOTE_INVALID_VALUE`, auch während der Fahrt und für die vorgemerkte Achse). Test: `test_goto_invalid_rejected_and_replaced`.
- **Akku-Messung nach der Fahrt** (`sensor_task.c`): die spätere Anforderung gewinnt, und nach jedem Abschluss-Ereignis (auch FEHLER/Kalibrierschritt) gilt `sensor_hold_off(BATT_SETTLE_MS)` auch für die periodische Messung. Vorher konnte ein älterer Retry-Termin die 2 s Erholzeit unterlaufen -> Messung im Spannungseinbruch -> falsche Sperre bis zum Laden. (Das Zahlenbeispiel im Review war falsch, `SENSOR_RETRY_MS` ist 5 s, der Mechanismus stimmte.)
- **`s_snap` in `app_state_start()`** unter Spinlock (praktisch kein Race, weil vor Konsole/Zigbee aufgerufen, aber einheitlich).

Verworfen:

- **`cluster_role` in `send_report()`:** `esp_zb_zcl_report_attr_cmd_t` hat in 1.6.8 kein Feld `cluster_role` (würde nicht kompilieren); die Seite ergibt sich aus `direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI` (= Server-Attribut an Client), so gesetzt.
- **`ESP_ZB_BDB_SIGNAL_DEVICE_RESET`** gibt es in 1.6.8 nicht. Laut `esp_zigbee_core.h` endet `esp_zb_bdb_reset_via_local_action()` mit `ESP_ZB_ZDO_SIGNAL_LEAVE` (Typ RESET) – genau das wird behandelt.
- **`esp_zb_set_node_descriptor_power_source()` vor `esp_zb_start()`:** der Header sagt ausdrücklich „should be called after esp_zb_start()“; bleibt so.
- **`ZB_CFG_OPERATIONAL_ALWAYS` unbenutzt:** wird in `app_state.c` (`update_outputs()`) ausgewertet, bevor `zb_device_set_status()` aufgerufen wird.

## Offene Punkte / TODO

- `ENC1_PPR_X1` (JGA25) messen: `hwtest` → `enc 1 hand`.
- Motor 1 anschließen und alle `M1_*`-Werte messen: Anlauf, Blockier-Schwelle N/T, Bremszone, Drehrichtung.
- Für Motor 2 `BLOCK_*`, `BRAKE_ZONE` und die Ränder am echten Wendegetriebe einstellen. Offen ist auch noch, ob es einen Anschlag hat.
- Slow Decay testen.
- `NTC_VREF_MV` = 3300 ist angenommen, nicht gemessen.
- **Unterspannungssperre beim Laden bestätigen:** löst sich nicht, solange geladen wird (siehe Zustandsautomat). Falls anders gewünscht, in `batt_calc_lockout()` ändern.
- **Hardwaretest Teil 2** komplett offen (Ablauf unter „Inbetriebnahme“).
- **ZBOSS-Verhalten ungeprüft** (nur Binary): ob Window-Covering-Befehle bei ConfigStatus Operational = 0 oder Mode Kalibrierbit = 1 abgewiesen werden; ob der Stack bei `GoToLiftPercentage` selbst Attribute ändert. Mit Z2M testen, dann ggf. `ZB_CFG_OPERATIONAL_ALWAYS = 0`.
- **Z2M:** automatisch erzeugte Definition prüfen; sonst Converter `z2m/rollo_h2.js` testen (Optionsnamen der modernExtend-Version prüfen). Reporting-Konfiguration durch Z2M beobachten.
- **RxOnWhenIdle** ohne Light Sleep: Stack-Standard (im Log) – ob das Gerät als „sleepy“ gilt, mit Z2M prüfen.
- **Light Sleep** (Messphase): Variante baut, ungetestet. USB-Konsole bricht im Schlaf ab; Task-WDT-Fütterung der app_task (alle 2 s) und Netzüberwachung (5 min) wecken regelmäßig – für die Strommessung ggf. anpassen.
- **Parameter Kalibrierung** (`CAL_BACKOFF/JOG/MIN_TRAVEL_*`, `HOME_TOL_*`) sind Schätzwerte, Motor 1 ohne bekannte CPR.
- **Fenstersensor / IAS Zone** (Endpoint 2) bei Bestückung fertig implementieren (Enrollment, Zonenstatus, Referenzlage einlernen).
- **Ladeende-Erkennung** für AKKU_LEER: derzeit nur über die periodische Messung (10 min); ggf. CHRG_N-Interrupt.

## Version

Die Version steht in `version.txt` (SemVer) und landet im App-Descriptor und im Startbanner.
