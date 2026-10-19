# Zigbee-Rollo Hardware-Test

Test-Firmware fuer die Zigbee-Rollosteuerplatine (ESP32-H2FH4S, 4 MB Flash), ESP-IDF v5.5.
Menue ueber die eingebaute USB-Serial/JTAG (GPIO26/27, werden nicht anderweitig benutzt).

## Bauen und Flashen

```bash
export IDF_PATH=~/.platformio/packages/framework-espidf IDF_TOOLS_PATH=~/.espressif
. $IDF_PATH/export.sh
idf.py set-target esp32h2
idf.py build
idf.py -p <port> flash monitor      # Port z. B. /dev/ttyACM0
```

Pinbelegung und Konstanten: `components/board/include/board_pins.h`.

## Sicherheitshinweise

- **Motoren fahren nur auf ausdruecklichen Befehl** (`motor`, `enc <n>`, `stall`). `all` bewegt nie einen Motor.
- Jede Fahrt dauert **maximal 2 s** und stoppt automatisch (Duty 0, beide IN-Pins LOW, nSLEEP LOW).
- Duty ist auf `MOTOR_DUTY_MAX` (100 %) begrenzt, Standard 50 %; die N20-Motoren laufen am Akku erst bei 100 % an; fuer `stall` 30 %.
- nFAULT (DRV8833) wird waehrend der Fahrt ueberwacht; LOW = sofortiger Stopp und Fehlermeldung.
- Vor dem ersten Motortest: Motor mechanisch frei, Akku/Versorgung geprueft, Hand am Trenner.
- Nach Reset ist nSLEEP LOW (Treiber aus).

## Befehle und erwartete Werte

| Befehl | Was passiert | PASS wenn |
|---|---|---|
| `led` | LED (GPIO8, aktiv LOW) blinkt 5x | immer (Sichtpruefung) |
| `button [s]` | BOOT-Taste (GPIO9) s Sekunden (Std. 10) beobachten, Druecke zaehlen | mindestens 1 Druck |
| `vbat` | Akkuspannung (GPIO1, Teiler 2:1) | 3,0 ... 4,25 V |
| `ntc` | NTC (GPIO4) gegen 3V3 (Annahme exakt 3,30 V), Beta 3380 | 5 ... 45 Grad C; FAIL bei offen (~3V3) oder Kurzschluss (~0 V) |
| `charge [watch]` | CHRG_N (GPIO5) lesen; `watch`: 30 s Aenderungen live | immer (Status wird angezeigt; USB am Lader ein-/ausstecken) |
| `i2cscan` | Scan 0x08..0x77 (SDA GPIO3, SCL GPIO25) | 0x18 antwortet |
| `accel [s]` | WHO_AM_I = 0x33, dann 10 Hz, +-2 g (10 bit), s Sekunden (Std. 10): X/Y/Z in g, |a|, Neigung der Z-Achse zur Senkrechten | Mittelwert von \|a\| 0,8 ... 1,2 g |
| `drv` | nSLEEP (GPIO0) HIGH, 5 ms warten, nFAULT (GPIO2) lesen, nSLEEP LOW | nFAULT HIGH |
| `motor <1\|2> <vor\|zurueck> [duty%] [ms]` | **Motor faehrt** (Std. 50 %, max 2000 ms), PWM 20 kHz | kein nFAULT-Fehler |
| `enc <1\|2> [duty%]` | **Motor faehrt** 1 s vor, 1 s zurueck; Counts und rpm (Abtrieb) alle 200 ms | je Richtung >= 20 Counts, Vorzeichen kehrt um |
| `enc <1\|2> hand [s]` | Motor aus, Welle von Hand drehen, Counts live | Counts treten auf |
| `stall <1\|2> [duty%]` | **Motor faehrt** mit 30 %; stoppt bei 200 ms ohne Encoder-Impuls (300 ms Anlauf-Schonfrist) | Stopp ohne Fehler (blockiert erkannt oder Zeitlimit erreicht) |
| `all` | led, vbat, ntc, charge, i2cscan, accel (gekuerzt auf 3 s), drv, Zusammenfassung | alle PASS |
| `help` / `menu` | Befehlsliste | |

`all` ueberspringt `button` (interaktiv) sowie `motor`, `enc`, `stall`. `charge` laeuft ohne `watch`.

## Encoder

- PCNT, Quadratur x4, Glitch-Filter 3 us, interne Pull-ups an A/B.
- Enc 2 (Waveshare N20): 1050 Impulse/Abtriebsumdrehung bei x1 = 4200 Counts bei x4.
- **Enc 1 (JGA25-370): Impulszahl unbekannt** (`ENC1_PPR_X1 = 0` in `board_pins.h`, TODO). Solange 0, werden nur Counts angezeigt, keine rpm.
  Ermitteln: `enc 1 hand`, Abtriebswelle genau eine Umdrehung drehen, Counts/4 ablesen und als `ENC1_PPR_X1` eintragen.
- Das Vorzeichen der Counts bei "vor" haengt von der Verdrahtung ab; der Test prueft nur die Umkehr. Meldung am Ende zeigt, welches Vorzeichen "vor" hat.
- Zaehler ist 16 Bit (+-32000); fuer 2-s-Fahrten ausreichend, nicht fuer Dauerlauf.

## Struktur

```
main/                 esp_console-REPL, Befehle
components/board/     board_pins.h (Pins, Konstanten)
components/led|button|adc|charger/   passive Tests
components/accel/     I2C-Scan, LIS2DH12
components/drv/       DRV8833 nSLEEP/nFAULT
components/motor/     LEDC-PWM, Fahrt mit Zeitlimit und nFAULT-Ueberwachung
components/encoder/   PCNT, Encoder-Test, Handmodus, Blockiererkennung
```

## Hinweise

- ADC: GPIO1 = ADC1_CH0, GPIO4 = ADC1_CH3 (am H2 gueltige ADC1-Kanaele).
- Pins nicht in Verwendung: GPIO26/27 (USB).

## Version

Die Version steht in `version.txt` (SemVer) und erscheint im Startbanner und im App-Descriptor. Release: `version.txt` erhoehen, committen, `git tag vX.Y.Z`.
