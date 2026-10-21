# zigbee-rollo

![Firmware](https://img.shields.io/badge/firmware-0.2.14-blue) ![Hardware-Test](https://img.shields.io/badge/hwtest-0.1.2-lightgrey) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-red) ![Target](https://img.shields.io/badge/target-ESP32--H2-green)

Zigbee-Rollosteuerung (ESP32-H2FH4S, eigene Platine Rev 0.1).

- `hwtest/` – Hardware-Test-Firmware mit Konsolenmenue (Baugruppen einzeln pruefen)
- `firmware/` – die eigentliche Rollo-Firmware (Software-Uebersicht siehe Projektarbeit-Vault)

Jedes Verzeichnis ist ein eigenstaendiges ESP-IDF-Projekt (`idf.py set-target esp32h2 && idf.py build`).
