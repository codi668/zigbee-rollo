# zigbee-rollo

Zigbee-Rollosteuerung (ESP32-H2FH4S, eigene Platine Rev 0.1).

- `hwtest/` – Hardware-Test-Firmware mit Konsolenmenue (Baugruppen einzeln pruefen)
- `firmware/` – die eigentliche Rollo-Firmware (Software-Uebersicht siehe Projektarbeit-Vault)

Jedes Verzeichnis ist ein eigenstaendiges ESP-IDF-Projekt (`idf.py set-target esp32h2 && idf.py build`).
