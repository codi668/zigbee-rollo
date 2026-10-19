// Zigbee2MQTT External Converter – Zigbee-Rollosteuerung "RolloH2"
//
// *** ENTWURF, UNGETESTET ***  (Stand Firmware 0.2.0)
//
// Z2M erzeugt für unbekannte Geräte inzwischen selbst eine Definition aus den
// Clustern ("generated definition"). Diese Datei ist nur nötig, wenn das nicht
// reicht (z. B. Tilt fehlt, Name/Beschreibung, Kalibriermodus-Schalter).
//
// Einbinden (Z2M 2.x): Datei nach <z2m-data>/external_converters/ kopieren,
// Z2M neu starten. (Z2M 1.x: in configuration.yaml unter external_converters.)
// Danach das Gerät in Z2M neu interviewen bzw. "Reconfigure" ausführen.
//
// Hinweise:
// - Firmware: 0 % = ganz offen (oben), 100 % = ganz zu (unten), wie ZCL.
//   Z2M/HA zeigen je nach "invert_cover" umgekehrt an.
// - Kalibriermodus: Window-Covering-Attribut "windowCoveringMode" Bit 1
//   (Wert 2 = starten, 0 = Endpunkt unten speichern bzw. abbrechen).
//   Testweise auch über das Z2M-Dev-Console-Panel schreibbar.
// - Optionsnamen von modernExtend.windowCovering/battery bitte gegen die
//   installierte zigbee-herdsman-converters-Version prüfen (TODO).

const m = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['RolloH2'],
    model: 'RolloH2',
    vendor: 'THOMAS',
    description: 'Eigenbau-Raffstoresteuerung (Heben + Wenden), ESP32-H2, Akku',
    extend: [
        m.windowCovering({controls: ['lift', 'tilt']}),
        m.battery({percentage: true, voltage: true}),
        m.temperature(),
    ],
    meta: {},
};

module.exports = definition;
