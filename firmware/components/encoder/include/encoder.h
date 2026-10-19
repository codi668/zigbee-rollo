/**
 * @file encoder.h
 * @brief Quadratur-Encoder per PCNT (Schicht 4): x4, Glitch-Filter, 32-Bit-Position.
 *
 * - Encoder 1 = Motor 1 (Heben, GPIO13/14), Encoder 2 = Motor 2 (Wenden, GPIO23/24).
 * - Der 16-Bit-PCNT-Zähler wird über accum_count + Watchpoints an ±ENC_PCNT_LIMIT
 *   vom IDF-Treiber zu einem 32-Bit-Wert aufsummiert (hwtest erprobt).
 * - Zählrichtung per ENC1_INVERT/ENC2_INVERT (config.h) umkehrbar.
 * - Position = Hardware-Zählerstand (ggf. invertiert) + Offset; encoder_set()
 *   verschiebt nur den Offset, der Zähler läuft ungestört weiter.
 *
 * Thread-sicher (kurzer Spinlock um Offset/Lesen).
 * Hinweis: Im Light Sleep zählt der PCNT nicht – motor_ctrl hält während einer
 * Fahrt einen PM-Lock. Handbewegungen im Schlaf gehen verloren.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/** Encoder-Kanal */
typedef enum {
    ENCODER_1 = 0,   /**< JGA25-370, Heben */
    ENCODER_2 = 1,   /**< N20, Wenden */
    ENCODER_COUNT
} encoder_id_t;

/** @brief Beide PCNT-Units einrichten und starten (interne Pull-ups an A/B). */
esp_err_t encoder_init(void);

/** @brief Aktuelle Position in Zählschritten (x4, vorzeichenbehaftet, Invert berücksichtigt). */
int32_t encoder_get(encoder_id_t id);

/** @brief Position auf einen Wert setzen (Referenzieren, z. B. 0 am oberen Anschlag). */
void encoder_set(encoder_id_t id, int32_t pos);

/**
 * @brief Zählschritte pro Abtriebsumdrehung (PPR_X1 * 4).
 * @return 0 = unbekannt (ENC1_PPR_X1 = 0) – dann keine Drehzahl berechnen!
 */
int32_t encoder_counts_per_rev(encoder_id_t id);
