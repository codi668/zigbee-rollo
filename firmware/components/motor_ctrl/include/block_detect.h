/**
 * @file block_detect.h
 * @brief Blockiererkennung (reine Logik, ohne ESP-IDF).
 *
 * Gleitendes Zeitfenster: Blockiert, wenn sich die Position in den letzten
 * window_ms um weniger als min_steps geändert hat (PWM > 0 vorausgesetzt,
 * das prüft der Aufrufer). Erst wenn das Fenster einmal voll ist (Messwerte
 * über mindestens window_ms), kann "blockiert" gemeldet werden.
 * Zeitbasiert, also unempfindlich gegen Jitter des 10-ms-Takts.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** maximale Anzahl Stützstellen im Fenster (reicht für 600 ms bei 10 ms Takt) */
#define BLOCK_DET_MAX_SAMPLES 64

typedef struct {
    uint32_t window_ms;
    int32_t  min_steps;
    uint32_t t[BLOCK_DET_MAX_SAMPLES];
    int32_t  pos[BLOCK_DET_MAX_SAMPLES];
    uint8_t  head;   /**< ältester Eintrag */
    uint8_t  count;
} block_det_t;

/** @brief Parameter setzen und Fenster leeren. */
void block_det_init(block_det_t *b, uint32_t window_ms, int32_t min_steps);

/** @brief Fenster leeren (z. B. am Ende der Anlauframpe = "scharf schalten"). */
void block_det_reset(block_det_t *b);

/**
 * @brief Neuen Messwert eintragen.
 * @param now_ms Zeitstempel (monoton, Überlauf erlaubt)
 * @param pos    Encoder-Position
 * @return true = blockiert
 */
bool block_det_update(block_det_t *b, uint32_t now_ms, int32_t pos);
