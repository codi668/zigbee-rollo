/* Blockiererkennung – reine Logik, auf dem Host getestet (test_host/test_block_detect.c) */
#include "block_detect.h"

void block_det_init(block_det_t *b, uint32_t window_ms, int32_t min_steps)
{
    b->window_ms = window_ms;
    b->min_steps = min_steps;
    block_det_reset(b);
}

void block_det_reset(block_det_t *b)
{
    b->head = 0;
    b->count = 0;
}

#define IDX(b, k) (uint8_t)(((b)->head + (k)) % BLOCK_DET_MAX_SAMPLES)

bool block_det_update(block_det_t *b, uint32_t now_ms, int32_t pos)
{
    /* Eintragen; bei vollem Puffer den ältesten verwerfen */
    if (b->count == BLOCK_DET_MAX_SAMPLES) {
        b->head = IDX(b, 1);
        b->count--;
    }
    uint8_t w = IDX(b, b->count);
    b->t[w] = now_ms;
    b->pos[w] = pos;
    b->count++;

    /* Ältesten Eintrag so wählen, dass er gerade noch >= window_ms alt ist:
     * solange auch der zweitälteste schon alt genug ist, den ältesten verwerfen. */
    while (b->count >= 2 && (uint32_t)(now_ms - b->t[IDX(b, 1)]) >= b->window_ms) {
        b->head = IDX(b, 1);
        b->count--;
    }
    if ((uint32_t)(now_ms - b->t[b->head]) < b->window_ms) return false;   /* Fenster noch nicht voll */

    int32_t d = pos - b->pos[b->head];
    if (d < 0) d = -d;
    return d < b->min_steps;
}
