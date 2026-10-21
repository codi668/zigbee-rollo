/**
 * @file sounds.h
 * @brief Töne auf dem Motor (Buzzer) bei Ereignissen, ohne die aufrufende Task zu blockieren.
 */
#pragma once

typedef enum {
    SOUND_FAULT = 0,   /**< nFAULT-Fehler: fallende Dreiklang-Folge */
} sound_t;

/** @brief Ton in einer eigenen Task abspielen (ein Ton gleichzeitig; weitere Aufrufe werden verworfen). */
void sound_play_async(sound_t s);
