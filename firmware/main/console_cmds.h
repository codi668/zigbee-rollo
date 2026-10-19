/**
 * @file console_cmds.h
 * @brief Debug-Konsole (ROLLO_DEBUG_CONSOLE in config.h): status, up, down, stop,
 * goto, jog, cal, ack, batt, reset factory, restart. Ohne den Schalter ein leerer Aufruf.
 */
#pragma once

#include "config.h"
#include "esp_err.h"

/** @brief REPL "rollo>" auf USB-Serial/JTAG starten. */
esp_err_t console_cmds_start(void);
