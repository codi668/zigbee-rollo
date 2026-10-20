/*
 * Debug-Konsole (ROLLO_DEBUG_CONSOLE) über USB-Serial/JTAG für die Inbetriebnahme.
 *
 * Alle Fahr- und Kalibrierbefehle gehen als Ereignis in die app_queue – genau
 * wie Zigbee-Befehle – und laufen damit durch den Zustandsautomaten
 * (Akku-Sperre, Kalibrierung, Fehlerzustand, Plausibilität). Es gibt keinen
 * direkten Weg zu motor_ctrl.
 */
#include "console_cmds.h"

#ifdef ROLLO_DEBUG_CONSOLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_state.h"
#include "config.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "motor_ctrl.h"
#include "sensor_task.h"
#include "zb_device.h"

static const char *TAG = "console";

static int axis_arg(const char *s)
{
    if (!s) return ROLLO_TILT_ONLY ? APP_AXIS_TILT : APP_AXIS_LIFT;   /* ohne Angabe: die vorhandene Achse */
    if (!strcmp(s, "lift") || !strcmp(s, "heben") || !strcmp(s, "1")) return APP_AXIS_LIFT;
    if (!strcmp(s, "tilt") || !strcmp(s, "wenden") || !strcmp(s, "2")) return APP_AXIS_TILT;
    return -1;
}

static int post(app_cmd_t c, int axis, int32_t v)
{
    if (axis < 0) {
        printf("Achse: lift | tilt\n");
        return 1;
    }
    if (!app_post_cmd(c, (uint8_t)axis, v, APP_SRC_CONSOLE)) {
        printf("app_queue voll\n");
        return 1;
    }
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    app_state_log_status();
    zb_device_log_info();
    return 0;
}

static int cmd_up(int argc, char **argv)   { return post(APP_CMD_UP, axis_arg(argc > 1 ? argv[1] : NULL), 0); }
static int cmd_down(int argc, char **argv) { return post(APP_CMD_DOWN, axis_arg(argc > 1 ? argv[1] : NULL), 0); }
static int cmd_stop(int argc, char **argv) { (void)argc; (void)argv; return post(APP_CMD_STOP, 0, 0); }
static int cmd_ack(int argc, char **argv)  { (void)argc; (void)argv; return post(APP_CMD_ACK, 0, 0); }

static int cmd_goto(int argc, char **argv)
{
    if (argc != 3) {
        printf("goto lift|tilt <0..100>   (0 = offen/oben, 100 = zu/unten)\n");
        return 1;
    }
    char *end;
    long v = strtol(argv[2], &end, 10);
    if (*end || v < 0 || v > 100) {
        printf("Prozent 0..100\n");
        return 1;
    }
    return post(APP_CMD_GOTO, axis_arg(argv[1]), (int32_t)v);
}

static int cmd_jog(int argc, char **argv)
{
    /* jog [lift|tilt] <schritte>  (+ = nach unten). In der Kalibrierung gilt die Kalibrierachse. */
    if (argc < 2 || argc > 3) {
        printf("jog [lift|tilt] <schritte>   (+ = nach unten)\n");
        return 1;
    }
    char *end;
    long v = strtol(argv[argc - 1], &end, 10);
    if (*end || v == 0 || v > 100000 || v < -100000) {
        printf("Schritte != 0, max. +-100000\n");
        return 1;
    }
    return post(APP_CMD_JOG, axis_arg(argc == 3 ? argv[1] : NULL), (int32_t)v);
}

static int cmd_cal(int argc, char **argv)
{
    const char *a = argc > 1 ? argv[1] : "all";
    if (!strcmp(a, "all")) return post(APP_CMD_CAL_START, APP_AXIS_MASK_ALL, 0);
    if (!strcmp(a, "lift")) return post(APP_CMD_CAL_START, 1 << APP_AXIS_LIFT, 0);
    if (!strcmp(a, "tilt")) return post(APP_CMD_CAL_START, 1 << APP_AXIS_TILT, 0);
    if (!strcmp(a, "save")) return post(APP_CMD_CAL_SAVE, 0, 0);
    if (!strcmp(a, "abort")) return post(APP_CMD_CAL_ABORT, 0, 0);
    printf("cal [all|lift|tilt|save|abort]\n");
    return 1;
}

static int cmd_limit(int argc, char **argv)
{
    /* limit top|bottom|clear [lift|tilt]   |   limit set [lift|tilt] <schritte_unten> */
    const char *w = argc > 1 ? argv[1] : "";
    if (!strcmp(w, "top") && argc <= 3) return post(APP_CMD_LIMIT_TOP, axis_arg(argc == 3 ? argv[2] : NULL), 0);
    if (!strcmp(w, "bottom") && argc <= 3) return post(APP_CMD_LIMIT_BOTTOM, axis_arg(argc == 3 ? argv[2] : NULL), 0);
    if (!strcmp(w, "clear") && argc <= 3) return post(APP_CMD_LIMIT_CLEAR, axis_arg(argc == 3 ? argv[2] : NULL), 0);
    if (!strcmp(w, "set") && (argc == 3 || argc == 4)) {
        char *end;
        long v = strtol(argv[argc - 1], &end, 10);
        if (*end || v <= 0 || v > 1000000) {
            printf("Schritte 1..1000000\n");
            return 1;
        }
        return post(APP_CMD_LIMIT_SET, axis_arg(argc == 4 ? argv[2] : NULL), (int32_t)v);
    }
    printf("limit top [lift|tilt]       aktuelle Lage = oben (0), Kalibrierung ungueltig\n"
           "limit bottom [lift|tilt]    aktuelle Lage = unten, speichern (vorher 'limit top')\n"
           "limit set [lift|tilt] <n>   aktuelle Lage = oben, unten = n Schritte, speichern\n"
           "limit clear [lift|tilt]     Kalibrierung loeschen\n"
           "Nur im Stillstand. Mit 'jog' vorher in Position fahren (- = hoch, + = runter).\n");
    return 1;
}

static int cmd_batt(int argc, char **argv)
{
    (void)argc; (void)argv;
    sensor_request_measure(0);   /* Ergebnis erscheint im Log (nur im Stillstand) */
    return 0;
}

static int cmd_reset(int argc, char **argv)
{
    if (argc == 2 && !strcmp(argv[1], "factory")) return post(APP_CMD_FACTORY_RESET, 0, 0);
    printf("reset factory   -> Werkseinstellung (wie Taste 10 s): Kalibrierung loeschen, Zigbee-Netz verlassen\n");
    return 1;
}

static int cmd_margin(int argc, char **argv)
{
    /* margin [on|off]: Sicherheitsabstand an den Endpunkten (0 %/100 % = Endpunkte, wenn aus) */
    if (argc == 1) {
        printf("Sicherheitsabstand: %s\n", motor_ctrl_margins_on() ? "an" : "aus");
        return 0;
    }
    if (argc == 2 && !strcmp(argv[1], "on")) return post(APP_CMD_MARGIN, APP_AXIS_TILT, 1);
    if (argc == 2 && !strcmp(argv[1], "off")) return post(APP_CMD_MARGIN, APP_AXIS_TILT, 0);
    printf("margin [on|off]\n");
    return 1;
}

static int cmd_trace(int argc, char **argv)
{
    /* trace [sekunden|off]: Messspur Position/Duty für die Auswertung in tools/rollo-debug.py */
    uint32_t s = 60;
    if (argc > 1) {
        if (!strcmp(argv[1], "off")) s = 0;
        else {
            char *end;
            long v = strtol(argv[1], &end, 10);
            if (*end || v < 1 || v > 600) {
                printf("trace [1..600|off]\n");
                return 1;
            }
            s = (uint32_t)v;
        }
    }
    motor_ctrl_trace(s);
    printf("trace %s\n", s ? "an" : "aus");
    return 0;
}

static int cmd_restart(int argc, char **argv)
{
    (void)argc; (void)argv;
    ESP_LOGW(TAG, "Neustart per Konsole");
    esp_restart();
    return 0;
}

esp_err_t console_cmds_start(void)
{
    static const esp_console_cmd_t cmds[] = {
        { .command = "status", .help = "Zustand, Positionen, Akku, Netz", .func = cmd_status },
        { .command = "up", .help = "Auf (Heben: 0 %, unkalibriert bis Anschlag)", .hint = "[lift|tilt]", .func = cmd_up },
        { .command = "down", .help = "Ab (100 %, unkalibriert bis Stopp/Timeout!)", .hint = "[lift|tilt]", .func = cmd_down },
        { .command = "stop", .help = "Stopp (im FEHLER: quittieren)", .func = cmd_stop },
        { .command = "goto", .help = "Prozentfahrt (nur kalibrierte Achse)", .hint = "lift|tilt <0..100>", .func = cmd_goto },
        { .command = "jog", .help = "relativ fahren (+ = runter)", .hint = "[lift|tilt] <schritte>", .func = cmd_jog },
        { .command = "cal", .help = "Kalibrierung: starten / unten speichern / abbrechen", .hint = "[all|lift|tilt|save|abort]", .func = cmd_cal },
        { .command = "limit", .help = "Endpunkte manuell setzen (ohne Anschlag)", .hint = "top|bottom|set|clear ...", .func = cmd_limit },
        { .command = "ack", .help = "Fehler quittieren", .func = cmd_ack },
        { .command = "batt", .help = "Akku jetzt messen (im Stillstand)", .func = cmd_batt },
        { .command = "reset", .help = "Werkseinstellung", .hint = "factory", .func = cmd_reset },
        { .command = "margin", .help = "Sicherheitsabstand an den Endpunkten (nur Stillstand)", .hint = "[on|off]", .func = cmd_margin },
        { .command = "trace", .help = "Messspur Position/Duty (selbst abschaltend)", .hint = "[sekunden|off]", .func = cmd_trace },
        { .command = "restart", .help = "Neustart", .func = cmd_restart },
    };

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "rollo>";
    rc.max_cmdline_length = 64;
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_err_t e = esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl);
    if (e != ESP_OK) return e;
    esp_console_register_help_command();
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        e = esp_console_cmd_register(&cmds[i]);
        if (e != ESP_OK) return e;
    }
    return esp_console_start_repl(repl);
}

#else

esp_err_t console_cmds_start(void)
{
    return ESP_OK;
}

#endif
