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
#include "battery.h"
#include "config.h"
#include "driver/usb_serial_jtag.h"
#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

static void print_version(void)
{
    /* gleiche Zeile wie das Start-Banner in main.c, die der Monitor (tools/rollo-debug.py) auswertet */
    ESP_LOGI(TAG, "=== Zigbee-Rollosteuerung %s (ESP-IDF %s) ===", esp_app_get_description()->version,
             esp_get_idf_version());
}

static int cmd_version(int argc, char **argv)
{
    (void)argc; (void)argv;
    print_version();
    return 0;
}

/** Beim Anstecken des USB-Kabels (Host erscheint) sofort die Firmware-Version ausgeben: Ein Monitor, der sich
 *  erst nach dem Start verbindet, hat das Start-Banner sonst verpasst. */
static void usb_watch_task(void *arg)
{
    (void)arg;
    bool was = usb_serial_jtag_is_connected();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        bool now = usb_serial_jtag_is_connected();
        if (now && !was) {
            vTaskDelay(pdMS_TO_TICKS(300));   /* dem Host Zeit geben, den Port zu öffnen */
            print_version();
        }
        was = now;
    }
}

/* ---- Motor als Buzzer: beep [Hz] [ms] [Duty%] | beep ok|error|start|scale|sweep ---- */
typedef struct { uint16_t hz; uint16_t ms; } note_t;   /* hz = 0: Pause */

static const note_t MEL_OK[]    = { {1568, 110}, {0, 40}, {2093, 200} };
static const note_t MEL_ERROR[] = { {440, 220}, {0, 80}, {440, 220}, {0, 80}, {330, 420} };
static const note_t MEL_START[] = { {1047, 110}, {1319, 110}, {1568, 110}, {2093, 260} };
static const note_t MEL_SCALE[] = { {523, 200}, {587, 200}, {659, 200}, {698, 200}, {784, 200}, {880, 200}, {988, 200}, {1047, 300} };

static int play(const note_t *n, int count, uint16_t duty_permille)
{
    for (int i = 0; i < count; i++) {
        if (n[i].hz == 0) {
            vTaskDelay(pdMS_TO_TICKS(n[i].ms));
            continue;
        }
        esp_err_t e = motor_ctrl_beep(MOTOR_TILT, n[i].hz, n[i].ms, duty_permille);
        if (e != ESP_OK) {
            printf("Ton nicht moeglich: %s (Motor faehrt, Fehlersperre oder Treiberfehler)\n", esp_err_to_name(e));
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return 0;
}

static int cmd_beep(int argc, char **argv)
{
    if (battery_lockout_active()) {
        printf("Akku leer: kein Ton\n");
        return 1;
    }
    uint16_t duty = BEEP_DUTY_DEFAULT_PCT * 10;
    if (argc >= 2 && !strcmp(argv[1], "ok")) return play(MEL_OK, sizeof(MEL_OK) / sizeof(MEL_OK[0]), duty);
    if (argc >= 2 && !strcmp(argv[1], "error")) return play(MEL_ERROR, sizeof(MEL_ERROR) / sizeof(MEL_ERROR[0]), duty);
    if (argc >= 2 && !strcmp(argv[1], "start")) return play(MEL_START, sizeof(MEL_START) / sizeof(MEL_START[0]), duty);
    if (argc >= 2 && !strcmp(argv[1], "scale")) return play(MEL_SCALE, sizeof(MEL_SCALE) / sizeof(MEL_SCALE[0]), duty);
    if (argc >= 2 && !strcmp(argv[1], "sweep")) {
        /* 300…3000 Hz in 150-Hz-Schritten: zeigt, bei welcher Frequenz der Motor am lautesten ist */
        for (int f = 300; f <= 3000; f += 150) {
            printf("%d Hz\n", f);
            esp_err_t e = motor_ctrl_beep(MOTOR_TILT, (uint32_t)f, 250, duty);
            if (e != ESP_OK) {
                printf("Ton nicht moeglich: %s\n", esp_err_to_name(e));
                return 1;
            }
            vTaskDelay(pdMS_TO_TICKS(60));
        }
        return 0;
    }
    long hz = 2000, ms = 300, pct = BEEP_DUTY_DEFAULT_PCT;
    char *end;
    if (argc >= 2) { hz = strtol(argv[1], &end, 10); if (*end) goto usage; }
    if (argc >= 3) { ms = strtol(argv[2], &end, 10); if (*end) goto usage; }
    if (argc >= 4) { pct = strtol(argv[3], &end, 10); if (*end) goto usage; }
    if (hz < BEEP_FREQ_MIN_HZ || hz > BEEP_FREQ_MAX_HZ || ms < 1 || ms > BEEP_MS_MAX || pct < 1 || pct > BEEP_DUTY_MAX_PCT)
        goto usage;
    {
        esp_err_t e = motor_ctrl_beep(MOTOR_TILT, (uint32_t)hz, (uint32_t)ms, (uint16_t)(pct * 10));
        if (e != ESP_OK) {
            printf("Ton nicht moeglich: %s (Motor faehrt, Fehlersperre oder Treiberfehler)\n", esp_err_to_name(e));
            return 1;
        }
    }
    return 0;
usage:
    printf("beep [Hz %d..%d] [ms 1..%d] [Duty %% 1..%d]   Standard 2000 Hz, 300 ms, %d %%\n"
           "beep ok|error|start|scale|sweep   (Motor 2 als Lautsprecher; nur im Stillstand)\n",
           BEEP_FREQ_MIN_HZ, BEEP_FREQ_MAX_HZ, BEEP_MS_MAX, BEEP_DUTY_MAX_PCT, BEEP_DUTY_DEFAULT_PCT);
    return 1;
}

/* tune <Duty%> <Hz:ms,Hz:ms,...>   (Hz 0 = Pause), z. B. "tune 25 523:200,0:50,659:200" */
#define TUNE_MAX_NOTES 40
static int cmd_tune(int argc, char **argv)
{
    if (battery_lockout_active()) {
        printf("Akku leer: kein Ton\n");
        return 1;
    }
    if (argc != 3) goto usage;
    char *end;
    long pct = strtol(argv[1], &end, 10);
    if (*end || pct < 1 || pct > BEEP_DUTY_MAX_PCT) goto usage;
    static motor_note_t notes[TUNE_MAX_NOTES];
    size_t n = 0;
    uint32_t total = 0;
    char *save = NULL;
    for (char *tok = strtok_r(argv[2], ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        if (n >= TUNE_MAX_NOTES) { printf("max. %d Toene je Befehl\n", TUNE_MAX_NOTES); return 1; }
        char *colon = strchr(tok, ':');
        if (!colon) goto usage;
        *colon = 0;
        long hz = strtol(tok, &end, 10);
        if (*end || hz < 0 || hz > BEEP_FREQ_MAX_HZ || (hz != 0 && hz < BEEP_FREQ_MIN_HZ)) goto usage;
        long ms = strtol(colon + 1, &end, 10);
        if (*end || ms < 1 || ms > BEEP_MS_MAX) goto usage;
        notes[n].hz = (uint16_t)hz;
        notes[n].ms = (uint16_t)ms;
        total += (uint32_t)ms;
        n++;
    }
    if (n == 0 || total > 20000) { printf("leer oder laenger als 20 s\n"); return 1; }
    esp_err_t e = motor_ctrl_tune(MOTOR_TILT, notes, n, (uint16_t)(pct * 10));
    if (e != ESP_OK) {
        printf("Ton nicht moeglich: %s (Motor faehrt, Fehlersperre oder Treiberfehler)\n", esp_err_to_name(e));
        return 1;
    }
    return 0;
usage:
    printf("tune <Duty%% 1..%d> <Hz:ms,Hz:ms,...>   Hz 0 = Pause, Hz %d..%d, ms 1..%d, max. %d Toene\n",
           BEEP_DUTY_MAX_PCT, BEEP_FREQ_MIN_HZ, BEEP_FREQ_MAX_HZ, BEEP_MS_MAX, TUNE_MAX_NOTES);
    return 1;
}

static int cmd_simfault(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("simuliere nFAULT von Motor 2 (Zustand FEHLER, Fehlerton), danach mit `ack` quittieren\n");
    app_state_simulate_fault();
    return 0;
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
        { .command = "version", .help = "Firmware-Version", .func = cmd_version },
        { .command = "beep", .help = "Motor 2 als Buzzer", .hint = "[Hz] [ms] [Duty%] | ok|error|start|scale|sweep", .func = cmd_beep },
        { .command = "tune", .help = "Tonfolge auf dem Motor", .hint = "<Duty%> <Hz:ms,...>", .func = cmd_tune },
        { .command = "simfault", .help = "nFAULT simulieren (Test: Zustand FEHLER und Fehlerton)", .func = cmd_simfault },
        { .command = "margin", .help = "Sicherheitsabstand an den Endpunkten (nur Stillstand)", .hint = "[on|off]", .func = cmd_margin },
        { .command = "trace", .help = "Messspur Position/Duty (selbst abschaltend)", .hint = "[sekunden|off]", .func = cmd_trace },
        { .command = "restart", .help = "Neustart", .func = cmd_restart },
    };

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "rollo>";
    rc.max_cmdline_length = 400;   /* Tonfolgen (tune) sind lang */
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_err_t e = esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl);
    if (e != ESP_OK) return e;
    esp_console_register_help_command();
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        e = esp_console_cmd_register(&cmds[i]);
        if (e != ESP_OK) return e;
    }
    e = esp_console_start_repl(repl);
    if (e == ESP_OK) xTaskCreate(usb_watch_task, "usb_watch", 2048, NULL, 2, NULL);
    return e;
}

#else

esp_err_t console_cmds_start(void)
{
    return ESP_OK;
}

#endif
