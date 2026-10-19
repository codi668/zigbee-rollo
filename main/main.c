/*
 * Zigbee-Rollo Hardware-Test (ESP32-H2FH4S)
 * Menue ueber USB-Serial/JTAG (esp_console). "help" zeigt alle Befehle.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board_pins.h"
#include "led.h"
#include "button.h"
#include "adc_meas.h"
#include "charger.h"
#include "accel.h"
#include "drv.h"
#include "motor.h"
#include "encoder.h"
#include "esp_console.h"
#include "esp_app_desc.h"

#define ALL_ACCEL_S 3   /* in "all" gekuerzt (Einzelbefehl: 10 s) */

static bool parse_int(const char *s, int *out)
{
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_id(const char *s, int *id)
{
    return parse_int(s, id) && (*id == 1 || *id == 2);
}

static int cmd_led(int argc, char **argv)    { led_test(); return 0; }
static int cmd_vbat(int argc, char **argv)   { vbat_test(); return 0; }
static int cmd_ntc(int argc, char **argv)    { ntc_test(); return 0; }
static int cmd_i2cscan(int argc, char **argv) { i2cscan_test(); return 0; }
static int cmd_drv(int argc, char **argv)    { drv_test(); return 0; }

static int cmd_button(int argc, char **argv)
{
    int s = 10;
    if (argc > 1 && (!parse_int(argv[1], &s) || s < 1 || s > 120)) {
        printf("Aufruf: button [sekunden 1..120]\n");
        return 1;
    }
    button_test(s);
    return 0;
}

static int cmd_charge(int argc, char **argv)
{
    int watch = 0;
    if (argc > 1) {
        if (strcmp(argv[1], "watch") != 0) { printf("Aufruf: charge [watch]\n"); return 1; }
        watch = 30;
    }
    charger_test(watch);
    return 0;
}

static int cmd_accel(int argc, char **argv)
{
    int s = 10;
    if (argc > 1 && (!parse_int(argv[1], &s) || s < 1 || s > 60)) {
        printf("Aufruf: accel [sekunden 1..60]\n");
        return 1;
    }
    accel_test(s);
    return 0;
}

/* motor <1|2> <vor|zurueck> [duty%] [ms] */
static int cmd_motor(int argc, char **argv)
{
    int id, duty = MOTOR_DUTY_DEFAULT, ms = MOTOR_RUN_MAX_MS, dir;
    if (argc < 3 || !parse_id(argv[1], &id)) goto usage;
    if (!strcmp(argv[2], "vor") || !strcmp(argv[2], "fwd")) dir = MOTOR_FWD;
    else if (!strcmp(argv[2], "zurueck") || !strcmp(argv[2], "back")) dir = MOTOR_BACK;
    else goto usage;
    if (argc > 3 && (!parse_int(argv[3], &duty) || duty < 1 || duty > 100)) goto usage;
    if (argc > 4 && (!parse_int(argv[4], &ms) || ms < 1)) goto usage;
    motor_test(id, dir, duty, ms);
    return 0;
usage:
    printf("Aufruf: motor <1|2> <vor|zurueck> [duty%% 1..100, Standard %d, max %d] [ms, max %d]\n",
           MOTOR_DUTY_DEFAULT, MOTOR_DUTY_MAX, MOTOR_RUN_MAX_MS);
    return 1;
}

/* enc <1|2> [duty%] | enc <1|2> hand [sekunden] */
static int cmd_enc(int argc, char **argv)
{
    int id, v = 0;
    if (argc < 2 || !parse_id(argv[1], &id)) goto usage;
    if (argc > 2 && !strcmp(argv[2], "hand")) {
        int s = 10;
        if (argc > 3 && (!parse_int(argv[3], &s) || s < 1 || s > 120)) goto usage;
        enc_hand_test(id, s);
        return 0;
    }
    if (argc > 2 && (!parse_int(argv[2], &v) || v < 1 || v > 100)) goto usage;
    enc_test(id, v);
    return 0;
usage:
    printf("Aufruf: enc <1|2> [duty%%]   (faehrt den Motor 2x 1 s)\n"
           "        enc <1|2> hand [s]  (Motor aus, nur zaehlen)\n");
    return 1;
}

/* stall <1|2> [duty%] */
static int cmd_stall(int argc, char **argv)
{
    int id, duty = 0;
    if (argc < 2 || !parse_id(argv[1], &id)) goto usage;
    if (argc > 2 && (!parse_int(argv[2], &duty) || duty < 1 || duty > 100)) goto usage;
    stall_test(id, duty);
    return 0;
usage:
    printf("Aufruf: stall <1|2> [duty%%, Standard 30]\n");
    return 1;
}

static int cmd_all(int argc, char **argv)
{
    struct { const char *name; bool ok; } r[7];
    int n = 0;
    printf("=== all: nur passive Tests (kein Motor) ===\n");
#define RUN(label, call) do { printf("\n--- %s ---\n", label); r[n].name = label; r[n].ok = call; n++; } while (0)
    RUN("led", led_test());
    RUN("vbat", vbat_test());
    RUN("ntc", ntc_test());
    RUN("charge", charger_test(0));
    RUN("i2cscan", i2cscan_test());
    RUN("accel", accel_test(ALL_ACCEL_S));
    RUN("drv", drv_test());
#undef RUN
    printf("\n=== Zusammenfassung ===\n");
    int fails = 0;
    for (int i = 0; i < n; i++) {
        printf("  %-8s %s\n", r[i].name, r[i].ok ? "PASS" : "FAIL");
        if (!r[i].ok) fails++;
    }
    printf("uebersprungen: button (interaktiv), motor, enc, stall (bewegen Motoren, nur auf Befehl)\n");
    printf("Gesamt: %s (%d von %d PASS)\n", fails ? "FAIL" : "PASS", n - fails, n);
    return 0;
}

static int cmd_help_text(int argc, char **argv)
{
    printf(
        "Befehle:\n"
        "  led                          LED 5x blinken lassen (Sichtpruefung)\n"
        "  button [s]                   BOOT-Taste s Sekunden lang pruefen (Standard 10)\n"
        "  vbat                         Akkuspannung messen\n"
        "  ntc                          NTC-Temperatur messen\n"
        "  charge [watch]               Ladestatus (watch: 30 s Aenderungen anzeigen)\n"
        "  i2cscan                      I2C-Scan, LIS2DH12 auf 0x18 erwartet\n"
        "  accel [s]                    Beschleunigungssensor, s Sekunden (Standard 10)\n"
        "  drv                          DRV8833 nSLEEP/nFAULT pruefen (ohne Motorlauf)\n"
        "  motor <1|2> <vor|zurueck> [duty%%] [ms]   MOTOR FAEHRT (max %d ms, max %d %% Duty)\n"
        "  enc <1|2> [duty%%]            MOTOR FAEHRT: Encoder + Zaehlrichtung pruefen\n"
        "  enc <1|2> hand [s]           Encoder von Hand drehen, Motor aus\n"
        "  stall <1|2> [duty%%]          MOTOR FAEHRT: Blockiererkennung (Standard 30 %%)\n"
        "  all                          alle passiven Tests (kein Motor, button uebersprungen)\n"
        "  help                         diese Hilfe\n",
        MOTOR_RUN_MAX_MS, MOTOR_DUTY_MAX);
    return 0;
}

static void reg(const char *name, const char *help, esp_console_cmd_func_t fn)
{
    esp_console_cmd_t c = { .command = name, .help = help, .func = fn };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));
}

void app_main(void)
{
    led_init();
    button_init();
    charger_init();
    drv_init();     /* nSLEEP zuerst LOW */
    motor_init();
    adc_meas_init();
    enc_init();

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rcfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rcfg.prompt = "hwtest> ";
    rcfg.task_stack_size = 8192; /* printf mit Float */
    esp_console_dev_usb_serial_jtag_config_t dcfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&dcfg, &rcfg, &repl));

    reg("led", "LED-Test", cmd_led);
    reg("button", "BOOT-Taste testen", cmd_button);
    reg("vbat", "Akkuspannung", cmd_vbat);
    reg("ntc", "NTC-Temperatur", cmd_ntc);
    reg("charge", "Ladestatus [watch]", cmd_charge);
    reg("i2cscan", "I2C-Scan", cmd_i2cscan);
    reg("accel", "LIS2DH12 Beschleunigung", cmd_accel);
    reg("drv", "DRV8833 nSLEEP/nFAULT", cmd_drv);
    reg("motor", "Motor fahren (max 2 s)", cmd_motor);
    reg("enc", "Encoder-Test", cmd_enc);
    reg("stall", "Blockiererkennung", cmd_stall);
    reg("all", "alle passiven Tests", cmd_all);
    reg("menu", "Befehlsuebersicht", cmd_help_text);
    esp_console_register_help_command();

    printf("\n=== Zigbee-Rollo Hardware-Test v%s ===\n", esp_app_get_description()->version);
    printf("'help' bzw. 'menu' zeigt die Befehle. Motorbefehle fahren nur auf ausdruecklichen Befehl.\n");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
