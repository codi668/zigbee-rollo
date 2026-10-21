/*
 * zb_device: Zigbee End Device (esp-zigbee-lib 1.6.8 / esp-zboss-lib 1.6.4).
 *
 * Grundlage: zigbee-rollo-join-test (auf der Hardware erprobt: Beitritt zum
 * Z2M-Netz, Befehle). Verbesserungen gegenüber dem Join-Test:
 *   - Rejoin nach Neustart im Netz (DEVICE_REBOOT mit Fehler) wird mit Backoff
 *     wiederholt statt aufzugeben; zusätzlich Netzüberwachung alle 5 min.
 *   - Network Steering mit Backoff 1 s -> 5 s -> 15 s -> 60 s ... 300 s statt jede Sekunde.
 *   - Werkseinstellung ohne portMAX_DELAY; esp_zb_factory_reset() startet selbst neu
 *     (geprüft im Objektcode der Apache-2.0-lizenzierten esp-zigbee-lib:
 *     zb_nvram_erase() gefolgt von esp_restart()); Notfallpfad ohne Lock.
 *   - Stack der zb_task ZB_TASK_STACK (8 KiB) statt 4 KiB.
 *
 * Alle Stack-Callbacks laufen in der zb_task und legen nur Ereignisse in die
 * app_queue. Funktionen, die aus anderen Tasks kommen, halten den Zigbee-Lock
 * (mit Timeout).
 */
#include "zb_device.h"
#include <inttypes.h>
#include <string.h>
#include "config.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"   /* nicht in esp_zigbee_core.h enthalten */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zb_logic.h"

#if !defined ZB_ED_ROLE
#error "ZB_ED_ROLE nicht gesetzt: CONFIG_ZB_ZED=y in sdkconfig.defaults pruefen (sdkconfig ggf. loeschen)."
#endif

#ifdef ROLLO_LIGHT_SLEEP
#if !CONFIG_FREERTOS_USE_TICKLESS_IDLE || !CONFIG_PM_ENABLE
#error "ROLLO_LIGHT_SLEEP braucht CONFIG_PM_ENABLE und CONFIG_FREERTOS_USE_TICKLESS_IDLE (Block 'Light Sleep' in sdkconfig.defaults)"
#endif
#endif

static const char *TAG = "zb";

/* Sleepy End Device: fest mit ROLLO_LIGHT_SLEEP, sonst nur auf Anforderung (Leistungstest, powertest.c) */
#ifdef ROLLO_LIGHT_SLEEP
static bool s_sleepy = true;
#else
static bool s_sleepy = false;
#endif
void zb_device_set_sleepy(bool on) { s_sleepy = on; }

#define EP              ROLLO_ZB_ENDPOINT
#define WC_CLUSTER      ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING
#define PWR_CLUSTER     ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG
#define TEMP_CLUSTER    ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT
#define SERVER          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE

static const uint16_t s_pos_attr_id[APP_AXIS_COUNT] = {
    ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
    ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
};

/* ---- Schattenkopie der Attribute (unter s_sh_lock) ---- */
typedef struct {
    uint8_t pos[APP_AXIS_COUNT];
    uint8_t batt_v;
    uint8_t batt_pct;
    int16_t temp;
    bool    batt_valid;
    bool    lockout;
    uint8_t cfg_status;
    uint8_t mode;
} zb_shadow_t;

static portMUX_TYPE s_sh_lock = portMUX_INITIALIZER_UNLOCKED;
static zb_shadow_t s_sh;
static volatile bool s_started;          /* Stack läuft, Lock darf benutzt werden */
static volatile app_net_t s_net = APP_NET_OFFLINE;

/* ---- nur unter Zigbee-Lock bzw. in der zb_task ---- */
static uint32_t s_rep_last_ms[APP_AXIS_COUNT];
static bool     s_rep_sent[APP_AXIS_COUNT];
static int64_t  s_batt_rep_us;
static bool     s_batt_rep_sent;
static uint32_t s_steer_attempt;
static uint32_t s_rejoin_attempt;
static bool     s_retry_pending;
static uint8_t  s_netcheck_fail;

/* ZCL-Strings: erstes Byte = Länge */
static char s_zcl_manuf[33];
static char s_zcl_model[33];
static char s_zcl_swbuild[17];

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void zcl_str(char *dst, size_t cap, const char *src)
{
    size_t n = strlen(src);
    if (n > cap - 1) n = cap - 1;
    dst[0] = (char)n;
    memcpy(dst + 1, src, n);
}

static void set_net(app_net_t n)
{
    if (s_net != n) {
        s_net = n;
        app_post_net(n);
    }
}

/* ------------------------------------------------------------------------- */
/* Attribute (Lock gehalten oder zb_task)                                     */

static void set_attr(uint16_t cluster, uint16_t attr, void *val)
{
    esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(EP, cluster, SERVER, attr, val, false);
    if (st != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Attribut 0x%04x/0x%04x setzen: ZCL-Status 0x%02x", cluster, attr, st);
}

/** Einmalige Meldung direkt an den Koordinator (unabhängig von Bindings/Reporting-Konfiguration) */
static void send_report(uint16_t cluster, uint16_t attr)
{
    if (!esp_zb_bdb_dev_joined()) return;
    esp_zb_zcl_report_attr_cmd_t cmd = { 0 };
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;   /* Koordinator */
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = EP;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = cluster;
    cmd.attributeID = attr;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    esp_err_t e = esp_zb_zcl_report_attr_cmd_req(&cmd);
    if (e != ESP_OK) ESP_LOGW(TAG, "Report 0x%04x/0x%04x: %s", cluster, attr, esp_err_to_name(e));
}

static void apply_all(void)
{
    zb_shadow_t sh;
    taskENTER_CRITICAL(&s_sh_lock);
    sh = s_sh;
    taskEXIT_CRITICAL(&s_sh_lock);
    for (int i = 0; i < APP_AXIS_COUNT; i++) set_attr(WC_CLUSTER, s_pos_attr_id[i], &sh.pos[i]);
    set_attr(WC_CLUSTER, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID, &sh.cfg_status);
    set_attr(WC_CLUSTER, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID, &sh.mode);
    if (sh.batt_valid) {
        set_attr(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &sh.batt_v);
        set_attr(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &sh.batt_pct);
#ifdef ROLLO_ZB_TEMPERATURE
        set_attr(TEMP_CLUSTER, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &sh.temp);
#endif
    }
}

static void report_battery(void)
{
    send_report(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID);
    send_report(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID);
#ifdef ROLLO_ZB_TEMPERATURE
    send_report(TEMP_CLUSTER, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
#endif
    s_batt_rep_us = esp_timer_get_time();
    s_batt_rep_sent = true;
}

static void report_all(void)
{
    for (int i = 0; i < APP_AXIS_COUNT; i++) {
        send_report(WC_CLUSTER, s_pos_attr_id[i]);
        s_rep_last_ms[i] = now_ms();
        s_rep_sent[i] = true;
    }
    bool bv;
    taskENTER_CRITICAL(&s_sh_lock);
    bv = s_sh.batt_valid;
    taskEXIT_CRITICAL(&s_sh_lock);
    if (bv) report_battery();
}

/* ------------------------------------------------------------------------- */
/* Netzlogik (zb_task)                                                        */

/* Muss mit gehaltenem Lock oder aus der zb_task aufgerufen werden. */
static void log_network_info(void)
{
    esp_zb_ieee_addr_t ext;
    esp_zb_get_extended_pan_id(ext);
    ESP_LOGI(TAG, "Netz: PAN 0x%04x, ext. PAN %02x%02x%02x%02x%02x%02x%02x%02x, Kanal %d, Adresse 0x%04x",
             esp_zb_get_pan_id(), ext[7], ext[6], ext[5], ext[4], ext[3], ext[2], ext[1], ext[0],
             esp_zb_get_current_channel(), esp_zb_get_short_address());
    esp_zb_nwk_info_iterator_t it = ESP_ZB_NWK_INFO_ITERATOR_INIT;
    esp_zb_nwk_neighbor_info_t nbr;
    while (esp_zb_nwk_get_next_neighbor(&it, &nbr) == ESP_OK) {
        if (nbr.relationship == ESP_ZB_NWK_RELATIONSHIP_PARENT)
            ESP_LOGI(TAG, "Parent 0x%04x: LQI %u, RSSI %d dBm", nbr.short_addr, nbr.lqi, nbr.rssi);
    }
}

static void commission_cb(uint8_t mode);

static void schedule_commission(uint8_t mode, uint32_t delay_ms)
{
    s_retry_pending = true;
    esp_zb_scheduler_alarm(commission_cb, mode, delay_ms);
}

static void commission_cb(uint8_t mode)
{
    s_retry_pending = false;
    esp_err_t e = esp_zb_bdb_start_top_level_commissioning(mode);
    if (e != ESP_OK) {
        uint32_t d = zbl_backoff_ms(mode == ESP_ZB_BDB_MODE_NETWORK_STEERING ? s_steer_attempt++ : s_rejoin_attempt++);
        ESP_LOGW(TAG, "Commissioning (Modus %u) nicht gestartet: %s, neuer Versuch in %" PRIu32 " s",
                 mode, esp_err_to_name(e), d / 1000);
        schedule_commission(mode, d);
    }
}

static void on_joined(const char *how)
{
    ESP_LOGI(TAG, "%s", how);
    s_steer_attempt = 0;
    s_rejoin_attempt = 0;
    s_netcheck_fail = 0;
    set_net(APP_NET_JOINED);
    log_network_info();
    apply_all();
    report_all();
}

/** Netzüberwachung: zweimal hintereinander nicht im Netz und kein Versuch geplant -> Rejoin */
static void netcheck_cb(uint8_t param)
{
    (void)param;
    if (esp_zb_bdb_dev_joined()) {
        s_netcheck_fail = 0;
        if (s_net != APP_NET_JOINED) on_joined("wieder im Netz");
    } else if (!esp_zb_bdb_is_factory_new() && !s_retry_pending) {
        if (++s_netcheck_fail >= 2) {
            s_netcheck_fail = 0;
            ESP_LOGW(TAG, "Netzueberwachung: offline -> Rejoin");
            set_net(APP_NET_OFFLINE);
            schedule_commission(ESP_ZB_BDB_MODE_INITIALIZATION, 10);
        }
    }
    esp_zb_scheduler_alarm(netcheck_cb, 0, ZB_NET_CHECK_MS);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = *p_sg_p;

    switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack initialisiert, starte BDB-Initialisierung");
#if ZB_TX_POWER_DBM != 0
        esp_zb_set_tx_power(ZB_TX_POWER_DBM);
        {
            int8_t pw = 0;
            esp_zb_get_tx_power(&pw);
            ESP_LOGI(TAG, "Sendeleistung %d dBm (ZB_TX_POWER_DBM)", pw);
        }
#endif
        schedule_commission(ESP_ZB_BDB_MODE_INITIALIZATION, 0);
        esp_zb_scheduler_alarm(netcheck_cb, 0, ZB_NET_CHECK_MS);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Werkseinstellung: Network Steering (Permit Join in Z2M einschalten)");
                set_net(APP_NET_JOINING);
                schedule_commission(ESP_ZB_BDB_MODE_NETWORK_STEERING, 0);
            } else {
                on_joined("Neustart im bestehenden Netz");
            }
        } else {
            /* Join-Test blieb hier für immer offline (z. B. Koordinator aus). Jetzt: Backoff. */
            uint32_t d = zbl_backoff_ms(s_rejoin_attempt++);
            ESP_LOGW(TAG, "%s fehlgeschlagen (%s), neuer Versuch in %" PRIu32 " s",
                     esp_zb_zdo_signal_to_string(sig), esp_err_to_name(err), d / 1000);
            set_net(esp_zb_bdb_is_factory_new() ? APP_NET_JOINING : APP_NET_OFFLINE);
            schedule_commission(ESP_ZB_BDB_MODE_INITIALIZATION, d);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            on_joined("Netz beigetreten");
        } else {
            uint32_t d = zbl_backoff_ms(s_steer_attempt++);
            ESP_LOGI(TAG, "Network Steering ohne Erfolg (%s), neuer Versuch in %" PRIu32 " s",
                     esp_err_to_name(err), d / 1000);
            set_net(APP_NET_JOINING);
            schedule_commission(ESP_ZB_BDB_MODE_NETWORK_STEERING, d);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE: {
        esp_zb_zdo_signal_leave_params_t *lp = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGW(TAG, "Netz verlassen (Typ %d)", lp ? lp->leave_type : -1);
        set_net(APP_NET_OFFLINE);
        if (lp && lp->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            /* ohne Rejoin (lokale Werkseinstellung oder in Z2M entfernt): Zigbee-Daten
             * löschen und neu starten -> danach Network Steering. Kalibrierung bleibt. */
            esp_zb_factory_reset();   /* startet neu */
        }
        break;
    }

    case ESP_ZB_NLME_STATUS_INDICATION: {
        esp_zb_zdo_signal_nwk_status_indication_params_t *p =
            (esp_zb_zdo_signal_nwk_status_indication_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (p) ESP_LOGW(TAG, "NWK-Status 0x%02x (Adresse 0x%04x)", p->status, p->network_addr);
        break;
    }

    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        if (s_sleepy) esp_zb_sleep_now();
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGD(TAG, "Signal %s", esp_zb_zdo_signal_to_string(sig));
        break;

    default:
        ESP_LOGI(TAG, "Signal %s (0x%x), Status %s", esp_zb_zdo_signal_to_string(sig), sig, esp_err_to_name(err));
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Befehle (zb_task): nur Ereignisse erzeugen                                 */

static esp_err_t on_movement(const esp_zb_zcl_window_covering_movement_message_t *m)
{
    if (!m || m->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return ESP_ERR_INVALID_ARG;
    if (m->info.dst_endpoint != EP) return ESP_OK;
    bool ok;
    switch (m->command) {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        ok = app_post_cmd(APP_CMD_UP, APP_AXIS_LIFT, 0, APP_SRC_ZIGBEE);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        ok = app_post_cmd(APP_CMD_DOWN, APP_AXIS_LIFT, 0, APP_SRC_ZIGBEE);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        ok = app_post_cmd(APP_CMD_STOP, 0, 0, APP_SRC_ZIGBEE);
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE: {
        bool lift = m->command == ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE;
        uint8_t pct = lift ? m->payload.percentage_lift_value : m->payload.percentage_tilt_value;
        if (pct > 100) {
            /* 0xFF = "unbekannt" o. Ä.: abweisen statt auf 100 % (= zu) zu begrenzen.
             * app_logic prüft zusätzlich (APP_NOTE_INVALID_VALUE). */
            ESP_LOGW(TAG, "GoTo%sPercentage %u abgewiesen (erlaubt 0..100)", lift ? "Lift" : "Tilt", pct);
            return ESP_ERR_INVALID_ARG;
        }
        ok = app_post_cmd(APP_CMD_GOTO, lift ? APP_AXIS_LIFT : APP_AXIS_TILT, pct, APP_SRC_ZIGBEE);
        break;
    }
    default:
        /* GoToLift/TiltValue: keine absoluten Werte (InstalledLimits) implementiert */
        ESP_LOGW(TAG, "Window-Covering-Befehl 0x%02x nicht unterstuetzt", m->command);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!ok) ESP_LOGE(TAG, "app_queue voll: Befehl 0x%02x verworfen", m->command);
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t on_set_attr(const esp_zb_zcl_set_attr_value_message_t *m)
{
    if (!m || m->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return ESP_ERR_INVALID_ARG;
    if (m->info.dst_endpoint == EP && m->info.cluster == WC_CLUSTER &&
        m->attribute.id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID && m->attribute.data.value &&
        m->attribute.data.size >= 1) {
        uint8_t mode = *(const uint8_t *)m->attribute.data.value;
        ESP_LOGI(TAG, "Mode geschrieben: 0x%02x", mode);
        return app_post_cmd(APP_CMD_MODE_WRITE, 0, mode, APP_SRC_ZIGBEE) ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t id, const void *message)
{
    switch (id) {
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        return on_movement((const esp_zb_zcl_window_covering_movement_message_t *)message);
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return on_set_attr((const esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        ESP_LOGD(TAG, "Action-Callback 0x%x", id);
        return ESP_OK;
    }
}

static void on_identify(uint8_t on)
{
    app_post_identify(on != 0);
}

/* ------------------------------------------------------------------------- */
/* Datenmodell                                                                */

static void check(const char *what, esp_err_t e)
{
    if (e != ESP_OK) ESP_LOGE(TAG, "%s: %s", what, esp_err_to_name(e));
}

/** Attribut anlegen; existiert es schon (vom Cluster-Create), nur den Wert setzen */
static void wc_attr(esp_zb_attribute_list_t *l, uint16_t id, void *val)
{
    if (esp_zb_window_covering_cluster_add_attr(l, id, val) != ESP_OK)
        check("WC update_attr", esp_zb_cluster_update_attr(l, id, val));
}

static esp_zb_ep_list_t *create_endpoints(void)
{
    zb_shadow_t sh;
    taskENTER_CRITICAL(&s_sh_lock);
    sh = s_sh;
    taskEXIT_CRITICAL(&s_sh_lock);

    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    /* Basic: Batteriebetrieb, Hersteller/Modell, Firmware-Version */
    esp_zb_basic_cluster_cfg_t bcfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&bcfg);
    check("Basic Hersteller", esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, s_zcl_manuf));
    check("Basic Modell", esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, s_zcl_model));
    check("Basic SW-Build", esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, s_zcl_swbuild));
    check("Basic", esp_zb_cluster_list_add_basic_cluster(cl, basic, SERVER));

    /* Identify */
    esp_zb_identify_cluster_cfg_t icfg = { .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE };
    check("Identify", esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(&icfg), SERVER));

    /* Power Configuration: nur Akku-Attribute, beide meldbar (BatteryVoltage ist im
     * ZBOSS-Standard nicht "reportable", daher generisch angelegt) */
    esp_zb_attribute_list_t *pwr = esp_zb_zcl_attr_list_create(PWR_CLUSTER);
    uint8_t acc = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    check("BatteryVoltage", esp_zb_cluster_add_attr(pwr, PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                                    ESP_ZB_ZCL_ATTR_TYPE_U8, acc, &sh.batt_v));
    check("BatteryPercentage", esp_zb_cluster_add_attr(pwr, PWR_CLUSTER,
                                                       ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                                       ESP_ZB_ZCL_ATTR_TYPE_U8, acc, &sh.batt_pct));
    check("PowerConfig", esp_zb_cluster_list_add_power_config_cluster(cl, pwr, SERVER));

    /* Window Covering: Typ 0x08 (Tilt Blind, Lift + Tilt). Mode ausdrücklich 0:
     * der SDK-Default ist "Wartungsmodus" (Bit 2), in dem ein Rollo nicht fährt. */
    esp_zb_window_covering_cluster_cfg_t wcfg = {
        .covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_TILT_BLIND_LIFT_AND_TILT,
        .covering_status = sh.cfg_status,
        .covering_mode = sh.mode,
    };
    esp_zb_attribute_list_t *wc = esp_zb_window_covering_cluster_create(&wcfg);
    wc_attr(wc, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &sh.pos[APP_AXIS_LIFT]);
    wc_attr(wc, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID, &sh.pos[APP_AXIS_TILT]);
    check("WindowCovering", esp_zb_cluster_list_add_window_covering_cluster(cl, wc, SERVER));

#ifdef ROLLO_ZB_TEMPERATURE
    /* NTC (nur melden, nicht abschalten). Bereich -20…+80 °C. */
    esp_zb_temperature_meas_cluster_cfg_t tcfg = {
        .measured_value = sh.temp,
        .min_value = -2000,
        .max_value = 8000,
    };
    check("Temperatur", esp_zb_cluster_list_add_temperature_meas_cluster(cl, esp_zb_temperature_meas_cluster_create(&tcfg), SERVER));
#endif

    esp_zb_ep_list_t *ep = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t epc = {
        .endpoint = EP,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0,
    };
    check("Endpoint 1", esp_zb_ep_list_add_ep(ep, cl, epc));

#ifdef ROLLO_WINDOW_SENSOR
    /* SENSOR: nicht bestückt, deaktiviert – zum Aktivieren ROLLO_WINDOW_SENSOR in config.h.
     * Endpoint 2: IAS Zone (0x0402), Zonentyp Contact Switch, Fenster gekippt = Alarm1.
     * TODO beim Aktivieren: Basic-Cluster für EP 2, Zone Status über
     * esp_zb_zcl_ias_zone_status_change_notif_cmd_req() melden, Enrollment mit Z2M testen. */
    esp_zb_cluster_list_t *cl2 = esp_zb_zcl_cluster_list_create();
    esp_zb_ias_zone_cluster_cfg_t zcfg = {
        .zone_state = 0,
        .zone_type = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH,
        .zone_status = 0,
        .zone_id = 0xff,
    };
    check("IAS Zone", esp_zb_cluster_list_add_ias_zone_cluster(cl2, esp_zb_ias_zone_cluster_create(&zcfg), SERVER));
    esp_zb_endpoint_config_t epc2 = {
        .endpoint = 2,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_IAS_ZONE_ID,
        .app_device_version = 0,
    };
    check("Endpoint 2", esp_zb_ep_list_add_ep(ep, cl2, epc2));
#endif
    return ep;
}

/** Standard-Reporting (gilt, bis Z2M beim Interview eine eigene Konfiguration schreibt) */
static void setup_reporting(uint16_t cluster, uint16_t attr, uint16_t min_s, uint16_t max_s, uint8_t delta)
{
    esp_zb_zcl_reporting_info_t r = { 0 };
    r.direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    r.ep = EP;
    r.cluster_id = cluster;
    r.cluster_role = SERVER;
    r.attr_id = attr;
    r.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    r.u.send_info.min_interval = min_s;
    r.u.send_info.max_interval = max_s;
    r.u.send_info.def_min_interval = min_s;
    r.u.send_info.def_max_interval = max_s;
    r.u.send_info.delta.u8 = delta;
    r.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    check("Reporting", esp_zb_zcl_update_reporting_info(&r));
}

static void zb_task(void *arg)
{
    (void)arg;
    esp_zb_cfg_t cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = ZB_POLL_INTERVAL_MS,   /* = Long-Poll-Intervall (SDK) */
        },
    };
    if (s_sleepy) esp_zb_sleep_enable(true);
    esp_zb_init(&cfg);

    esp_zb_device_register(create_endpoints());
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_identify_notify_handler_register(EP, on_identify);

    setup_reporting(WC_CLUSTER, s_pos_attr_id[APP_AXIS_LIFT], 1, 3600, 1);
    setup_reporting(WC_CLUSTER, s_pos_attr_id[APP_AXIS_TILT], 1, 3600, 1);
    setup_reporting(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, ZB_BATT_REPORT_MIN_S, ZB_BATT_REPORT_S, 1);
    setup_reporting(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, ZB_BATT_REPORT_MIN_S,
                    ZB_BATT_REPORT_S, 2);

    check("Kanalmaske", esp_zb_set_primary_network_channel_set(ZB_CHANNEL_MASK));
    if (s_sleepy) esp_zb_set_rx_on_when_idle(false);   /* Sleepy End Device: Parent puffert, wir pollen */
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_set_node_descriptor_power_source(false);   /* Batterie */
    ESP_LOGI(TAG, "Zigbee %s gestartet: End Device, Poll %d ms, RxOnWhenIdle %s",
             esp_zb_get_version_string(), ZB_POLL_INTERVAL_MS, esp_zb_get_rx_on_when_idle() ? "ja" : "nein (sleepy)");
    /* Erst freigeben, dann die Schattenkopie unter Lock übernehmen: Werte, die die
     * app_task währenddessen setzt, gehen so nicht verloren. */
    s_started = true;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(ZB_LOCK_TIMEOUT_MS))) {
        apply_all();
        esp_zb_lock_release();
    }
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------------- */
/* Öffentliche Funktionen                                                     */

esp_err_t zb_device_init(void)
{
    zcl_str(s_zcl_manuf, sizeof(s_zcl_manuf), ROLLO_ZB_MANUFACTURER);
    zcl_str(s_zcl_model, sizeof(s_zcl_model), ROLLO_ZB_MODEL);
    zcl_str(s_zcl_swbuild, sizeof(s_zcl_swbuild), esp_app_get_description()->version);

    taskENTER_CRITICAL(&s_sh_lock);
    s_sh.pos[0] = s_sh.pos[1] = ZBL_POS_UNKNOWN;
    s_sh.temp = ZBL_TEMP_UNKNOWN;
    s_sh.cfg_status = zbl_config_status(false);
    s_sh.mode = zbl_mode_attr(false, false);
    taskEXIT_CRITICAL(&s_sh_lock);

    esp_zb_platform_config_t pc = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    return esp_zb_platform_config(&pc);
}

esp_err_t zb_device_start(void)
{
    if (xTaskCreate(zb_task, "zb_task", ZB_TASK_STACK, NULL, ZB_TASK_PRIO, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static bool lock(void)
{
    if (!s_started) return false;
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(ZB_LOCK_TIMEOUT_MS))) return true;
    ESP_LOGW(TAG, "Zigbee-Lock nicht erhalten (%d ms)", ZB_LOCK_TIMEOUT_MS);
    return false;
}

void zb_device_set_position(uint8_t axis, uint8_t attr, bool final)
{
    if (axis >= APP_AXIS_COUNT) return;
    taskENTER_CRITICAL(&s_sh_lock);
    s_sh.pos[axis] = attr;
    taskEXIT_CRITICAL(&s_sh_lock);
    if (!lock()) return;
    set_attr(WC_CLUSTER, s_pos_attr_id[axis], &attr);
    uint32_t t = now_ms();
    if (zbl_report_due(final, t, s_rep_last_ms[axis], s_rep_sent[axis])) {
        send_report(WC_CLUSTER, s_pos_attr_id[axis]);
        s_rep_last_ms[axis] = t;
        s_rep_sent[axis] = true;
    }
    esp_zb_lock_release();
}

void zb_device_set_battery(const battery_status_t *b)
{
    uint8_t v = zbl_battery_voltage_attr(b->vbat_mv);
    uint8_t p = zbl_battery_percent_attr(b->percent_x2);
    int16_t t = zbl_temperature_attr(b->ntc == BATT_NTC_OK, b->temp_c);
    taskENTER_CRITICAL(&s_sh_lock);
    bool lockout_changed = s_sh.batt_valid && s_sh.lockout != b->lockout;
    s_sh.batt_v = v;
    s_sh.batt_pct = p;
    s_sh.temp = t;
    s_sh.lockout = b->lockout;
    s_sh.batt_valid = true;
    taskEXIT_CRITICAL(&s_sh_lock);
    if (!lock()) return;
    set_attr(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &v);
    set_attr(PWR_CLUSTER, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &p);
#ifdef ROLLO_ZB_TEMPERATURE
    set_attr(TEMP_CLUSTER, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &t);
#endif
    int64_t now = esp_timer_get_time();
    if (lockout_changed || !s_batt_rep_sent || now - s_batt_rep_us >= (int64_t)ZB_BATT_REPORT_S * 1000000)
        report_battery();
    esp_zb_lock_release();
}

void zb_device_set_status(bool operational, bool calibrating)
{
    uint8_t cs = zbl_config_status(operational);
    uint8_t mode = zbl_mode_attr(calibrating, false);
    taskENTER_CRITICAL(&s_sh_lock);
    s_sh.cfg_status = cs;
    s_sh.mode = mode;
    taskEXIT_CRITICAL(&s_sh_lock);
    if (!lock()) return;
    set_attr(WC_CLUSTER, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID, &cs);
    set_attr(WC_CLUSTER, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID, &mode);
    esp_zb_lock_release();
}

app_net_t zb_device_net_state(void)
{
    return s_net;
}

void zb_device_log_info(void)
{
    if (!lock()) {
        ESP_LOGI(TAG, "Stack nicht gestartet / Lock belegt");
        return;
    }
    if (esp_zb_bdb_dev_joined()) log_network_info();
    else ESP_LOGI(TAG, "nicht im Netz (%s)", esp_zb_bdb_is_factory_new() ? "Werkseinstellung, Pairing" : "Rejoin");
    esp_zb_lock_release();
}

/** Notfallpfad: zb_storage direkt löschen (Lock nicht zu bekommen), dann Neustart */
static void __attribute__((noreturn)) emergency_reset(void)
{
    ESP_LOGE(TAG, "Notfallpfad: zb_storage direkt loeschen und neu starten");
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
    if (p) {
        esp_err_t e = esp_partition_erase_range(p, 0, p->size);
        if (e != ESP_OK) ESP_LOGE(TAG, "zb_storage loeschen: %s", esp_err_to_name(e));
    } else {
        ESP_LOGE(TAG, "Partition zb_storage nicht gefunden");
    }
    vTaskDelay(pdMS_TO_TICKS(100));   /* Log ausgeben */
    esp_restart();
}

void zb_device_factory_reset(void)
{
    if (s_started && esp_zb_lock_acquire(pdMS_TO_TICKS(ZB_LOCK_TIMEOUT_MS))) {
        if (esp_zb_bdb_dev_joined()) {
            /* Leave an den Koordinator (Z2M entfernt das Gerät), danach kommt
             * ESP_ZB_ZDO_SIGNAL_LEAVE (Typ RESET) -> esp_zb_factory_reset() -> Neustart */
            ESP_LOGW(TAG, "Werkseinstellung: Leave senden");
            esp_zb_bdb_reset_via_local_action();
            esp_zb_lock_release();
            /* Aufrufer ist die app_task (Task-WDT 5 s): vor und nach dem Warten füttern
             * (ohne Anmeldung liefert esp_task_wdt_reset() nur einen Fehlercode) */
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(ZB_FACTORY_LEAVE_WAIT_MS));
            esp_task_wdt_reset();
            ESP_LOGW(TAG, "Leave-Signal blieb aus -> Zigbee-Daten loeschen");
            if (esp_zb_lock_acquire(pdMS_TO_TICKS(ZB_LOCK_TIMEOUT_MS))) esp_zb_factory_reset();
        } else {
            esp_zb_factory_reset();   /* löscht zb_storage, startet neu */
        }
    }
    emergency_reset();
}
