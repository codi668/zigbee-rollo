/*
 * Encoder per PCNT, Basis hwtest/components/encoder (auf der Hardware erprobt:
 * Quadratur x4, Glitch-Filter 3 us, accum_count mit Watchpoints).
 */
#include "encoder.h"
#include "board_pins.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "encoder";

static const int s_pin_a[ENCODER_COUNT] = { PIN_ENC1_A, PIN_ENC2_A };
static const int s_pin_b[ENCODER_COUNT] = { PIN_ENC1_B, PIN_ENC2_B };
static const int s_ppr_x1[ENCODER_COUNT] = { ENC1_PPR_X1, ENC2_PPR_X1 };
static const int s_invert[ENCODER_COUNT] = { ENC1_INVERT, ENC2_INVERT };

static pcnt_unit_handle_t s_unit[ENCODER_COUNT];
static int32_t s_offset[ENCODER_COUNT];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t unit_init(int i)
{
    gpio_config_t g = {
        .pin_bit_mask = (1ULL << s_pin_a[i]) | (1ULL << s_pin_b[i]),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   /* externe Pull-ups nicht bestückt */
    };
    esp_err_t e = gpio_config(&g);
    if (e != ESP_OK) return e;

    pcnt_unit_config_t ucfg = {
        .low_limit = -ENC_PCNT_LIMIT,
        .high_limit = ENC_PCNT_LIMIT,
        .flags.accum_count = true,
    };
    if ((e = pcnt_new_unit(&ucfg, &s_unit[i])) != ESP_OK) return e;

    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = ENC_GLITCH_NS };
    if ((e = pcnt_unit_set_glitch_filter(s_unit[i], &fcfg)) != ESP_OK) return e;

    /* Quadratur x4: je Kanal beide Flanken, Pegel des anderen Kanals gibt die Richtung */
    pcnt_chan_config_t ca = { .edge_gpio_num = s_pin_a[i], .level_gpio_num = s_pin_b[i] };
    pcnt_chan_config_t cb = { .edge_gpio_num = s_pin_b[i], .level_gpio_num = s_pin_a[i] };
    pcnt_channel_handle_t cha, chb;
    if ((e = pcnt_new_channel(s_unit[i], &ca, &cha)) != ESP_OK) return e;
    if ((e = pcnt_new_channel(s_unit[i], &cb, &chb)) != ESP_OK) return e;
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Watchpoints an den Grenzen: erst damit summiert der Treiber Überläufe (accum_count) */
    if ((e = pcnt_unit_add_watch_point(s_unit[i], ENC_PCNT_LIMIT)) != ESP_OK) return e;
    if ((e = pcnt_unit_add_watch_point(s_unit[i], -ENC_PCNT_LIMIT)) != ESP_OK) return e;
    if ((e = pcnt_unit_enable(s_unit[i])) != ESP_OK) return e;
    if ((e = pcnt_unit_clear_count(s_unit[i])) != ESP_OK) return e;
    return pcnt_unit_start(s_unit[i]);
}

esp_err_t encoder_init(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++) {
        esp_err_t e = unit_init(i);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "Encoder %d: PCNT-Init fehlgeschlagen (%s)", i + 1, esp_err_to_name(e));
            return e;
        }
        s_offset[i] = 0;
    }
    if (ENC1_PPR_X1 == 0)
        ESP_LOGW(TAG, "Encoder 1: Impulse/Umdrehung unbekannt (ENC1_PPR_X1 = 0, TODO) - nur Zaehlschritte");
    ESP_LOGI(TAG, "bereit (x4, Glitch-Filter %d ns)", ENC_GLITCH_NS);
    return ESP_OK;
}

/* Rohwert mit Vorzeichenkorrektur */
static int32_t raw(encoder_id_t id)
{
    int c = 0;
    if (s_unit[id]) pcnt_unit_get_count(s_unit[id], &c);
    return s_invert[id] ? -(int32_t)c : (int32_t)c;
}

int32_t encoder_get(encoder_id_t id)
{
    if (id >= ENCODER_COUNT) return 0;
    int32_t r = raw(id);
    taskENTER_CRITICAL(&s_lock);
    int32_t pos = r + s_offset[id];
    taskEXIT_CRITICAL(&s_lock);
    return pos;
}

void encoder_set(encoder_id_t id, int32_t pos)
{
    if (id >= ENCODER_COUNT) return;
    int32_t r = raw(id);
    taskENTER_CRITICAL(&s_lock);
    s_offset[id] = pos - r;
    taskEXIT_CRITICAL(&s_lock);
}

int32_t encoder_counts_per_rev(encoder_id_t id)
{
    return id < ENCODER_COUNT ? (int32_t)s_ppr_x1[id] * 4 : 0;
}
