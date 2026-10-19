/*
 * ui: Taste per GPIO-Interrupt + Polling während des Drückens, LED-Muster.
 * Basis: hwtest/components/{button,led} (Pegel, Pull-up, aktiv LOW erprobt).
 */
#include "ui.h"
#include "btn_logic.h"
#include "board_pins.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

static TaskHandle_t s_task;
static ui_event_cb_t s_cb;
static void *s_cb_ctx;
static volatile led_pattern_t s_pattern = LED_PAT_OFF;
static volatile bool s_pressed;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_pattern_t0;      /* unter s_lock */
static bool s_pattern_changed;     /* unter s_lock */

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static bool btn_raw(void) { return gpio_get_level(PIN_BOOT_BTN) == BTN_ACTIVE_LEVEL; }
static void led_out(bool on) { gpio_set_level(PIN_LED, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL); }

static void IRAM_ATTR btn_isr(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    if (s_task) vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static void emit(btn_event_t be)
{
    ui_event_t e;
    switch (be) {
    case BTN_EV_SHORT:     e = UI_EVT_BTN_SHORT; break;
    case BTN_EV_LONG_HINT: e = UI_EVT_BTN_LONG_HINT; break;
    case BTN_EV_LONG:      e = UI_EVT_BTN_LONG; break;
    case BTN_EV_VERY_LONG: e = UI_EVT_BTN_VERY_LONG; break;
    default: return;
    }
    ESP_LOGI(TAG, "Taste: %s", ui_event_str(e));
    if (s_cb) s_cb(e, s_cb_ctx);
}

static void ui_task(void *arg)
{
    (void)arg;
    btn_logic_t btn;
    btn_logic_init(&btn, now_ms(), btn_raw());
    s_pressed = btn.stable;
    for (;;) {
        uint32_t t = now_ms();
        emit(btn_logic_update(&btn, t, btn_raw()));
        s_pressed = btn.stable && btn.armed;

        taskENTER_CRITICAL(&s_lock);
        led_pattern_t p = s_pattern;
        if (s_pattern_changed) {
            s_pattern_changed = false;
            s_pattern_t0 = t;
        }
        uint32_t pt = t - s_pattern_t0;
        taskEXIT_CRITICAL(&s_lock);
        led_out(led_pattern_level(p, pt));

        /* Wartezeit: kürzester benötigter Takt, sonst bis zum nächsten Interrupt */
        TickType_t wait = portMAX_DELAY;
        if (led_pattern_period(p)) wait = pdMS_TO_TICKS(LED_TICK_MS);
        if (btn_logic_busy(&btn)) wait = pdMS_TO_TICKS(BTN_POLL_MS);
        ulTaskNotifyTake(pdTRUE, wait);
    }
}

esp_err_t ui_init(ui_event_cb_t cb, void *ctx)
{
    if (s_task) return ESP_ERR_INVALID_STATE;
    s_cb = cb;
    s_cb_ctx = ctx;

    /* LED: Pegel "aus" vor dem Umschalten auf Ausgang setzen (kein Aufblitzen) */
    led_out(false);
    gpio_config_t led = { .pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT };
    esp_err_t e = gpio_config(&led);
    if (e != ESP_OK) return e;
    led_out(false);

    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    /* extern 10k vorhanden, intern schadet nicht (wie hwtest) */
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    if ((e = gpio_config(&btn)) != ESP_OK) return e;
    e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;   /* INVALID_STATE = schon installiert */

    if (xTaskCreate(ui_task, "ui_task", UI_TASK_STACK, NULL, UI_TASK_PRIO, &s_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    if ((e = gpio_isr_handler_add(PIN_BOOT_BTN, btn_isr, NULL)) != ESP_OK) return e;
    ESP_LOGI(TAG, "bereit (Taste GPIO%d, LED GPIO%d)", PIN_BOOT_BTN, PIN_LED);
    return ESP_OK;
}

void ui_set_led(led_pattern_t p)
{
    if ((unsigned)p >= LED_PAT_COUNT) p = LED_PAT_OFF;
    bool changed = false;
    taskENTER_CRITICAL(&s_lock);
    if (p != s_pattern) {
        s_pattern = p;
        s_pattern_changed = true;
        changed = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    if (changed && s_task) xTaskNotifyGive(s_task);
}

led_pattern_t ui_get_led(void)
{
    return s_pattern;
}

bool ui_button_pressed(void)
{
    return s_pressed;
}

const char *ui_event_str(ui_event_t e)
{
    switch (e) {
    case UI_EVT_BTN_SHORT:     return "kurz";
    case UI_EVT_BTN_LONG_HINT: return "3 s erreicht";
    case UI_EVT_BTN_LONG:      return "lang (3 s)";
    case UI_EVT_BTN_VERY_LONG: return "sehr lang (10 s)";
    default:                   return "?";
    }
}
