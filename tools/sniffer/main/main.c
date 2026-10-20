/*
 * zb_sniffer: 802.15.4-Sniffer auf einem zweiten ESP32-H2.
 *
 * Empfängt im Promiscuous-Modus alle Frames auf einem Kanal (Standard 18, per
 * -DSNIFF_CHANNEL=NN beim Bauen änderbar) und gibt sie als Textzeilen über die USB-Konsole aus:
 *
 *     FR <Zeit_us> <RSSI> <LQI> <PSDU als Hex, ohne FCS>
 *
 * tools/zb-sniffer.py wandelt die Zeilen in eine pcap-Datei (LINKTYPE_IEEE802_15_4_NOFCS, 230) um,
 * die Wireshark liest. Die Hardware verwirft Frames mit falscher FCS; bei empfangenen Frames steht an
 * der FCS-Stelle RSSI + LQI (siehe esp_ieee802154.h).
 */
#include <stdio.h>
#include <string.h>
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#ifndef SNIFF_CHANNEL
#define SNIFF_CHANNEL 18
#endif

#define RING_N     64
#define PSDU_MAX   127

typedef struct {
    uint64_t t_us;
    int8_t   rssi;
    uint8_t  lqi;
    uint8_t  len;                 /* PSDU-Länge ohne FCS */
    uint8_t  data[PSDU_MAX];
} frame_t;

static frame_t s_ring[RING_N];
static volatile uint32_t s_head;  /* nur ISR/Radio-Callback schreibt */
static volatile uint32_t s_tail;  /* nur print_task schreibt */
static volatile uint32_t s_dropped;

/* Aufruf aus dem Funktreiber (ISR-nah): nur kopieren, nichts ausgeben */
void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info)
{
    uint32_t h = s_head;
    if (h - s_tail >= RING_N) {
        s_dropped++;
    } else if (frame[0] > 2 && frame[0] <= PSDU_MAX + 2) {
        frame_t *f = &s_ring[h % RING_N];
        f->t_us = esp_timer_get_time();
        f->len = frame[0] - 2;                  /* letzte 2 Bytes: RSSI, LQI statt FCS */
        f->rssi = (int8_t)frame[frame[0] - 1];
        f->lqi = frame[frame[0]];
        memcpy(f->data, &frame[1], f->len);
        s_head = h + 1;
    }
    esp_ieee802154_receive_handle_done(frame);
}

static void print_task(void *arg)
{
    (void)arg;
    static char line[40 + PSDU_MAX * 2];
    uint32_t last_drop = 0;
    for (;;) {
        if (s_tail == s_head) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        const frame_t *f = &s_ring[s_tail % RING_N];
        int n = snprintf(line, sizeof(line), "FR %llu %d %u ", (unsigned long long)f->t_us, f->rssi, f->lqi);
        for (int i = 0; i < f->len; i++) n += snprintf(line + n, sizeof(line) - n, "%02x", f->data[i]);
        line[n++] = '\n';
        fwrite(line, 1, n, stdout);
        s_tail++;
        if (s_dropped != last_drop) {
            printf("DROP %lu\n", (unsigned long)s_dropped);
            last_drop = s_dropped;
        }
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_ieee802154_enable();
    esp_ieee802154_set_channel(SNIFF_CHANNEL);
    esp_ieee802154_set_promiscuous(true);
    esp_ieee802154_set_rx_when_idle(true);
    esp_ieee802154_receive();
    printf("SNIFFER bereit, Kanal %d\n", SNIFF_CHANNEL);
    xTaskCreate(print_task, "print", 4096, NULL, 5, NULL);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        printf("SNIFFER Kanal %d\n", SNIFF_CHANNEL);   /* Lebenszeichen, damit das Skript Verbindungsabbrüche merkt */
    }
}
