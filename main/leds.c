#include "leds.h"
#include "config.h"

#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/rmt_tx.h>

static const char *TAG = "LEDS";

/* 8 MHz RMT clock -> 125 ns ticks; a WS2812B bit period is 1.25 us = 10 ticks */
#define LED_RMT_HZ 8000000

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_copy_enc = NULL;

/* One LED frame: 24 data bits (GRB, MSB first) + a >=50 us low latch symbol.
 * Fits in one C3 RMT memory block (48 symbols), so no custom encoder needed. */
static rmt_symbol_word_t s_frame[24 + 1];

static void build_frame(uint8_t r, uint8_t g, uint8_t b)
{
    /* WS2812B wants GRB order */
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    int i = 0;
    for (int bit = 23; bit >= 0; bit--) {
        int on = (grb >> bit) & 1;
        s_frame[i].level0 = 1;
        s_frame[i].duration0 = on ? 6 : 3;   /* T1H 750 ns / T0H 375 ns */
        s_frame[i].level1 = 0;
        s_frame[i].duration1 = on ? 4 : 7;   /* T1L 500 ns / T0L 875 ns */
        i++;
    }
    /* latch: >= 50 us of low level (400 ticks) */
    s_frame[24].level0 = 0;
    s_frame[24].duration0 = 400;
    s_frame[24].level1 = 0;
    s_frame[24].duration1 = 1;
}

static esp_err_t push_frame(void)
{
    rmt_transmit_config_t tx = {
        .loop_count = 0,
        .flags.eot_level = 0,   /* end the frame low */
    };
    esp_err_t err = rmt_transmit(s_chan, s_copy_enc, s_frame, sizeof(s_frame), &tx);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(s_chan, -1);
    }
    return err;
}

void leds_init(void)
{
    if (s_chan) {
        return;
    }
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = GPIO_LED_DATA,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_HZ,
        .mem_block_symbols = 48,      /* SOC_RMT_MEM_WORDS_PER_CHANNEL on esp32c3 */
        .trans_queue_depth = 2,
    };
    if (rmt_new_tx_channel(&tx_cfg, &s_chan) != ESP_OK) {
        ESP_LOGE(TAG, "RMT tx channel alloc failed - LED rail disabled");
        s_chan = NULL;
        return;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    esp_err_t err = rmt_new_copy_encoder(&copy_cfg, &s_copy_enc);
    if (err == ESP_OK) {
        err = rmt_enable(s_chan);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED rail init failed (%s) - disabled", esp_err_to_name(err));
        rmt_del_channel(s_chan);
        s_chan = NULL;
        return;
    }

    leds_apply();
    ESP_LOGI(TAG, "LED rail: %d WS2812B on GPIO %d", LED_COUNT, (int)GPIO_LED_DATA);
}

void leds_apply(void)
{
    if (!s_chan) {
        return;
    }
    uint8_t r = 0, g = 0, b = 0;
    if (s_cfg.leds_on) {
        r = (s_cfg.leds_color >> 16) & 0xFF;
        g = (s_cfg.leds_color >> 8) & 0xFF;
        b = s_cfg.leds_color & 0xFF;
    }
    for (int i = 0; i < LED_COUNT; i++) {
        build_frame(r, g, b);
        if (push_frame() != ESP_OK) {
            ESP_LOGW(TAG, "LED frame transmit failed");
            return;
        }
    }
}