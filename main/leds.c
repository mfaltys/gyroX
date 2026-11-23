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

/* One continuous frame: LED_COUNT * 24 data bits (GRB, MSB first) + one long
 * latch. The RMT driver streams payloads larger than the channel's 48-symbol
 * memory block, so the whole rail goes out as a single unbroken WS2812 frame.
 * That matters: with separate per-LED frames, each LED applies its own reset
 * detection to the ~55 us low gap between frames - and clone LEDs have reset
 * thresholds anywhere from ~30 us to 80+ us, so some LEDs desync mid-stream
 * and latch garbage (first LED fine, rest random). One frame + one long latch
 * removes that ambiguity completely. */
static rmt_symbol_word_t s_frame[LED_COUNT * 24 + 1];

static void build_frames(uint8_t r, uint8_t g, uint8_t b)
{
    /* WS2812B wants GRB order */
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    int i = 0;
    for (int led = 0; led < LED_COUNT; led++) {
        for (int bit = 23; bit >= 0; bit--) {
            int on = (grb >> bit) & 1;
            s_frame[i].level0 = 1;
            s_frame[i].duration0 = on ? 6 : 3;   /* T1H 750 ns / T0H 375 ns */
            s_frame[i].level1 = 0;
            s_frame[i].duration1 = on ? 4 : 7;   /* T1L 500 ns / T0L 875 ns */
            i++;
        }
    }
    /* latch: ~300 us of low (2400 ticks) - safe for every WS2812x/SK6812 clone */
    s_frame[i].level0 = 0;
    s_frame[i].duration0 = 2400;
    s_frame[i].level1 = 0;
    s_frame[i].duration1 = 1;
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
        int32_t br = s_cfg.leds_bright;
        if (br < 0) br = 0;
        if (br > 100) br = 100;
        r = (uint8_t)(((int32_t)((s_cfg.leds_color >> 16) & 0xFF) * br) / 100);
        g = (uint8_t)(((int32_t)((s_cfg.leds_color >> 8) & 0xFF) * br) / 100);
        b = (uint8_t)(((int32_t)(s_cfg.leds_color & 0xFF) * br) / 100);
    }
    build_frames(r, g, b);
    if (push_frame() != ESP_OK) {
        ESP_LOGW(TAG, "LED frame transmit failed");
    }
}