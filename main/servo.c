#include "servo.h"
#include "config.h"

#include <esp_log.h>
#include <driver/ledc.h>

static const char *TAG = "SERVO";

static int32_t s_value = 0;

esp_err_t servo_init(void)
{
    /* LEDC timer: 14-bit resolution at 50 Hz (ESP32-C3 supports up to 14 bits) */
    ledc_timer_config_t timer = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    /* LEDC channel: route output to the servo GPIO */
    ledc_channel_config_t channel = {
        .gpio_num = GPIO_SERVO_OUT,
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    /* Start centered */
    servo_set_value(0);
    ESP_LOGI(TAG, "Servo on GPIO %d (LEDC %d Hz, 14-bit)", (int)GPIO_SERVO_OUT, SERVO_FREQ_HZ);
    return ESP_OK;
}

static int32_t value_to_pulse_us(int32_t value)
{
    if (value > 100) value = 100;
    if (value < -100) value = -100;

    int32_t center = s_cfg.servo_center_us;
    int32_t pmin = s_cfg.servo_min_us;
    int32_t pmax = s_cfg.servo_max_us;
    if (pmin >= center) pmin = center - 1;
    if (pmax <= center) pmax = center + 1;

    int32_t us;
    if (value >= 0) {
        us = center + (int32_t)(((int64_t)value * (pmax - center)) / 100);
    } else {
        us = center - (int32_t)(((int64_t)(-value) * (center - pmin)) / 100);
    }
    if (us < pmin) us = pmin;
    if (us > pmax) us = pmax;
    return us;
}

void servo_set_value(int32_t value)
{
    s_value = value;
    int32_t us = value_to_pulse_us(value);
    servo_set_pulse_us(us);
}

void servo_set_pulse_us(int32_t us)
{
    int32_t pmin = s_cfg.servo_min_us;
    int32_t pmax = s_cfg.servo_max_us;
    if (us < pmin) us = pmin;
    if (us > pmax) us = pmax;

    /* 20 ms period, 14-bit duty (0..16383) */
    uint32_t duty = (uint32_t)(((int64_t)us * 16383) / 20000);
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

int32_t servo_get_value(void) { return s_value; }
