#include "mixing.h"
#include "gyro.h"
#include "pwm_in.h"
#include "servo.h"
#include "config.h"

#include <math.h>
#include <inttypes.h>
#include <esp_log.h>

static const char *TAG = "MIXING";

static mixing_state_t s_state = {0};
static float s_target_heading = 0.0f;

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Wrap angle to -180..180 */
static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void mixing_init(void)
{
    s_state.output = 0;
    s_state.rx_valid = false;
    s_state.gyro_ready = false;
    s_target_heading = 0.0f;
    ESP_LOGI(TAG, "Mixer init: mode=%" PRId32 " gain=%" PRId32 " deadband=%" PRId32,
             s_cfg.gyro_mode, s_cfg.gyro_gain, s_cfg.steer_deadband);
}

void mixing_reset_heading(void)
{
    s_target_heading = gyro_get_heading();
    ESP_LOGI(TAG, "Heading-hold target reset to %.1f deg", s_target_heading);
}

void mixing_update(void)
{
    int32_t rx = pwm_in_get_value();
    bool rx_valid = pwm_in_is_valid();

    gyro_reading_t g;
    bool gyro_ok = (gyro_read(&g) == ESP_OK && g.ready);
    s_state.gyro_ready = gyro_ok;

    /* ---- receiver input ---- */
    if (!rx_valid) {
        /* Failsafe: no signal from receiver -> straight */
        rx = 0;
        s_state.rx_valid = false;
    } else {
        s_state.rx_valid = true;
    }

    /* deadband around center */
    if (rx > -s_cfg.steer_deadband && rx < s_cfg.steer_deadband) {
        rx = 0;
    }
    rx += s_cfg.steer_trim;
    if (s_cfg.steer_reverse) rx = -rx;
    rx = clamp_i32(rx, -100, 100);
    s_state.rx = rx;

    /* ---- gyro correction ---- */
    int32_t corr = 0;
    if (gyro_ok) {
        int32_t rate = (int32_t)roundf(g.gz);
        if (s_cfg.gyro_reverse) rate = -rate;
        s_state.rate_z = rate;

        if (s_cfg.gyro_mode == 1) { /* heading hold */
            float err = wrap180(s_target_heading - g.heading);
            corr = (int32_t)roundf(err * ((float)s_cfg.hold_p / 10.0f));
        } else { /* rate mode */
            corr = (int32_t)roundf(((float)rate * (float)s_cfg.gyro_gain) / 1000.0f);
        }
        corr = clamp_i32(corr, -100, 100);
    }
    s_state.correction = corr;

    /* ---- mix ---- */
    int32_t out = clamp_i32(rx + corr, -100, 100);
    s_state.output = out;
    servo_set_value(out);
}

const mixing_state_t *mixing_get_state(void)
{
    return &s_state;
}
