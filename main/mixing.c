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

/* Confirmation tweak: a short scripted servo sweep (+amp, -amp, center over
 * ~300 ms) that overrides the mix - the visual "it did something" cue for the
 * boot button (AP is up / gyro back on). Runs inside the 100 Hz loop, so no
 * extra task and the mix resumes deterministically afterwards. */
#define TWEAK_AMPLITUDE  30   /* servo units, -100..100 */
#define TWEAK_TICKS      30   /* 30 ticks x 10 ms = 300 ms total */
static volatile bool s_tweak_req = false;
static bool s_tweak_active = false;
static int s_tweak_tick = 0;
static bool s_tweak_reset_after = false;

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
    s_state.gyro_on = true;   /* the gyro always starts enabled */
    s_target_heading = 0.0f;
    ESP_LOGI(TAG, "Mixer init: mode=%" PRId32 " gain=%" PRId32 " deadband=%" PRId32,
             s_cfg.gyro_mode, s_cfg.gyro_gain, s_cfg.steer_deadband);
}

void mixing_reset_heading(void)
{
    s_target_heading = gyro_get_heading();
    ESP_LOGI(TAG, "Heading-hold target reset to %.1f deg", s_target_heading);
}

void mixing_set_gyro(bool on)
{
    bool was = s_state.gyro_on;
    s_state.gyro_on = on;
    if (on && !was) {
        /* Off -> on: confirm with the tweak, and re-capture the heading-hold
         * target AFTER the sweep so hold cannot fight it or yank the wheels
         * from a stale target. */
        s_tweak_reset_after = true;
        s_tweak_req = true;
    }
    ESP_LOGI(TAG, "Gyro %s", on ? "ON (correction active)" : "OFF (raw steering passthrough)");
}

void mixing_tweak(bool reset_heading_after)
{
    s_tweak_reset_after = reset_heading_after;
    s_tweak_req = true;
}

void mixing_update(void)
{
    int32_t rx = pwm_in_get_value();
    bool rx_valid = pwm_in_is_valid();

    gyro_reading_t g;
    bool gyro_ok = (gyro_read(&g) == ESP_OK && g.ready);
    s_state.gyro_ready = gyro_ok;

    /* ---- confirmation tweak: scripted sweep overrides the mix ---- */
    if (s_tweak_req && !s_tweak_active) {
        s_tweak_req = false;
        s_tweak_active = true;
        s_tweak_tick = 0;
    }
    if (s_tweak_active) {
        int32_t pos = (s_tweak_tick < 10) ? TWEAK_AMPLITUDE
                    : (s_tweak_tick < 20) ? -TWEAK_AMPLITUDE : 0;
        s_state.output = pos;
        servo_set_value(pos);
        if (++s_tweak_tick >= TWEAK_TICKS) {
            s_tweak_active = false;
            if (s_tweak_reset_after) {
                s_tweak_reset_after = false;
                /* Only meaningful in heading-hold mode: re-capture the target
                 * so hold cannot yank from a stale angle. Rate mode never
                 * reads the heading target, so there is nothing to refresh. */
                if (s_cfg.gyro_mode == 1) {
                    mixing_reset_heading();
                }
            }
        }
        return;
    }

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

        /* gyro_on = user toggle: off means raw steering passthrough */
        if (s_state.gyro_on) {
            if (s_cfg.gyro_mode == 1) { /* heading hold */
                float err = wrap180(s_target_heading - g.heading);
                corr = (int32_t)roundf(err * ((float)s_cfg.hold_p / 10.0f));
            } else { /* rate mode */
                corr = (int32_t)roundf(((float)rate * (float)s_cfg.gyro_gain) / 1000.0f);
            }
            corr = clamp_i32(corr, -100, 100);
        }
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
