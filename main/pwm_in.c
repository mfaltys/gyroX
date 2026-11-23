#include "pwm_in.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

static const char *TAG = "PWM_IN";

static gpio_num_t s_gpio;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t s_raw_us = 0;
static volatile int32_t s_value = 0;
static volatile bool s_valid = false;
static volatile int64_t s_last_valid_us = 0;
static volatile int64_t s_rise_us = 0;

/* Map pulse width to -100..100 around the configured center */
static int32_t map_pulse(int32_t us)
{
    int32_t center = DEF_PWM_CENTER_US;
    int32_t pmin = DEF_PWM_MIN_US;
    int32_t pmax = DEF_PWM_MAX_US;

    if (us > center) {
        if (us > pmax) us = pmax;
        return (int32_t)(((int64_t)(us - center) * 100) / (pmax - center));
    }
    if (us < pmin) us = pmin;
    return -(int32_t)(((int64_t)(center - us) * 100) / (center - pmin));
}

static void IRAM_ATTR pwm_in_isr(void *arg)
{
    (void)arg;
    int64_t t = esp_timer_get_time();
    uint32_t level = gpio_get_level(s_gpio);

    portENTER_CRITICAL_ISR(&s_lock);
    if (level == 1) {
        s_rise_us = t;
    } else {
        if (s_rise_us != 0) {
            int64_t width = t - s_rise_us;
            s_rise_us = 0;
            if (width >= 400 && width <= 3000) {
                /* EMA smoothing: raw_us = 0.8*raw + 0.2*width */
                s_raw_us = (int32_t)(((int64_t)s_raw_us * 8 + (int64_t)width * 2) / 10);
                s_value = map_pulse(s_raw_us);
                if (s_value > 100) s_value = 100;
                if (s_value < -100) s_value = -100;
                s_valid = true;
                s_last_valid_us = t;
            }
        }
    }
    portEXIT_CRITICAL_ISR(&s_lock);
}

void pwm_in_init(gpio_num_t gpio)
{
    s_gpio = gpio;
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(gpio, pwm_in_isr, NULL);
    ESP_LOGI(TAG, "PWM input on GPIO %d", (int)gpio);
}

int32_t pwm_in_get_value(void)
{
    int32_t v;
    portENTER_CRITICAL(&s_lock);
    v = s_value;
    portEXIT_CRITICAL(&s_lock);
    return v;
}

int32_t pwm_in_get_raw_us(void)
{
    int32_t u;
    portENTER_CRITICAL(&s_lock);
    u = s_raw_us;
    portEXIT_CRITICAL(&s_lock);
    return u;
}

bool pwm_in_is_valid(void)
{
    bool valid;
    int64_t last;
    portENTER_CRITICAL(&s_lock);
    valid = s_valid;
    last = s_last_valid_us;
    portEXIT_CRITICAL(&s_lock);

    if (!valid) return false;
    return (esp_timer_get_time() - last) < ((int64_t)PWM_LOST_TIMEOUT_MS * 1000LL);
}
