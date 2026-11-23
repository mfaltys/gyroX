/* esp32-drift — RC car gyro for Seeed XIAO ESP32C3 + MPU-6050
 *
 * Sits between an ELRS PWM receiver and the steering servo:
 *   RX PWM -> gyro correction -> servo PWM
 *
 * Boot button (GPIO 9 on XIAO ESP32C3) - hold ~3 s AFTER power-up:
 *   Held 3 s after boot -> Wi-Fi AP + web UI for configuring gyro settings
 *   Not held            -> drive mode, no Wi-Fi at all
 *   The AP is gone again after a power cycle unless the button is held again.
 *
 * GPIO 9 is a strapping pin: holding it AT power-up/reset puts the chip into
 * ROM download mode (that is the flash gesture), so the button can only be
 * sampled once the app is running.
 */

#include "config.h"
#include "gyro.h"
#include "pwm_in.h"
#include "servo.h"
#include "mixing.h"
#include "web.h"
#include "leds.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "DRIFT";
static volatile bool s_web_started = false;

/* Web UI entry: the XIAO boot button pulls GPIO 9 to GND. GPIO 9 is a
 * strapping pin (low at reset = ROM download mode), so it is only sampled
 * AFTER the app is running: a continuous hold of WEB_BTN_HOLD_MS starts the
 * AP + web UI. Drive mode is never delayed - the mixer keeps running while
 * the button is watched, and the hold may happen at any time after boot. */
static void button_task(void *arg)
{
    (void)arg;
    uint32_t held_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WEB_BTN_POLL_MS));
        held_ms = (gpio_get_level(GPIO_BOOT_BTN) == 0)
                      ? held_ms + WEB_BTN_POLL_MS
                      : 0;

        if (held_ms >= WEB_BTN_HOLD_MS && !s_web_started) {
            s_web_started = true;
            ESP_LOGI(TAG, "Boot button held %d ms - starting web UI", (int)WEB_BTN_HOLD_MS);
            web_start();
            ESP_LOGI(TAG, "Web UI mode active");
            vTaskDelete(NULL);   /* one-shot: AP stays up until power cycle */
        }
    }
}

static void mixing_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10)); /* 100 Hz control loop */
        mixing_update();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "esp32-drift starting (ESP-IDF target: esp32c3)");

    config_load();

    /* Gyro: init + calibrate if we have no saved offset (car must be still).
     * If the MPU-6050 is missing/unwired we keep running without gyro
     * correction instead of crashing - servo still follows the receiver. */
    if (gyro_init() == ESP_OK) {
        if (s_cfg.gyro_z_offset == 0.0f) {
            ESP_LOGW(TAG, "No gyro offset in NVS - calibrating now, keep the car still!");
            gyro_calibrate(GYRO_CAL_SAMPLES);
        }
    } else {
        ESP_LOGE(TAG, "MPU-6050 not found - running WITHOUT gyro correction");
        ESP_LOGE(TAG, "Check wiring: SDA=XIAO D5 (GPIO%d), SCL=XIAO D4 (GPIO%d), AD0->GND, VCC, GND",
                 (int)GPIO_I2C_SDA, (int)GPIO_I2C_SCL);
    }

    pwm_in_init(GPIO_PWM_IN);
    servo_init();
    mixing_init();

    leds_init();   /* WS2812B rail: sets up RMT, applies saved on/off + color */

    xTaskCreate(mixing_task, "mixing", 3072, NULL, 6, NULL);

    /* Web UI: hold the boot button ~3 s AFTER power-up. GPIO 9 is a strapping
     * pin (low at reset = ROM download mode), so it cannot be sampled at
     * reset; the button task watches it from here on. Drive mode is never
     * delayed, and the hold may happen at any time after boot. */
    gpio_set_direction(GPIO_BOOT_BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BOOT_BTN, GPIO_PULLUP_ONLY);
    xTaskCreate(button_task, "btn", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "Hold boot button %d s after power-up for the web UI",
             (int)(WEB_BTN_HOLD_MS / 1000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
