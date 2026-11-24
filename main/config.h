#ifndef GYROX_CONFIG_H
#define GYROX_CONFIG_H

#include <driver/gpio.h>
#include <cJSON.h>

/*
 * gyroX — RC car gyro (Seeed XIAO ESP32C3 + MPU-6050)
 *
 * The GPIO pins below are the suggested defaults (see README.md wiring), matched to
 * the XIAO silkscreen. Tuning values are stored in NVS and editable over the web UI;
 * GPIO assignments are compile-time and changed here.
 */

/* -------------------------- GPIOs -------------------------- */
/* Mapped to the XIAO ESP32C3 silkscreen labels (see README wiring table):
 *   D0=GPIO2  D1=GPIO3  D2=GPIO4  D3=GPIO5  D4=GPIO6  D5=GPIO7
 *   D6=GPIO21 D7=GPIO20 D8=GPIO8  D9=GPIO9(BOOT BTN)  D10=GPIO10
 */
#define GPIO_PWM_IN      GPIO_NUM_10   /* XIAO D10 - steering channel from ELRS PWM receiver */
#define GPIO_SERVO_OUT   GPIO_NUM_8    /* XIAO D8  - signal to steering servo                */
#define GPIO_I2C_SDA     GPIO_NUM_7    /* XIAO D5  - MPU-6050 SDA                            */
#define GPIO_I2C_SCL     GPIO_NUM_6    /* XIAO D4  - MPU-6050 SCL                            */
#define GPIO_BOOT_BTN    GPIO_NUM_9    /* XIAO boot button (D9) - hold 3 s after boot for web UI */
#define GPIO_LED_DATA    GPIO_NUM_21   /* XIAO D6  - WS2812B data (3-LED rail), non-strap pin */

/* --------------------- MPU-6050 (I2C) --------------------- */
#define MPU6050_ADDR      0x68         /* AD0 tied to GND (0x69 if AD0 -> VCC)      */
#define MPU6050_I2C_PORT  I2C_NUM_0
#define MPU6050_I2C_HZ    100000       /* 100 kHz - reliable with weak pull-ups     */

/* --------------------------- NVS -------------------------- */
#define NVS_NAMESPACE     "gyrox"

/* ----------------- Servo output (LEDC, 50 Hz) -------------- */
#define SERVO_FREQ_HZ         50
#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL    LEDC_CHANNEL_0
#define SERVO_RESOLUTION_BITS 16

#define DEF_SERVO_MIN_US      500
#define DEF_SERVO_CENTER_US   1500
#define DEF_SERVO_MAX_US      2500

/* ---------------- PWM input (ELRS receiver) --------------- */
#define DEF_PWM_MIN_US        800
#define DEF_PWM_CENTER_US    1500
#define DEF_PWM_MAX_US       2200
#define PWM_LOST_TIMEOUT_MS   600   /* no valid pulse -> failsafe steering */

/* --------------------- Gyro --------------------- */
#define GYRO_CAL_SAMPLES   1500     /* samples at boot while car sits still      */
#define GYRO_CAL_MS        1500     /* time allowed for boot calibration         */
#define DEF_GYRO_DLPF      21       /* DLPF bandwidth Hz: 5 / 10 / 21 / 44       */
#define DEF_GYRO_FS        1000     /* full-scale deg/s: 250 / 500 / 1000 / 2000 */

/* ----------------------- mDNS ----------------------- */
/* Advertise the web UI as gyrox.local (and gyroX.local alias) so a client
 * that has joined the gyroX AP can reach it by name instead of 192.168.4.1.
 * This matters for OTA firmware uploads, which the captive portal blocks. */
#define MDNS_HOSTNAME        "gyrox"      /* gyrox.local */
#define MDNS_HOSTNAME_ALIAS  "gyroX"      /* gyroX.local alias */
#define MDNS_INSTANCE_NAME   "gyroX"

/* ------------------- Web UI (AP mode) ------------------- */
#define WEB_BTN_HOLD_MS   3000      /* hold BOOT button this long after boot  */
#define WEB_BTN_POLL_MS   20        /* button task sampling period            */
#define WEB_AP_SSID       "gyroX"
#define WEB_AP_PASSWORD   ""        /* open AP                                   */
#define WEB_AP_CHANNEL    1
#define WEB_AP_IP         "192.168.4.1"
#define WEB_AP_NETMASK    "255.255.255.0"
#define WEB_AP_GW         "192.168.4.1"
#define WEB_DNS_PORT      53

/* ----------------------- LED rail ------------------------- */
#define LED_COUNT          3        /* WS2812B LEDs on the rail                  */
#define DEF_LEDS_ON        0        /* rail off by default                       */
#define DEF_LEDS_COLOR     0xFF0000 /* packed 0xRRGGBB                           */
#define DEF_LEDS_BRIGHT    100      /* 0..100 percent                            */

/* ------------------- NVS config keys ------------------- */
#define CFG_VERSION          "cfg_ver"        /* int32  */
#define CFG_GYRO_GAIN        "gyro_gain"      /* int32  x10  (correction per 100 deg/s) */
#define CFG_GYRO_MODE        "gyro_mode"      /* int32  0 = rate, 1 = heading hold      */
#define CFG_GYRO_REVERSE     "gyro_reverse"   /* int32  0/1                             */
#define CFG_STEER_REVERSE    "steer_reverse"  /* int32  0/1                             */
#define CFG_STEER_DEADBAND   "steer_deadband" /* int32  0..50 percent                   */
#define CFG_STEER_TRIM       "steer_trim"     /* int32 -100..100                        */
#define CFG_SERVO_MIN_US     "servo_min_us"   /* int32                                  */
#define CFG_SERVO_CENTER_US  "servo_center_us"/* int32                                  */
#define CFG_SERVO_MAX_US     "servo_max_us"   /* int32                                  */
#define CFG_GYRO_Z_OFFSET    "gyro_z_off"    /* float (blob) deg/s                     */
#define CFG_HEADING_HOLD_P   "hold_p"         /* int32  x10                             */
#define CFG_GYRO_DLPF        "gyro_dlpf"      /* int32  Hz: 5/10/21/44                  */
#define CFG_GYRO_FS          "gyro_fs"        /* int32  deg/s: 250/500/1000/2000        */
#define CFG_PROFILE          "profile"        /* int32  0 gen 1 drift 2 bash 3 custom   */
#define CFG_LEDS_ON          "leds_on"        /* int32  0/1                             */
#define CFG_LEDS_COLOR       "leds_color"     /* int32  packed 0xRRGGBB                 */
#define CFG_LEDS_BRIGHT      "leds_bright"    /* int32  0..100 percent                  */
#define CFG_WEB_AP_SSID      "web_ap_ssid"    /* string                                 */

/* ------------------- Compile-time defaults ------------------- */
#define DEF_CFG_VERSION          1
#define DEF_GYRO_GAIN           80   /* 8.0 % correction per 100 deg/s yaw rate */
#define DEF_GYRO_MODE            0   /* 0 = rate, 1 = heading hold              */
#define DEF_GYRO_REVERSE         0
#define DEF_STEER_REVERSE        0
#define DEF_STEER_DEADBAND       8
#define DEF_STEER_TRIM           0
#define DEF_HEADING_HOLD_P      40   /* 4.0 deg/s per degree heading error      */
#define DEF_PROFILE              0   /* 0 general, 1 drift, 2 bashing, 3 custom  */

/* ------------------------------------------------------------------ */
/* Runtime configuration (loaded from / saved to NVS)                 */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t version;
    int32_t gyro_gain;       /* x10 */
    int32_t gyro_mode;       /* 0 = rate, 1 = heading hold */
    int32_t gyro_reverse;
    int32_t steer_reverse;
    int32_t steer_deadband;  /* 0..50 */
    int32_t steer_trim;      /* -100..100 */
    int32_t servo_min_us;
    int32_t servo_center_us;
    int32_t servo_max_us;
    float   gyro_z_offset;   /* deg/s */
    int32_t hold_p;          /* x10 */
    int32_t gyro_dlpf;       /* Hz: 5/10/21/44 */
    int32_t gyro_fs;         /* deg/s: 250/500/1000/2000 */
    int32_t profile;         /* 0 general, 1 drift, 2 bashing, 3 custom */
    int32_t leds_on;         /* 0/1 */
    uint32_t leds_color;     /* packed 0xRRGGBB */
    int32_t leds_bright;     /* 0..100 percent */
    char    web_ap_ssid[32];
} gyrox_config_t;

extern gyrox_config_t s_cfg;

/* config.c */
esp_err_t config_load(void);     /* load from NVS, write defaults on first boot */
esp_err_t config_save(void);     /* persist current s_cfg to NVS */
esp_err_t config_set_from_json(cJSON *root);   /* apply + save fields from web UI */
void config_apply_defaults(void);

#endif /* GYROX_CONFIG_H */
