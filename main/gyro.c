#include "gyro.h"
#include "config.h"

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <driver/i2c.h>

static const char *TAG = "GYRO";

/* MPU-6050 registers */
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A   /* DLPF */
#define REG_GYRO_CONFIG 0x1B   /* +/- 250 deg/s */
#define REG_ACCEL_CONFIG 0x1C  /* +/- 2 g */
#define REG_ACCEL_XOUT_H 0x3B

/* Sensitivity: +/- 250 deg/s -> 131 LSB per deg/s; +/- 2 g -> 16384 LSB per g */
#define GYRO_LSB_PER_DEG 131.0f
#define ACCEL_LSB_PER_G 16384.0f

static i2c_port_t s_i2c_port = MPU6050_I2C_PORT;
static uint8_t s_addr = MPU6050_ADDR;   /* runtime-selected (0x68 or 0x69) */
static bool s_gyro_present = false;
static float s_gx_off = 0.0f, s_gy_off = 0.0f, s_gz_off = 0.0f;
static float s_heading = 0.0f;
static int64_t s_last_read_us = 0;

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read register(s) from an explicit address (used for probing before s_addr is chosen) */
static esp_err_t i2c_read_regs_addr(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Quick ACK probe of a single 7-bit address */
static bool i2c_probe(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

/* Scan the bus and log every ACKing address - invaluable when wiring is suspect */
static void i2c_bus_scan(void)
{
    ESP_LOGW(TAG, "Scanning I2C bus (SDA=GPIO%d SCL=GPIO%d)...",
             (int)GPIO_I2C_SDA, (int)GPIO_I2C_SCL);
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_probe(a)) {
            ESP_LOGW(TAG, "  device ACKs at 0x%02X", a);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "  no I2C devices found - check SDA/SCL swap, pull-ups, power, and AD0");
    }
}

esp_err_t gyro_init(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = MPU6050_I2C_HZ },
    };
    ESP_ERROR_CHECK(i2c_param_config(MPU6050_I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(MPU6050_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* Probe both possible addresses (AD0 low -> 0x68, AD0 high -> 0x69) with retries,
     * so a slow power-up on the module doesn't cause a false "not found". */
    const uint8_t addrs[2] = { 0x68, 0x69 };
    for (int attempt = 0; attempt < 5 && !s_gyro_present; attempt++) {
        for (int i = 0; i < 2 && !s_gyro_present; i++) {
            uint8_t who = 0;
            if (i2c_read_regs_addr(addrs[i], REG_WHO_AM_I, &who, 1) == ESP_OK && who != 0x00 && who != 0xFF) {
                s_addr = addrs[i];
                s_gyro_present = true;
                if (who != 0x68) {
                    ESP_LOGW(TAG, "WHO_AM_I=0x%02X at 0x%02X (not a classic MPU-6050; MPU-6500/9250 clones work the same)", who, addrs[i]);
                }
            }
        }
        if (!s_gyro_present) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!s_gyro_present) {
        ESP_LOGE(TAG, "MPU-6050 not found at 0x68/0x69");
        i2c_bus_scan();
        return ESP_ERR_NOT_FOUND;
    }

    i2c_write_reg(REG_PWR_MGMT_1, 0x00);           /* wake up, internal 8 MHz clock */
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_reg(REG_CONFIG, 0x06);               /* DLPF ~5 Hz */
    i2c_write_reg(REG_GYRO_CONFIG, 0x00);          /* +/- 250 deg/s */
    i2c_write_reg(REG_ACCEL_CONFIG, 0x00);         /* +/- 2 g */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Restore the zero-rate offset saved in NVS (config_load() already ran).
     * Without this the offset was only ever live in the session that
     * calibrated it - every reboot ran with uncorrected gyro bias. */
    s_gz_off = s_cfg.gyro_z_offset;

    s_last_read_us = esp_timer_get_time();
    ESP_LOGI(TAG, "MPU-6050 found at 0x%02X (restored offset z=%.3f deg/s)", s_addr, s_gz_off);
    return ESP_OK;
}

static int16_t s16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

esp_err_t gyro_read(gyro_reading_t *out)
{
    if (!s_gyro_present) {
        memset(out, 0, sizeof(*out));
        out->ready = false;
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[14];
    if (i2c_read_regs(REG_ACCEL_XOUT_H, buf, 14) != ESP_OK) {
        out->ready = false;
        return ESP_FAIL;
    }

    int64_t now = esp_timer_get_time();
    float dt = (now - s_last_read_us) / 1e6f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
    s_last_read_us = now;

    out->ax = s16(buf + 0) / ACCEL_LSB_PER_G;
    out->ay = s16(buf + 2) / ACCEL_LSB_PER_G;
    out->az = s16(buf + 4) / ACCEL_LSB_PER_G;
    out->temp_c = s16(buf + 6) / 340.0f + 36.53f;

    out->gx = s16(buf + 8) / GYRO_LSB_PER_DEG - s_gx_off;
    out->gy = s16(buf + 10) / GYRO_LSB_PER_DEG - s_gy_off;
    out->gz = s16(buf + 12) / GYRO_LSB_PER_DEG - s_gz_off;
    out->rate_z = out->gz;

    /* Integrate yaw heading from gyro z (car is assumed flat on the ground) */
    s_heading += out->gz * dt;
    while (s_heading > 180.0f) s_heading -= 360.0f;
    while (s_heading < -180.0f) s_heading += 360.0f;
    out->heading = s_heading;
    out->ready = true;
    return ESP_OK;
}

esp_err_t gyro_calibrate(int samples)
{
    if (!s_gyro_present) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Gyro calibration: keep the car still (%d samples)", samples);

    double gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < samples; i++) {
        uint8_t buf[14];
        if (i2c_read_regs(REG_ACCEL_XOUT_H, buf, 14) == ESP_OK) {
            gx += s16(buf + 8) / GYRO_LSB_PER_DEG;
            gy += s16(buf + 10) / GYRO_LSB_PER_DEG;
            gz += s16(buf + 12) / GYRO_LSB_PER_DEG;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    s_gx_off = (float)(gx / samples);
    s_gy_off = (float)(gy / samples);
    s_gz_off = (float)(gz / samples);
    s_cfg.gyro_z_offset = s_gz_off;

    ESP_LOGI(TAG, "Gyro calibrated: off x=%.3f y=%.3f z=%.3f deg/s",
             s_gx_off, s_gy_off, s_gz_off);
    return config_save();
}

void gyro_reset_heading(float heading)
{
    s_heading = heading;
}

float gyro_get_heading(void) { return s_heading; }
float gyro_get_rate_z(void)
{
    gyro_reading_t r;
    if (gyro_read(&r) == ESP_OK) {
        return r.rate_z;
    }
    return 0.0f;
}
