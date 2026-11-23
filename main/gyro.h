#ifndef GYROX_GYRO_H
#define GYROX_GYRO_H

#include <driver/i2c.h>
#include <esp_err.h>
#include <stdbool.h>

/* MPU-6050 fused output */
typedef struct {
    float gx, gy, gz;   /* deg/s, zero-rate offset removed */
    float ax, ay, az;   /* g */
    float temp_c;
    float heading;      /* deg, integrated from gyro z */
    float rate_z;       /* deg/s */
    bool ready;         /* true once MPU-6050 responded */
} gyro_reading_t;

esp_err_t gyro_init(void);                  /* wake up MPU-6050, configure DLPF + range */
void gyro_apply_sensor_config(void);        /* (re)write DLPF + full-scale from s_cfg */
esp_err_t gyro_read(gyro_reading_t *out);   /* read + filter + integrate heading */
esp_err_t gyro_calibrate(int samples);      /* keep car still; stores zero-rate offset */
void gyro_reset_heading(float heading);
float gyro_get_heading(void);
float gyro_get_rate_z(void);

#endif /* GYROX_GYRO_H */
