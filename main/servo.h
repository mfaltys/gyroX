#ifndef DRIFT_SERVO_H
#define DRIFT_SERVO_H

#include <stdint.h>

#include <esp_err.h>

/* Steering servo output via LEDC (50 Hz, 500-2500 us) */
esp_err_t servo_init(void);

/* Set position: -100 (min) .. 0 (center) .. 100 (max), clamped to servo min/max */
void servo_set_value(int32_t value);
/* Raw pulse width in microseconds, clamped to configured min/max */
void servo_set_pulse_us(int32_t us);
/* Last commanded value -100..100 */
int32_t servo_get_value(void);

#endif /* DRIFT_SERVO_H */
