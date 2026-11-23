#ifndef GYROX_PWM_IN_H
#define GYROX_PWM_IN_H

#include <driver/gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/* ELRS PWM receiver input decoder (GPIO edge timestamps) */
void pwm_in_init(gpio_num_t gpio);

/* Latest decoded steering value: -100 (full left) .. 100 (full right) */
int32_t pwm_in_get_value(void);
/* Last valid pulse width in microseconds */
int32_t pwm_in_get_raw_us(void);
/* True if a valid pulse arrived within PWM_LOST_TIMEOUT_MS */
bool pwm_in_is_valid(void);

#endif /* GYROX_PWM_IN_H */
