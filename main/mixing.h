#ifndef GYROX_MIXING_H
#define GYROX_MIXING_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef struct {
    int32_t rx;           /* receiver input, -100..100 */
    int32_t rate_z;       /* gyro yaw rate, deg/s */
    float heading;        /* deg */
    int32_t correction;   /* gyro steering correction, -100..100 */
    int32_t output;       /* final servo value, -100..100 */
    bool rx_valid;        /* receiver signal present */
    bool gyro_ready;      /* MPU-6050 responding */
    bool gyro_on;         /* correction enabled - boot button tap toggles it */
} mixing_state_t;

void mixing_init(void);            /* uses global s_cfg from config.h */
void mixing_update(void);          /* call ~100 Hz */
const mixing_state_t *mixing_get_state(void);
void mixing_reset_heading(void);  /* heading-hold: capture current heading as target */
void mixing_set_gyro(bool on);    /* enable/disable correction. Re-enabling plays
                                   * the confirmation tweak, then re-captures the
                                   * heading-hold target so it cannot yank. */
void mixing_tweak(bool reset_heading_after);  /* brief servo twitch: visual cue
                                   * (AP-up "release the button", gyro re-enable). */

#endif /* GYROX_MIXING_H */
