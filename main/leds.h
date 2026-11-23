#ifndef GYROX_LEDS_H
#define GYROX_LEDS_H

/* WS2812B rail (LED_COUNT LEDs) on GPIO_LED_DATA, driven with the RMT
 * peripheral — no external components needed. State (on/off + color) comes
 * from s_cfg.leds_*; leds_init() sets up RMT and applies the saved state,
 * leds_apply() re-pushes it after config changes (called from the web API). */
void leds_init(void);
void leds_apply(void);

#endif /* GYROX_LEDS_H */