/** @file battery.h — 3S battery ADC conversion and state-of-charge estimate. */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>

/* PA4 sees the midpoint of 100k (battery+) / 27k (GND).
 * 12.6 V full charge becomes 2.68 V, safely below the 3.3 V ADC limit. */
#define BATTERY_ADC_REF_MV       3300U
#define BATTERY_ADC_FULL_SCALE   4095U
#define BATTERY_DIVIDER_HIGH_KOHM 100U
#define BATTERY_DIVIDER_LOW_KOHM   27U

/** Convert a 12-bit ADC sample to battery-terminal millivolts. */
uint16_t battery_mv_from_adc(uint16_t raw);

/** Approximate remaining capacity for a resting/lightly-loaded 3S LiPo. */
uint8_t battery_percent_from_mv(uint16_t battery_mv);

/** Read one ADC1/IN4 sample. Implemented by the real-hardware target. */
bool battery_adc_read_raw(uint16_t *raw);

#endif
