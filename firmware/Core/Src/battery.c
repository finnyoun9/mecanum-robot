#include "battery.h"

#include <stddef.h>

typedef struct {
    uint16_t mv;
    uint8_t percent;
} battery_curve_point_t;

/* Conservative piecewise approximation for a 3S LiPo. Voltage is also
 * displayed because percentage from terminal voltage is inherently rough,
 * especially while the motors are drawing current. */
static const battery_curve_point_t curve[] = {
    {10500U,   0U},
    {11100U,  15U},
    {11550U,  50U},
    {12000U,  80U},
    {12600U, 100U},
};

uint16_t battery_mv_from_adc(uint16_t raw) {
    if (raw > BATTERY_ADC_FULL_SCALE) raw = BATTERY_ADC_FULL_SCALE;

    const uint32_t numerator = (uint32_t)raw * BATTERY_ADC_REF_MV *
        (BATTERY_DIVIDER_HIGH_KOHM + BATTERY_DIVIDER_LOW_KOHM);
    const uint32_t denominator = BATTERY_ADC_FULL_SCALE * BATTERY_DIVIDER_LOW_KOHM;
    return (uint16_t)((numerator + denominator / 2U) / denominator);
}

uint8_t battery_percent_from_mv(uint16_t battery_mv) {
    const size_t count = sizeof(curve) / sizeof(curve[0]);
    if (battery_mv <= curve[0].mv) return curve[0].percent;
    if (battery_mv >= curve[count - 1U].mv) return curve[count - 1U].percent;

    for (size_t i = 1U; i < count; ++i) {
        if (battery_mv <= curve[i].mv) {
            const uint32_t mv_span = curve[i].mv - curve[i - 1U].mv;
            const uint32_t pct_span = curve[i].percent - curve[i - 1U].percent;
            const uint32_t offset = battery_mv - curve[i - 1U].mv;
            return (uint8_t)(curve[i - 1U].percent +
                (offset * pct_span + mv_span / 2U) / mv_span);
        }
    }
    return 0U;
}
