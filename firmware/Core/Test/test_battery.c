#include <assert.h>
#include <stdio.h>

#include "battery.h"

int main(void) {
    assert(battery_mv_from_adc(0U) == 0U);
    assert(battery_mv_from_adc(4095U) == 15522U);
    assert(battery_mv_from_adc(3325U) >= 12595U);
    assert(battery_mv_from_adc(3325U) <= 12605U);
    assert(battery_mv_from_adc(5000U) == 15522U);

    assert(battery_percent_from_mv(10000U) == 0U);
    assert(battery_percent_from_mv(10500U) == 0U);
    assert(battery_percent_from_mv(11100U) == 15U);
    assert(battery_percent_from_mv(11550U) == 50U);
    assert(battery_percent_from_mv(12000U) == 80U);
    assert(battery_percent_from_mv(12600U) == 100U);
    assert(battery_percent_from_mv(13000U) == 100U);
    assert(battery_percent_from_mv(11775U) == 65U);

    printf("PASS battery ADC conversion and 3S percentage curve\n");
    return 0;
}
