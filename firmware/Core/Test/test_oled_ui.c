/** @file test_oled_ui.c — deterministic framebuffer tests for OLED pages. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "oled_ui.h"

static uint32_t checksum(const uint8_t *frame) {
    uint32_t sum = 0U;
    for (uint16_t i = 0; i < SSD1306_FRAME_BYTES; ++i) {
        sum = sum * 33U + frame[i];
    }
    return sum;
}

int main(void) {
    uint8_t frame[SSD1306_FRAME_BYTES];
    uint32_t initial_sum;
    oled_ui_data_t data = {
        .tof_mm = 771U,
        .error_flags = 0x04U,
        .comm_ok = true,
        .emergency_stop = false,
        .target_deci_rads = {123, -45, 67, -89},
        .measured_deci_rads = {120, -40, 60, -80},
        .gyro_milli_rads = {12, -45, 678},
        .qw_centi = 99,
    };

    memset(frame, 0xFF, sizeof(frame));
    oled_ui_render(frame, 0U, &data);
    initial_sum = checksum(frame);
    assert(initial_sum != 0U);
    assert(memchr(frame, 0x00, sizeof(frame)) != NULL);

    data.tof_mm = 1234U;
    oled_ui_render(frame, 0U, &data);
    assert(checksum(frame) != initial_sum);

    oled_ui_render(NULL, 0U, &data);
    oled_ui_render(frame, 0U, NULL);
    printf("PASS OLED UI page rendering\n");
    return 0;
}
