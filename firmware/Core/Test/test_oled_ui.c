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
    uint32_t sums[OLED_UI_PAGE_COUNT];
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

    for (uint8_t page = 0; page < OLED_UI_PAGE_COUNT; ++page) {
        memset(frame, 0xFF, sizeof(frame));
        oled_ui_render(frame, page, &data);
        sums[page] = checksum(frame);
        assert(sums[page] != 0U);
        assert(memchr(frame, 0x00, sizeof(frame)) != NULL);
    }
    assert(sums[0] != sums[1]);
    assert(sums[1] != sums[2]);

    oled_ui_render(NULL, 0U, &data);
    oled_ui_render(frame, 0U, NULL);
    printf("PASS OLED UI page rendering\n");
    return 0;
}
