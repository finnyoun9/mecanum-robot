/** @file oled_ui.h — allocation-free 128x64 chassis status pages. */
#ifndef OLED_UI_H
#define OLED_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "ssd1306.h"

#define OLED_UI_PAGE_COUNT 3U

typedef struct {
    uint16_t tof_mm;
    uint8_t error_flags;
    bool comm_ok;
    bool emergency_stop;
    int16_t target_deci_rads[4];
    int16_t measured_deci_rads[4];
    int16_t gyro_milli_rads[3];
    int16_t qw_centi;
} oled_ui_data_t;

/** Render one complete page into a caller-owned 1024-byte framebuffer. */
void oled_ui_render(uint8_t frame[SSD1306_FRAME_BYTES], uint8_t page,
                    const oled_ui_data_t *data);

#endif
