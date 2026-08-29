/** @file oled_ui.c — compact 5x7 text UI for the chassis SSD1306. */
#include "oled_ui.h"

#include <stddef.h>
#include <string.h>

/* Digits 0-9 followed by uppercase A-Z. Columns are vertical, LSB at top. */
static const uint8_t glyphs[36][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
};

static const uint8_t *glyph(char c) {
    static const uint8_t plus[5]  = {0x08,0x08,0x3E,0x08,0x08};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t dot[5]   = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t blank[5] = {0};
    if (c >= '0' && c <= '9') return glyphs[(uint8_t)(c - '0')];
    if (c >= 'A' && c <= 'Z') return glyphs[10U + (uint8_t)(c - 'A')];
    if (c == '+') return plus;
    if (c == '-') return minus;
    if (c == '.') return dot;
    if (c == '/') return slash;
    if (c == ':') return colon;
    return blank;
}

/* The panel is only 0.96 inch across. A native 5x7 glyph is physically
 * about 1 mm tall and reads as noise at normal viewing distance, so render
 * the compact font at 2x (10x14 pixels) into the page-format framebuffer. */
static void pixel(uint8_t *frame, uint8_t x, uint8_t y) {
    if (x < SSD1306_WIDTH && y < SSD1306_HEIGHT) {
        frame[(uint16_t)(y / 8U) * SSD1306_WIDTH + x] |= (uint8_t)(1U << (y % 8U));
    }
}

static void text(uint8_t *frame, uint8_t row, uint8_t col, const char *s) {
    uint8_t x = (uint8_t)(col * 12U);
    uint8_t y = (uint8_t)(row * 16U);
    if (row >= 4U || s == NULL) return;

    while (*s != '\0' && x + 10U < SSD1306_WIDTH) {
        const uint8_t *g = glyph(*s++);
        for (uint8_t glyph_x = 0; glyph_x < 5U; ++glyph_x) {
            for (uint8_t glyph_y = 0; glyph_y < 7U; ++glyph_y) {
                if ((g[glyph_x] & (uint8_t)(1U << glyph_y)) != 0U) {
                    pixel(frame, (uint8_t)(x + glyph_x * 2U),
                          (uint8_t)(y + glyph_y * 2U));
                    pixel(frame, (uint8_t)(x + glyph_x * 2U + 1U),
                          (uint8_t)(y + glyph_y * 2U));
                    pixel(frame, (uint8_t)(x + glyph_x * 2U),
                          (uint8_t)(y + glyph_y * 2U + 1U));
                    pixel(frame, (uint8_t)(x + glyph_x * 2U + 1U),
                          (uint8_t)(y + glyph_y * 2U + 1U));
                }
            }
        }
        x = (uint8_t)(x + 12U);
    }
}

static void unsigned_dec(char *out, uint16_t value, uint8_t width) {
    out[width] = '\0';
    do {
        out[--width] = (char)('0' + value % 10U);
        value /= 10U;
    } while (width > 0U);
}

static void signed_fixed(char out[7], int16_t value, uint16_t scale,
                         uint8_t decimals) {
    uint16_t magnitude = value < 0 ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
    uint16_t whole = magnitude / scale;
    uint16_t fraction = magnitude % scale;
    out[0] = value < 0 ? '-' : '+';
    out[1] = (char)('0' + (whole / 10U) % 10U);
    out[2] = (char)('0' + whole % 10U);
    out[3] = '.';
    out[4] = (char)('0' + fraction / (scale / 10U));
    out[5] = decimals > 1U ? (char)('0' + (fraction / (scale / 100U)) % 10U) : '\0';
    out[6] = '\0';
}

static void render_overview(uint8_t *frame, const oled_ui_data_t *d) {
    char value[6];
    text(frame, 0, 0, "ROBOT");
    text(frame, 1, 0, "TOF");
    unsigned_dec(value, d->tof_mm, 4U);
    text(frame, 1, 4, value);
    text(frame, 2, 0, d->comm_ok ? "LINK OK" : "NO LINK");
    value[0] = "0123456789ABCDEF"[d->error_flags >> 4];
    value[1] = "0123456789ABCDEF"[d->error_flags & 0x0FU];
    value[2] = '\0';
    text(frame, 3, 0, d->emergency_stop ? "STOP ERR" : "RUN ERR");
    text(frame, 3, 8, value);
}

static void render_wheels(uint8_t *frame, const oled_ui_data_t *d) {
    static const char *labels[4] = {"FL", "FR", "RL", "RR"};
    char target[7];
    for (uint8_t i = 0; i < 4U; ++i) {
        signed_fixed(target, d->target_deci_rads[i], 10U, 1U);
        text(frame, i, 0, labels[i]);
        text(frame, i, 3, target);
    }
}

static void render_imu(uint8_t *frame, const oled_ui_data_t *d) {
    static const char *labels[3] = {"X", "Y", "Z"};
    char value[7];
    text(frame, 0, 0, "GYRO");
    for (uint8_t i = 0; i < 3U; ++i) {
        signed_fixed(value, d->gyro_milli_rads[i], 1000U, 2U);
        text(frame, (uint8_t)(1U + i), 0, labels[i]);
        text(frame, (uint8_t)(1U + i), 2, value);
    }
}

void oled_ui_render(uint8_t frame[SSD1306_FRAME_BYTES], uint8_t page,
                    const oled_ui_data_t *data) {
    if (frame == NULL || data == NULL) return;
    memset(frame, 0, SSD1306_FRAME_BYTES);
    switch (page % OLED_UI_PAGE_COUNT) {
    case 0: render_overview(frame, data); break;
    case 1: render_wheels(frame, data); break;
    default: render_imu(frame, data); break;
    }
}
