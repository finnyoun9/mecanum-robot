/** @file oled_ui.c — compact one-page status dashboard for the SSD1306. */
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

/* Native 5x7 glyphs: a one-pixel stroke and 6x12 character grid. */
static void pixel(uint8_t *frame, uint8_t x, uint8_t y) {
    if (x < SSD1306_WIDTH && y < SSD1306_HEIGHT) {
        frame[(uint16_t)(y / 8U) * SSD1306_WIDTH + x] |= (uint8_t)(1U << (y % 8U));
    }
}

static void text_at(uint8_t *frame, uint8_t x, uint8_t y, const char *s) {
    if (s == NULL) return;

    while (*s != '\0' && x + 5U < SSD1306_WIDTH) {
        const uint8_t *g = glyph(*s++);
        for (uint8_t glyph_x = 0; glyph_x < 5U; ++glyph_x) {
            for (uint8_t glyph_y = 0; glyph_y < 7U; ++glyph_y) {
                if ((g[glyph_x] & (uint8_t)(1U << glyph_y)) != 0U) {
                    pixel(frame, (uint8_t)(x + glyph_x),
                          (uint8_t)(y + glyph_y));
                }
            }
        }
        x = (uint8_t)(x + 6U);
    }
}

/* The distance is the chassis UI's primary action signal. A 2x glyph gives a
 * crisp 10x14 digit without a second font table or dynamic framebuffer. */
static void large_text(uint8_t *frame, uint8_t x, uint8_t y, const char *s) {
    if (s == NULL) return;
    while (*s != '\0' && x + 9U < SSD1306_WIDTH) {
        const uint8_t *g = glyph(*s++);
        for (uint8_t glyph_x = 0; glyph_x < 5U; ++glyph_x) {
            for (uint8_t glyph_y = 0; glyph_y < 7U; ++glyph_y) {
                if ((g[glyph_x] & (uint8_t)(1U << glyph_y)) != 0U) {
                    for (uint8_t dx = 0; dx < 2U; ++dx) {
                        for (uint8_t dy = 0; dy < 2U; ++dy) {
                            pixel(frame, (uint8_t)(x + glyph_x * 2U + dx),
                                  (uint8_t)(y + glyph_y * 2U + dy));
                        }
                    }
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

static void signed_two(char out[4], int16_t value) {
    uint16_t magnitude = value < 0 ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
    out[0] = value < 0 ? '-' : '+';
    out[1] = (char)('0' + (magnitude / 10U) % 10U);
    out[2] = (char)('0' + magnitude % 10U);
    out[3] = '\0';
}

void oled_ui_render(uint8_t frame[SSD1306_FRAME_BYTES], uint8_t page,
                    const oled_ui_data_t *data) {
    char value[6];
    char line[22];
    char wheel[4];
    char gyro[3][4];

    if (frame == NULL || data == NULL) return;
    (void)page;
    memset(frame, 0, SSD1306_FRAME_BYTES);
    /* Header, then the one field a driver must read at a glance. */
    text_at(frame, 0U, 0U, "TOF RANGE MM");
    if (data->tof_valid) {
        unsigned_dec(value, data->tof_mm, 4U);
    } else {
        memcpy(value, "----", 5U);
    }
    large_text(frame, 39U, 10U, value);

    line[0] = 'L'; line[1] = 'I'; line[2] = 'N'; line[3] = 'K'; line[4] = ':';
    line[5] = data->comm_ok ? 'O' : 'X'; line[6] = 'K'; line[7] = ' ';
    line[8] = data->emergency_stop ? 'S' : 'R'; line[9] = 'U'; line[10] = 'N';
    line[11] = ' '; line[12] = 'E'; line[13] = ':';
    line[14] = "0123456789ABCDEF"[data->error_flags >> 4];
    line[15] = "0123456789ABCDEF"[data->error_flags & 0x0FU]; line[16] = '\0';
    text_at(frame, 0U, 28U, line);

    for (uint8_t i = 0; i < 3U; ++i) {
        signed_two(gyro[i], (int16_t)(data->gyro_milli_rads[i] / 1000));
    }
    line[0] = 'G'; line[1] = ':'; memcpy(&line[2], gyro[0], 3U); line[5] = ',';
    memcpy(&line[6], gyro[1], 3U); line[9] = ','; memcpy(&line[10], gyro[2], 3U);
    line[13] = '\0'; text_at(frame, 0U, 40U, line);

    line[0] = 'W'; line[1] = ':';
    for (uint8_t i = 0; i < 4U; ++i) {
        signed_two(wheel, (int16_t)(data->target_deci_rads[i] / 10));
        memcpy(&line[2U + i * 4U], wheel, 3U);
        line[5U + i * 4U] = i == 3U ? '\0' : ' ';
    }
    text_at(frame, 0U, 52U, line);
}
