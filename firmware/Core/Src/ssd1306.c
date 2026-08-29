/**
 * @file ssd1306.c
 * @brief Memory-bounded SSD1306 I2C transport for a 128x64 panel.
 *
 * Graphics and fonts deliberately stay outside this driver. The caller may
 * keep a framebuffer where appropriate; ssd1306_clear() uses one 32-byte
 * constant chunk, so merely enabling the display does not consume 1 KiB of
 * the STM32F103's 20 KiB SRAM.
 */

#include "ssd1306.h"

#include <stddef.h>
#include <string.h>

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#else
#include "i2c.h"
#endif

#define SSD1306_HAL_ADDR       ((uint16_t)(SSD1306_I2C_ADDR << 1))
#define SSD1306_CONTROL_CMD    0x00U
#define SSD1306_CONTROL_DATA   0x40U
#define SSD1306_TIMEOUT_MS     5U
#define SSD1306_CHUNK_BYTES    32U

static I2C_HandleTypeDef *bus = NULL;
static uint8_t tx[SSD1306_CHUNK_BYTES + 1U];

void ssd1306_set_i2c(void *hi2c) {
    bus = (I2C_HandleTypeDef *)hi2c;
}

static bool send(uint8_t control, const uint8_t *data, uint16_t size) {
    if (bus == NULL || data == NULL || size == 0U || size > SSD1306_CHUNK_BYTES) {
        return false;
    }
    /* Send the SSD1306 control byte as data, exactly as the known-good
     * driver in stm32-smart-home-ota does. Some clone panels are stricter
     * about this than HAL's memory-write convenience wrapper. */
    tx[0] = control;
    memcpy(&tx[1], data, size);
    return HAL_I2C_Master_Transmit(bus, SSD1306_HAL_ADDR, tx, (uint16_t)(size + 1U),
                                   SSD1306_TIMEOUT_MS) == HAL_OK;
}

static bool set_page_window(uint8_t page) {
    uint8_t page_command = (uint8_t)(0xB0U | page);
    uint8_t high_column = 0x10U;
    uint8_t low_column = 0x00U;
    return page < (SSD1306_HEIGHT / 8U) &&
           send(SSD1306_CONTROL_CMD, &page_command, 1U) &&
           send(SSD1306_CONTROL_CMD, &high_column, 1U) &&
           send(SSD1306_CONTROL_CMD, &low_column, 1U);
}

bool ssd1306_write_frame(const uint8_t *frame, uint16_t size) {
    if (frame == NULL || size != SSD1306_FRAME_BYTES) {
        return false;
    }
    for (uint8_t page = 0; page < (SSD1306_HEIGHT / 8U); ++page) {
        if (!ssd1306_write_page(page, &frame[(uint16_t)page * SSD1306_WIDTH])) {
            return false;
        }
    }
    return true;
}

bool ssd1306_write_page(uint8_t page, const uint8_t data[SSD1306_WIDTH]) {
    if (data == NULL || !set_page_window(page)) {
        return false;
    }
    for (uint16_t offset = 0; offset < SSD1306_WIDTH;
         offset += SSD1306_CHUNK_BYTES) {
        if (!send(SSD1306_CONTROL_DATA, &data[offset], SSD1306_CHUNK_BYTES)) {
            return false;
        }
    }
    return true;
}

bool ssd1306_clear(void) {
    static const uint8_t zeros[SSD1306_CHUNK_BYTES] = {0};
    for (uint8_t page = 0; page < (SSD1306_HEIGHT / 8U); ++page) {
        if (!set_page_window(page)) return false;
        for (uint8_t chunk = 0; chunk < SSD1306_WIDTH / SSD1306_CHUNK_BYTES;
             ++chunk) {
            if (!send(SSD1306_CONTROL_DATA, zeros, sizeof(zeros))) return false;
        }
    }
    return true;
}

bool ssd1306_set_enabled(bool enabled) {
    uint8_t command = enabled ? 0xAFU : 0xAEU;
    return send(SSD1306_CONTROL_CMD, &command, 1);
}

bool ssd1306_set_contrast(uint8_t contrast) {
    uint8_t commands[2] = {0x81, contrast};
    return send(SSD1306_CONTROL_CMD, commands, sizeof(commands));
}

bool ssd1306_set_all_pixels(bool enabled) {
    uint8_t command = enabled ? 0xA5U : 0xA4U;
    return send(SSD1306_CONTROL_CMD, &command, 1);
}

bool ssd1306_init(void) {
    static const uint8_t commands[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock divide / oscillator */
        0xA8, 0x3F, /* multiplex 1/64 */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line 0 */
        0x8D, 0x14, /* charge pump on */
        0x20, 0x00, /* horizontal addressing */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan reversed */
        0xDA, 0x12, /* COM pins for 128x64 */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* pre-charge */
        0xDB, 0x30, /* VCOMH */
        0xA4,       /* use display RAM */
        0xA6,       /* normal display */
    };

    if (!send(SSD1306_CONTROL_CMD, commands, sizeof(commands)) ||
        !ssd1306_clear()) return false;
    return ssd1306_set_enabled(true);
}
