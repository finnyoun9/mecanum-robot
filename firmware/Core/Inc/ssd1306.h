/** @file ssd1306.h — minimal 128x64 SSD1306 I2C display transport. */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_I2C_ADDR    0x3CU
#define SSD1306_WIDTH       128U
#define SSD1306_HEIGHT      64U
#define SSD1306_FRAME_BYTES (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)

/** Bind the I2C bus. The display may share I2C2 with MPU6050/VL53L0X. */
void ssd1306_set_i2c(void *hi2c);

/** Initialise a 128x64 SSD1306 and clear its display RAM. */
bool ssd1306_init(void);

/** Write one page-formatted frame (8 vertical pixels per byte). */
bool ssd1306_write_frame(const uint8_t *frame, uint16_t size);

/** Clear display RAM without allocating a framebuffer. */
bool ssd1306_clear(void);

bool ssd1306_set_enabled(bool enabled);
bool ssd1306_set_contrast(uint8_t contrast);
/** Force every pixel on/off without changing display RAM (hardware test). */
bool ssd1306_set_all_pixels(bool enabled);

#endif /* SSD1306_H */
