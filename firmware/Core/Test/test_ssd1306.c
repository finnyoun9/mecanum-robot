/** @file test_ssd1306.c — host tests for the bounded SSD1306 transport. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "ssd1306.h"

static I2C_HandleTypeDef fake_bus;
static bool nack_all;
static uint16_t calls;
static uint16_t data_bytes;
static uint8_t first_command;
static uint8_t last_command;

int HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr,
                     uint16_t mem_addr, uint16_t mem_add_size, uint8_t *data,
                     uint16_t size, uint32_t timeout) {
    (void)hi2c; (void)dev_addr; (void)mem_addr; (void)mem_add_size;
    (void)data; (void)size; (void)timeout;
    return HAL_ERROR;
}

int HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t dev_addr,
                            uint8_t *data, uint16_t size, uint32_t timeout) {
    assert(hi2c == &fake_bus);
    assert(dev_addr == (SSD1306_I2C_ADDR << 1));
    assert(timeout <= 5 && timeout != HAL_MAX_DELAY);
    assert(size >= 2U);
    if (nack_all) return HAL_ERROR;
    ++calls;
    if (data[0] == 0x00U) {
        if (calls == 1) first_command = data[1];
        last_command = data[size - 1U];
    } else if (data[0] == 0x40U) {
        data_bytes = (uint16_t)(data_bytes + size - 1U);
    }
    return HAL_OK;
}

static void setup(void) {
    memset(&fake_bus, 0, sizeof(fake_bus));
    nack_all = false;
    calls = 0;
    data_bytes = 0;
    first_command = 0;
    last_command = 0;
    ssd1306_set_i2c(&fake_bus);
}

static void test_init_sends_commands_and_clears_ram(void) {
    setup();
    assert(ssd1306_init());
    assert(first_command == 0xAE);
    assert(last_command == 0xAF);
    assert(data_bytes == SSD1306_FRAME_BYTES);
    printf("PASS test_init_sends_commands_and_clears_ram\n");
}

static void test_full_frame_and_size_guard(void) {
    static uint8_t frame[SSD1306_FRAME_BYTES];
    setup();
    memset(frame, 0xA5, sizeof(frame));
    assert(!ssd1306_write_frame(frame, sizeof(frame) - 1));
    assert(calls == 0);
    assert(ssd1306_write_frame(frame, sizeof(frame)));
    assert(data_bytes == sizeof(frame));
    printf("PASS test_full_frame_and_size_guard\n");
}

static void test_single_page_is_bounded(void) {
    static uint8_t page[SSD1306_WIDTH];
    setup();
    memset(page, 0x5A, sizeof(page));
    assert(!ssd1306_write_page(8U, page));
    assert(calls == 0);
    assert(ssd1306_write_page(3U, page));
    assert(data_bytes == SSD1306_WIDTH);
    printf("PASS test_single_page_is_bounded\n");
}

static void test_bus_errors_fail_closed(void) {
    setup();
    nack_all = true;
    assert(!ssd1306_init());
    ssd1306_set_i2c(NULL);
    assert(!ssd1306_clear());
    printf("PASS test_bus_errors_fail_closed\n");
}

int main(void) {
    test_init_sends_commands_and_clears_ram();
    test_full_frame_and_size_guard();
    test_single_page_is_bounded();
    test_bus_errors_fail_closed();
    printf("\nAll SSD1306 tests passed.\n");
    return 0;
}
