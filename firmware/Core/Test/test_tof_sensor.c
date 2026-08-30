/** @file test_tof_sensor.c — VL53L0X driver tests against a fake I2C device. */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "tof_sensor.h"

static I2C_HandleTypeDef fake_bus;
static bool nack_all;
static bool data_ready;
static bool corrupt_range_burst;
static uint8_t nvm_selector;
static uint8_t result_block[12];
static uint32_t fake_ticks;
static uint16_t last_dev_addr;
static uint32_t max_timeout_seen;

uint32_t hal_get_tick_ms(void) {
    return fake_ticks++;
}

int HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr,
                     uint16_t mem_addr, uint16_t mem_add_size, uint8_t *data,
                     uint16_t size, uint32_t timeout) {
    assert(mem_add_size == I2C_MEMADD_SIZE_8BIT);
    assert(timeout != HAL_MAX_DELAY);
    last_dev_addr = dev_addr;
    if (timeout > max_timeout_seen) max_timeout_seen = timeout;
    if (nack_all || hi2c == NULL || data == NULL) return HAL_ERROR;

    if (mem_addr == 0x83 && size == 1) {
        data[0] = 1; /* NVM strobe completes. */
        return HAL_OK;
    }
    if (mem_addr == 0x13 && size == 1) {
        data[0] = data_ready ? 0x07 : 0x00;
        return HAL_OK;
    }
    if (mem_addr == 0x14 && size == 1) {
        data[0] = result_block[0];
        return HAL_OK;
    }
    if (mem_addr == 0x14 && size == sizeof(result_block)) {
        memcpy(data, result_block, size);
        if (corrupt_range_burst) data[11] = data[10];
        return HAL_OK;
    }
    if (mem_addr == 0x1E && size == 1) {
        data[0] = result_block[10];
        return HAL_OK;
    }
    if (mem_addr == 0x1F && size == 1) {
        data[0] = result_block[11];
        return HAL_OK;
    }
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return HAL_ERROR;
    memcpy(data, &hi2c->mem[mem_addr], size);
    return HAL_OK;
}

int HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr,
                      uint16_t mem_addr, uint16_t mem_add_size, uint8_t *data,
                      uint16_t size, uint32_t timeout) {
    assert(mem_add_size == I2C_MEMADD_SIZE_8BIT);
    assert(timeout != HAL_MAX_DELAY);
    last_dev_addr = dev_addr;
    if (timeout > max_timeout_seen) max_timeout_seen = timeout;
    if (nack_all || hi2c == NULL || data == NULL) return HAL_ERROR;
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return HAL_ERROR;
    if (mem_addr == 0x94 && size == 1) nvm_selector = data[0];
    memcpy(&hi2c->mem[mem_addr], data, size);
    return HAL_OK;
}

static void setup_healthy(void) {
    memset(&fake_bus, 0, sizeof(fake_bus));
    memset(result_block, 0, sizeof(result_block));
    nack_all = false;
    data_ready = true; /* reference calibrations finish immediately */
    corrupt_range_burst = false;
    nvm_selector = 0;
    fake_ticks = 0;
    last_dev_addr = 0;
    max_timeout_seen = 0;
    fake_bus.mem[0xC0] = 0xEE;
    fake_bus.mem[0x91] = 0xAB;
    fake_bus.mem[0x84] = 0x10;
    fake_bus.mem[0x92] = 5U; /* five non-aperture SPADs */
    memset(&fake_bus.mem[0xB0], 0xFF, 6U);
    tof_sensor_set_i2c(&fake_bus);
}

static void stage_range(uint8_t raw_status, uint16_t mm) {
    memset(result_block, 0, sizeof(result_block));
    result_block[0] = (uint8_t)(raw_status << 3);
    result_block[10] = (uint8_t)(mm >> 8);
    result_block[11] = (uint8_t)mm;
    data_ready = true;
}

static void test_init_configures_continuous_ranging(void) {
    setup_healthy();
    assert(tof_init());
    assert(last_dev_addr == (TOF_I2C_ADDR << 1));
    assert(max_timeout_seen <= 5);
    assert(fake_bus.mem[0xB0] == 0x1F); /* first five good SPADs enabled */
    assert(fake_bus.mem[0x84] == 0x00); /* ready interrupt active low */
    assert(fake_bus.mem[0x01] == 0xE8); /* standard ranging sequence */
    assert(fake_bus.mem[0x00] == 0x02); /* back-to-back mode */
    printf("PASS test_init_configures_continuous_ranging\n");
}

static void test_init_fails_closed(void) {
    setup_healthy();
    fake_bus.mem[0xC0] = 0xEA;
    assert(!tof_init());

    setup_healthy();
    nack_all = true;
    assert(!tof_init());

    tof_sensor_set_i2c(NULL);
    assert(!tof_init());
    printf("PASS test_init_fails_closed\n");
}

static void test_valid_range_and_error_hold_last_good(void) {
    tof_status_t status;
    setup_healthy();
    assert(tof_init());

    stage_range(11, 345);
    assert(tof_read_mm(&status) == 345);
    assert(status == TOF_OK);

    /* STM32F1 I2C2 + this breakout can duplicate the burst's first range
     * byte. Single-byte range reads must still return the true 0x0159. */
    corrupt_range_burst = true;
    stage_range(11, 345);
    assert(tof_read_mm(&status) == 345);
    assert(status == TOF_OK);
    corrupt_range_burst = false;

    stage_range(4, 999); /* signal failure is not a usable distance */
    assert(tof_read_mm(&status) == 345);
    assert(status == TOF_OUT_OF_RANGE);

    stage_range(11, 2501);
    assert(tof_read_mm(&status) == 345);
    assert(status == TOF_OUT_OF_RANGE);

    stage_range(11, 0); /* invalid near-range result must never become 000 mm */
    assert(tof_read_mm(&status) == 345);
    assert(status == TOF_OUT_OF_RANGE);
    printf("PASS test_valid_range_and_error_hold_last_good\n");
}

static void test_three_missed_samples_report_timeout(void) {
    tof_status_t status = TOF_TIMEOUT;
    setup_healthy();
    assert(tof_init());
    stage_range(11, 200);
    assert(tof_read_mm(&status) == 200 && status == TOF_OK);

    data_ready = false;
    assert(tof_read_mm(&status) == 200 && status == TOF_NO_SAMPLE);
    assert(tof_read_mm(&status) == 200 && status == TOF_NO_SAMPLE);
    assert(tof_read_mm(&status) == 200 && status == TOF_TIMEOUT);
    printf("PASS test_three_missed_samples_report_timeout\n");
}

int main(void) {
    test_init_configures_continuous_ranging();
    test_init_fails_closed();
    test_valid_range_and_error_hold_last_good();
    test_three_missed_samples_report_timeout();
    printf("\nAll VL53L0X tests passed.\n");
    return 0;
}
