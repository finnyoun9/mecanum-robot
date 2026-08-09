/**
 * @file tof_sensor.c
 * @brief VL53L0X ToF driver via STM32 HAL I2C.
 *
 * I2C bus/pin assignment isn't final (hardware in transit), so the actual
 * HAL transaction calls are commented-out TODO placeholders, same pattern
 * as motor.c/encoder.c. Init/read/status logic is complete.
 */

#include "tof_sensor.h"
#include <stddef.h>

/* --- Replace with actual HAL includes in real project --- */
/* #include "i2c.h" */

/* --- I2C address (default, before any address-change command) --- */
#define TOF_I2C_ADDR 0x29

/* --- Register map (subset used by this driver, per VL53L0X datasheet/API) --- */
#define TOF_REG_MODEL_ID            0xC0
#define TOF_MODEL_ID_VAL            0xEE
#define TOF_REG_SYSRANGE_START      0x00
#define TOF_REG_RESULT_RANGE_STATUS 0x14
#define TOF_REG_RESULT_RANGE_MM     0x1E /* RESULT_RANGE_STATUS + 10, per ST API */

#define TOF_RANGE_STATUS_MASK  0x78 /* bits [6:3]: range status code */
#define TOF_RANGE_STATUS_VALID 0x00 /* status code 0 = valid range ("no error") */

#define TOF_MAX_TIMEOUT_MS 50   /* datasheet: worst-case ranging cycle ~30-40ms */
#define TOF_MAX_VALID_MM   2000 /* VL53L0X practical max range */

#define TOF_TIMEOUT_ERR_THRESHOLD 3 /* consecutive misses before reporting TOF_TIMEOUT */

static uint16_t last_valid_mm       = 0;
static uint8_t  consecutive_timeouts = 0;

bool tof_init(void) {
    uint8_t model_id = 0;
    /* HAL_I2C_Mem_Read(&hi2c1, TOF_I2C_ADDR << 1, TOF_REG_MODEL_ID,
                         I2C_MEMADD_SIZE_8BIT, &model_id, 1, HAL_MAX_DELAY); */
    (void)model_id;

    if (model_id != TOF_MODEL_ID_VAL) {
        /* return false; -- uncomment once I2C bus is wired; placeholder
         * always proceeds so the rest of the driver stays testable */
    }

    consecutive_timeouts = 0;
    last_valid_mm = 0;
    return true;
}

/** Poll RESULT_RANGE_STATUS data-ready bit (bit 0) until set or timed out. */
static bool tof_wait_for_data(void) {
    /*
    uint32_t start = hal_get_tick_ms();
    uint8_t status_reg = 0;
    do {
        HAL_I2C_Mem_Read(&hi2c1, TOF_I2C_ADDR << 1, TOF_REG_RESULT_RANGE_STATUS,
                          I2C_MEMADD_SIZE_8BIT, &status_reg, 1, HAL_MAX_DELAY);
        if (status_reg & 0x01) return true;
    } while ((hal_get_tick_ms() - start) < TOF_MAX_TIMEOUT_MS);
    return false;
    */
    return false; /* placeholder: always "times out" until I2C is wired */
}

uint16_t tof_read_mm(tof_status_t *status) {
    uint8_t start_cmd = 0x01;
    /* HAL_I2C_Mem_Write(&hi2c1, TOF_I2C_ADDR << 1, TOF_REG_SYSRANGE_START,
                          I2C_MEMADD_SIZE_8BIT, &start_cmd, 1, HAL_MAX_DELAY); */
    (void)start_cmd;

    if (!tof_wait_for_data()) {
        if (consecutive_timeouts < 0xFF) consecutive_timeouts++;
        if (status) {
            *status = (consecutive_timeouts >= TOF_TIMEOUT_ERR_THRESHOLD)
                       ? TOF_TIMEOUT : TOF_OK;
        }
        return last_valid_mm; /* hold last good reading rather than report 0 */
    }

    uint8_t range_status = 0;
    uint8_t buf[2] = {0};
    /* HAL_I2C_Mem_Read(&hi2c1, TOF_I2C_ADDR << 1, TOF_REG_RESULT_RANGE_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &range_status, 1, HAL_MAX_DELAY); */
    /* HAL_I2C_Mem_Read(&hi2c1, TOF_I2C_ADDR << 1, TOF_REG_RESULT_RANGE_MM,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), HAL_MAX_DELAY); */

    consecutive_timeouts = 0;
    uint16_t mm = (uint16_t)((buf[0] << 8) | buf[1]);

    if ((range_status & TOF_RANGE_STATUS_MASK) != TOF_RANGE_STATUS_VALID ||
        mm > TOF_MAX_VALID_MM) {
        if (status) *status = TOF_OUT_OF_RANGE;
        return last_valid_mm;
    }

    last_valid_mm = mm;
    if (status) *status = TOF_OK;
    return mm;
}
