/**
 * @file mpu6050.c
 * @brief MPU6050 accel+gyro driver via STM32 HAL I2C.
 *
 * The bus handle is injected with mpu6050_set_i2c() rather than referenced
 * by name, the same pattern motor.c uses for its timers. That keeps this
 * file free of any assumption about WHICH I2C peripheral it sits on — the
 * hardware target picks I2C2 (PB10/PB11, since PB8 is RL's PWMA) while the
 * SIL build injects its mock — and it lets a test drive the driver against
 * a fake register file.
 *
 * Every transaction uses a bounded timeout, never HAL_MAX_DELAY: a missing
 * or unpowered sensor must not stall SensorTask indefinitely.
 */

#include "mpu6050.h"
#include <stddef.h>

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#else
#include "i2c.h"
#endif

/* 7-bit address shifted into the HAL's 8-bit slave-address form. */
#define MPU6050_HAL_ADDR ((uint16_t)(MPU6050_I2C_ADDR << 1))

/* A single register access is a handful of bytes at 400 kHz; the 14-byte
 * burst is ~0.4 ms. 10 ms is generous for a healthy sensor and still keeps
 * a dead bus from eating more than a fraction of the 10 ms task period. */
#define MPU6050_I2C_TIMEOUT_MS 10U

static I2C_HandleTypeDef *bus = NULL;

void mpu6050_set_i2c(void *hi2c) {
    bus = (I2C_HandleTypeDef *)hi2c;
}

/* Both helpers are no-ops when no bus has been injected, so a build that
 * forgets mpu6050_set_i2c() reads zeros instead of dereferencing NULL. */
static bool reg_write(uint8_t reg_addr, uint8_t value) {
    if (bus == NULL) return false;
    return HAL_I2C_Mem_Write(bus, MPU6050_HAL_ADDR, reg_addr,
                             I2C_MEMADD_SIZE_8BIT, &value, 1,
                             MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool reg_read(uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (bus == NULL) return false;
    return HAL_I2C_Mem_Read(bus, MPU6050_HAL_ADDR, reg_addr,
                            I2C_MEMADD_SIZE_8BIT, data, len,
                            MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

bool mpu6050_init(void) {
    uint8_t who_am_i = 0;

    if (!reg_read(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        return false;
    }
    if (who_am_i != MPU6050_WHOAMI_VAL) {
        /* Genuine clones report 0x72/0x98. Rejecting here is deliberate:
         * silently accepting an unknown WHO_AM_I is how a mis-wired bus or
         * a different chip turns into hours of debugging a "drifting"
         * filter. Widen MPU6050_WHOAMI_VAL if a real clone shows up. */
        return false;
    }

    /* Wake from sleep, select PLL clock source (PWR_MGMT_1 bit CLKSEL=1) */
    if (!reg_write(MPU6050_REG_PWR_MGMT_1, 0x01)) return false;

    /* Enable all accel/gyro axes (clear standby bits) */
    if (!reg_write(MPU6050_REG_PWR_MGMT_2, 0x00)) return false;

    /* DLPF_CFG=3 -> ~44Hz bandwidth (gyro AND accel share this on 6050),
     * internal sample rate becomes 1kHz */
    if (!reg_write(MPU6050_REG_CONFIG, 0x03)) return false;

    /* Sample rate = 1kHz / (1 + SMPLRT_DIV) -> divider=9 gives 100Hz */
    if (!reg_write(MPU6050_REG_SMPLRT_DIV, 9)) return false;

    /* Gyro full-scale: +/-500 dps */
    if (!reg_write(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500DPS)) return false;

    /* Accel full-scale: +/-4g */
    if (!reg_write(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G)) return false;

    return true;
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buf[14] = {0}; /* ACCEL_XOUT_H..GYRO_ZOUT_L (incl. 2 temp bytes) */

    /* On a failed read buf stays zeroed, which yields a zero accel vector.
     * MahonyAHRSupdateIMU() skips its accel correction when the norm
     * collapses (ahrs.c), so a dropped sample degrades to gyro-only
     * integration rather than yanking the attitude toward a bogus down. */
    (void)reg_read(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6..7] = TEMP_OUT, unused */
    gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

void mpu6050_convert_units(const int16_t accel_raw[3], const int16_t gyro_raw[3],
                            float accel_mps2[3], float gyro_rads[3]) {
    for (int i = 0; i < 3; i++) {
        accel_mps2[i] = ((float)accel_raw[i] / MPU6050_ACCEL_SENS_LSB_PER_G)
                         * MPU6050_STANDARD_GRAVITY;
        gyro_rads[i] = ((float)gyro_raw[i] / MPU6050_GYRO_SENS_LSB_PER_DPS)
                        * MPU6050_DEG_TO_RAD;
    }
}
