/**
 * @file mpu9250.c
 * @brief MPU9250 accel+gyro driver via STM32 HAL I2C.
 *
 * I2C bus/pin assignment isn't final (hardware in transit), so the actual
 * HAL transaction calls are commented-out TODO placeholders, same pattern
 * as motor.c/encoder.c. Init sequence and unit-conversion math are complete.
 */

#include "mpu9250.h"
#include <stddef.h>

/* --- Replace with actual HAL includes in real project --- */
/* #include "i2c.h" */

bool mpu9250_init(void) {
    uint8_t who_am_i = 0;
    /* HAL_I2C_Mem_Read(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY); */
    (void)who_am_i;

    if (who_am_i != MPU9250_WHOAMI_VAL &&
        who_am_i != MPU9255_WHOAMI_VAL &&
        who_am_i != ICM20948_WHOAMI_VAL) {
        /* return false; -- uncomment once I2C bus is wired; placeholder
         * always proceeds so the rest of the init sequence stays testable */
    }

    uint8_t reg;

    /* Wake from sleep, select PLL clock source (PWR_MGMT_1 bit CLKSEL=1) */
    reg = 0x01;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    /* Enable all accel/gyro axes (clear standby bits) */
    reg = 0x00;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_PWR_MGMT_2,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    /* DLPF_CFG=3 -> ~44Hz bandwidth, internal sample rate becomes 1kHz */
    reg = 0x03;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    /* Sample rate = 1kHz / (1 + SMPLRT_DIV) -> divider=9 gives 100Hz */
    reg = 9;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_SMPLRT_DIV,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    /* Gyro full-scale: +/-500 dps */
    reg = MPU9250_GYRO_FS_500DPS;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_GYRO_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    /* Accel full-scale: +/-4g */
    reg = MPU9250_ACCEL_FS_4G;
    /* HAL_I2C_Mem_Write(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY); */

    (void)reg;
    return true;
}

void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buf[14] = {0}; /* ACCEL_XOUT_H..GYRO_ZOUT_L (incl. 2 temp bytes) */

    /* HAL_I2C_Mem_Read(&hi2c1, MPU9250_I2C_ADDR << 1, MPU9250_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), HAL_MAX_DELAY); */

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6..7] = TEMP_OUT, unused */
    gyro[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

void mpu9250_convert_units(const int16_t accel_raw[3], const int16_t gyro_raw[3],
                            float accel_mps2[3], float gyro_rads[3]) {
    for (int i = 0; i < 3; i++) {
        accel_mps2[i] = ((float)accel_raw[i] / MPU9250_ACCEL_SENS_LSB_PER_G)
                         * MPU9250_STANDARD_GRAVITY;
        gyro_rads[i] = ((float)gyro_raw[i] / MPU9250_GYRO_SENS_LSB_PER_DPS)
                        * MPU9250_DEG_TO_RAD;
    }
}
