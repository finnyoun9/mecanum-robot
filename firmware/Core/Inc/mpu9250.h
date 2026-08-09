/**
 * @file mpu9250.h
 * @brief I2C driver for MPU9250 accel+gyro (register map/init sequence is
 *        shared with ICM-20948's accel/gyro subsystem, so this also covers
 *        that part if substituted).
 *
 * Configured for:
 *   Gyro range:  +/-500 dps  (sensitivity 65.5 LSB/(deg/s))
 *   Accel range: +/-4 g      (sensitivity 8192 LSB/g)
 *   Sample rate: 100 Hz (1kHz internal base / (1 + SMPLRT_DIV), DLPF enabled)
 *
 * The onboard AK8963 magnetometer is not used — this project runs the
 * simpler IMU-only (6-DOF) Mahony filter in ahrs.c.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>

/* --- I2C address (AD0 pin tied low) --- */
#define MPU9250_I2C_ADDR 0x68

/* --- Register map (subset used by this driver, per MPU-9250 datasheet) --- */
#define MPU9250_REG_SMPLRT_DIV    0x19
#define MPU9250_REG_CONFIG        0x1A
#define MPU9250_REG_GYRO_CONFIG   0x1B
#define MPU9250_REG_ACCEL_CONFIG  0x1C
#define MPU9250_REG_ACCEL_CONFIG2 0x1D
#define MPU9250_REG_PWR_MGMT_1    0x6B
#define MPU9250_REG_PWR_MGMT_2    0x6C
#define MPU9250_REG_WHO_AM_I      0x75
#define MPU9250_REG_ACCEL_XOUT_H  0x3B  /* burst read: accel(6) + temp(2) + gyro(6) */
#define MPU9250_REG_GYRO_XOUT_H   0x43

/* --- Expected WHO_AM_I values --- */
#define MPU9250_WHOAMI_VAL  0x71  /* MPU9250 */
#define MPU9255_WHOAMI_VAL  0x73  /* MPU9255 */
#define ICM20948_WHOAMI_VAL 0xEA  /* ICM-20948, same accel/gyro register family */

/* --- Full-scale range select bits (FS_SEL/AFS_SEL = 1, bits [4:3]) --- */
#define MPU9250_GYRO_FS_500DPS 0x08  /* GYRO_CONFIG  */
#define MPU9250_ACCEL_FS_4G    0x08  /* ACCEL_CONFIG */

/* --- Sensitivity constants for the ranges configured above --- */
#define MPU9250_GYRO_SENS_LSB_PER_DPS 65.5f
#define MPU9250_ACCEL_SENS_LSB_PER_G  8192.0f
#define MPU9250_DEG_TO_RAD            0.017453293f /* pi/180 */
#define MPU9250_STANDARD_GRAVITY      9.80665f      /* m/s^2 */

/**
 * @brief Initialise the MPU9250: verify WHO_AM_I, wake from sleep, select
 *        PLL clock source, configure DLPF/sample rate and full-scale ranges.
 * @return true if WHO_AM_I matched a known device.
 */
bool mpu9250_init(void);

/**
 * @brief Burst-read raw accel + gyro registers.
 * @param accel Raw accel counts [x,y,z]
 * @param gyro  Raw gyro counts [x,y,z]
 */
void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3]);

/**
 * @brief Convert raw LSB counts to physical units using the full-scale
 *        ranges configured in mpu9250_init() (+/-4g, +/-500dps).
 * @param accel_raw  Raw accel counts
 * @param gyro_raw   Raw gyro counts
 * @param accel_mps2 Output accel, m/s^2
 * @param gyro_rads  Output gyro, rad/s
 */
void mpu9250_convert_units(const int16_t accel_raw[3], const int16_t gyro_raw[3],
                            float accel_mps2[3], float gyro_rads[3]);

#endif /* MPU9250_H */
