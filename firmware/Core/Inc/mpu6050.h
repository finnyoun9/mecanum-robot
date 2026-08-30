/**
 * @file mpu6050.h
 * @brief I2C driver for MPU6050 accel+gyro (6-DOF, no magnetometer).
 *
 * Configured for:
 *   Gyro range:  +/-500 dps  (sensitivity 65.5 LSB/(deg/s))
 *   Accel range: +/-4 g      (sensitivity 8192 LSB/g)
 *   Sample rate: 100 Hz (1kHz internal base / (1 + SMPLRT_DIV), DLPF enabled)
 *
 * MPU6050 has no onboard magnetometer, so this project runs the IMU-only
 * (6-DOF) Mahony filter in ahrs.c — matches this sensor natively, no
 * feature is being left unused.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/* --- I2C address (AD0 pin tied low) --- */
#define MPU6050_I2C_ADDR 0x68

/* --- Register map (subset used by this driver, per MPU-6050 datasheet) --- */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A  /* DLPF_CFG applies to BOTH accel and gyro on 6050 */
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_PWR_MGMT_2   0x6C
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  /* burst read: accel(6) + temp(2) + gyro(6) */
#define MPU6050_REG_GYRO_XOUT_H  0x43

/* --- Expected WHO_AM_I value (datasheet default; some clone boards read
 * 0x72 or 0x98 instead — relax the check in mpu6050_init() if a genuine
 * clone doesn't match 0x68) --- */
#define MPU6050_WHOAMI_VAL 0x68

/* --- Full-scale range select bits (FS_SEL/AFS_SEL = 1, bits [4:3]) --- */
#define MPU6050_GYRO_FS_500DPS 0x08  /* GYRO_CONFIG  */
#define MPU6050_ACCEL_FS_4G    0x08  /* ACCEL_CONFIG */

/* --- Sensitivity constants for the ranges configured above --- */
#define MPU6050_GYRO_SENS_LSB_PER_DPS 65.5f
#define MPU6050_ACCEL_SENS_LSB_PER_G  8192.0f
#define MPU6050_DEG_TO_RAD            0.017453293f /* pi/180 */
#define MPU6050_STANDARD_GRAVITY      9.80665f      /* m/s^2 */

/**
 * @brief Bind the I2C bus handle this driver transacts on.
 *
 * Must be called before mpu6050_init(). Injected rather than referenced by
 * name (the motor.c/motor_set_tim pattern) so the driver carries no
 * assumption about which I2C peripheral it sits on: the hardware target
 * passes I2C2 (PB10/PB11 — PB8, I2C1's SCL, is RL's PWMA), while SIL and
 * unit tests pass a mock. Until it is called every transaction fails
 * closed rather than dereferencing NULL.
 *
 * @param hi2c Pointer to an I2C_HandleTypeDef (void* to keep this header
 *             free of any HAL dependency, as motor.h does for timers).
 */
void mpu6050_set_i2c(void *hi2c);

/**
 * @brief Initialise the MPU6050: verify WHO_AM_I, wake from sleep, select
 *        PLL clock source, configure DLPF/sample rate and full-scale ranges.
 * @return true if WHO_AM_I matched and every configuration write succeeded.
 *         false on any I2C failure, on an unexpected WHO_AM_I, or if no bus
 *         has been injected.
 */
bool mpu6050_init(void);

/**
 * @brief Burst-read raw accel + gyro registers.
 * @param accel Raw accel counts [x,y,z]
 * @param gyro  Raw gyro counts [x,y,z]
 */
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);

/**
 * @brief Convert raw LSB counts to physical units using the full-scale
 *        ranges configured in mpu6050_init() (+/-4g, +/-500dps).
 * @param accel_raw  Raw accel counts
 * @param gyro_raw   Raw gyro counts
 * @param accel_mps2 Output accel, m/s^2
 * @param gyro_rads  Output gyro, rad/s
 */
void mpu6050_convert_units(const int16_t accel_raw[3], const int16_t gyro_raw[3],
                            float accel_mps2[3], float gyro_rads[3]);

#endif /* MPU6050_H */
