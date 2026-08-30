/**
 * @file test_mpu6050.c
 * @brief Unit tests for mpu6050.c against a fake I2C register file.
 *
 * The driver used to be unreachable: every HAL call was commented out, so
 * mpu6050_init() returned true unconditionally and read_raw() always
 * produced zeros. Injecting the bus with mpu6050_set_i2c() is what makes
 * the real transaction path testable off-hardware — these tests drive the
 * same code that runs on silicon, only the bus is fake.
 *
 * The fake deliberately models the two things that bite on a real bus: a
 * device that does not answer (NACK) and a device that answers with the
 * wrong identity (a different chip, or SDA/SCL swapped onto another
 * module). Both must fail closed rather than silently produce attitude.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "mpu6050.h"
#include "i2c.h"

/* --- Fake bus ------------------------------------------------------------
 * mock_i2c_t is the SIL mock's register-file device (hal_stubs.c copies to
 * and from .mem). Tests set .mem directly to stage what the chip reports.
 * nack_all forces every transaction to fail, modelling an absent device. */
static I2C_HandleTypeDef fake_bus;
static bool nack_all = false;

int HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                     uint16_t mem_add_size, uint8_t *data, uint16_t size,
                     uint32_t timeout) {
    (void)dev_addr;
    /* A bounded timeout is part of the contract: HAL_MAX_DELAY here would
     * let a dead sensor stall SensorTask forever. */
    assert(timeout != HAL_MAX_DELAY);
    assert(mem_add_size == I2C_MEMADD_SIZE_8BIT);
    if (nack_all) return HAL_ERROR;
    if (!hi2c || !data) return HAL_ERROR;
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return HAL_ERROR;
    memcpy(data, &hi2c->mem[mem_addr], size);
    return HAL_OK;
}

int HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t mem_addr,
                      uint16_t mem_add_size, uint8_t *data, uint16_t size,
                      uint32_t timeout) {
    (void)dev_addr;
    assert(timeout != HAL_MAX_DELAY);
    assert(mem_add_size == I2C_MEMADD_SIZE_8BIT);
    if (nack_all) return HAL_ERROR;
    if (!hi2c || !data) return HAL_ERROR;
    if ((size_t)mem_addr + size > sizeof(hi2c->mem)) return HAL_ERROR;
    memcpy(&hi2c->mem[mem_addr], data, size);
    return HAL_OK;
}

/* Present a healthy MPU6050 and bind it. */
static void setup_healthy(void) {
    memset(&fake_bus, 0, sizeof(fake_bus));
    nack_all = false;
    fake_bus.mem[MPU6050_REG_WHO_AM_I] = MPU6050_WHOAMI_VAL;
    mpu6050_set_i2c(&fake_bus);
}

/* Stage a big-endian 16-bit sample pair into the burst-read window. */
static void stage_sample(int16_t ax, int16_t ay, int16_t az,
                          int16_t gx, int16_t gy, int16_t gz) {
    uint8_t *m = &fake_bus.mem[MPU6050_REG_ACCEL_XOUT_H];
    const int16_t vals[7] = {ax, ay, az, 0 /* temp */, gx, gy, gz};
    for (int i = 0; i < 7; i++) {
        m[i * 2]     = (uint8_t)((uint16_t)vals[i] >> 8);
        m[i * 2 + 1] = (uint8_t)((uint16_t)vals[i] & 0xFF);
    }
}

/* ===== Init ===== */

static void test_init_configures_every_register(void) {
    setup_healthy();
    assert(mpu6050_init() == true);

    /* Wake + PLL clock source, not the default sleep-with-internal-osc. */
    assert(fake_bus.mem[MPU6050_REG_PWR_MGMT_1] == 0x01);
    /* All six axes out of standby. */
    assert(fake_bus.mem[MPU6050_REG_PWR_MGMT_2] == 0x00);
    /* DLPF_CFG=3 (~44 Hz), which also sets the 1 kHz internal base rate. */
    assert(fake_bus.mem[MPU6050_REG_CONFIG] == 0x03);
    /* 1 kHz / (1 + 9) = 100 Hz, matching SensorTask's 10 ms period. */
    assert(fake_bus.mem[MPU6050_REG_SMPLRT_DIV] == 9);
    /* Ranges must match the sensitivity constants convert_units() uses. */
    assert(fake_bus.mem[MPU6050_REG_GYRO_CONFIG] == MPU6050_GYRO_FS_500DPS);
    assert(fake_bus.mem[MPU6050_REG_ACCEL_CONFIG] == MPU6050_ACCEL_FS_4G);

    printf("PASS test_init_configures_every_register\n");
}

static void test_init_rejects_wrong_who_am_i(void) {
    setup_healthy();
    fake_bus.mem[MPU6050_REG_WHO_AM_I] = 0x71;  /* e.g. an MPU9250 */
    assert(mpu6050_init() == false);

    /* Must bail before touching config: half-configuring a chip that is not
     * the one we think it is leaves the bus in an unknown state. */
    assert(fake_bus.mem[MPU6050_REG_PWR_MGMT_1] == 0x00);

    printf("PASS test_init_rejects_wrong_who_am_i\n");
}

static void test_init_fails_when_bus_nacks(void) {
    setup_healthy();
    nack_all = true;
    assert(mpu6050_init() == false);
    printf("PASS test_init_fails_when_bus_nacks\n");
}

static void test_init_fails_with_no_bus_injected(void) {
    /* Rebinding to NULL models a target that forgot mpu6050_set_i2c().
     * Must fail closed, not dereference NULL. */
    mpu6050_set_i2c(NULL);
    assert(mpu6050_init() == false);
    printf("PASS test_init_fails_with_no_bus_injected\n");
}

/* ===== Reads ===== */

static void test_read_raw_decodes_big_endian(void) {
    setup_healthy();
    assert(mpu6050_init() == true);

    stage_sample(1000, -2000, 8192, 100, -200, 300);

    int16_t accel[3], gyro[3];
    mpu6050_read_raw(accel, gyro);

    assert(accel[0] == 1000);
    assert(accel[1] == -2000);   /* sign must survive the byte assembly */
    assert(accel[2] == 8192);
    assert(gyro[0] == 100);
    assert(gyro[1] == -200);
    assert(gyro[2] == 300);

    printf("PASS test_read_raw_decodes_big_endian\n");
}

static void test_read_raw_skips_temperature_bytes(void) {
    setup_healthy();
    assert(mpu6050_init() == true);

    stage_sample(0, 0, 0, 1, 2, 3);
    /* Poison TEMP_OUT: gyro must not shift into it. */
    fake_bus.mem[MPU6050_REG_ACCEL_XOUT_H + 6] = 0xFF;
    fake_bus.mem[MPU6050_REG_ACCEL_XOUT_H + 7] = 0xFF;

    int16_t accel[3], gyro[3];
    mpu6050_read_raw(accel, gyro);

    assert(gyro[0] == 1 && gyro[1] == 2 && gyro[2] == 3);

    printf("PASS test_read_raw_skips_temperature_bytes\n");
}

static void test_failed_read_yields_zero_vector(void) {
    setup_healthy();
    assert(mpu6050_init() == true);
    stage_sample(1000, 1000, 1000, 500, 500, 500);

    nack_all = true;
    int16_t accel[3] = {9, 9, 9}, gyro[3] = {9, 9, 9};
    mpu6050_read_raw(accel, gyro);

    /* Zeros, not stale values and not garbage: a zero accel norm is what
     * makes MahonyAHRSupdateIMU() skip its correction step and coast on
     * gyro integration instead of snapping attitude to a bogus down. */
    for (int i = 0; i < 3; i++) {
        assert(accel[i] == 0);
        assert(gyro[i] == 0);
    }

    printf("PASS test_failed_read_yields_zero_vector\n");
}

/* ===== Unit conversion ===== */

static void test_convert_units_matches_configured_ranges(void) {
    /* 1 g on Z at +/-4 g full scale is 8192 LSB. */
    const int16_t accel_raw[3] = {0, 0, 8192};
    /* 65.5 LSB per deg/s at +/-500 dps -> 6550 LSB = 100 deg/s. */
    const int16_t gyro_raw[3] = {6550, 0, 0};

    float accel_mps2[3], gyro_rads[3];
    mpu6050_convert_units(accel_raw, gyro_raw, accel_mps2, gyro_rads);

    assert(fabsf(accel_mps2[2] - MPU6050_STANDARD_GRAVITY) < 0.01f);
    assert(fabsf(gyro_rads[0] - (100.0f * MPU6050_DEG_TO_RAD)) < 0.001f);

    printf("PASS test_convert_units_matches_configured_ranges\n");
}

int main(void) {
    test_init_configures_every_register();
    test_init_rejects_wrong_who_am_i();
    test_init_fails_when_bus_nacks();
    test_init_fails_with_no_bus_injected();
    test_read_raw_decodes_big_endian();
    test_read_raw_skips_temperature_bytes();
    test_failed_read_yields_zero_vector();
    test_convert_units_matches_configured_ranges();
    printf("\nAll MPU6050 tests passed.\n");
    return 0;
}
