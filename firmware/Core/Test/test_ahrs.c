/**
 * @file test_ahrs.c
 * @brief Host-buildable unit tests for the Mahony AHRS filter (ahrs.c).
 *
 * No test framework — plain main() with assert(). Pure C math, so this
 * builds and runs on a host machine without any STM32/FreeRTOS deps.
 *
 * Build & run (from this directory):
 *   gcc -Wall -Wextra -std=c99 -I../Inc -lm test_ahrs.c ../Src/ahrs.c -o test_ahrs && ./test_ahrs
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "ahrs.h"

#define DT 0.01f /* 100 Hz, matches SensorTask period */

static float quat_norm(const float *q) {
    return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/* (a) Stationary: zero gyro, accel az=+1 (matches the identity-attitude
 *     gravity reference used internally), starting from identity — the
 *     filter should stay at identity since there's no error to correct. */
static void test_stationary_identity(void) {
    float q[4];
    MahonyAHRSreset(q);

    for (int i = 0; i < 200; i++) {
        MahonyAHRSupdateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, q, DT);
    }

    assert(fabsf(q[0] - 1.0f) < 1e-3f);
    assert(fabsf(q[1]) < 1e-3f);
    assert(fabsf(q[2]) < 1e-3f);
    assert(fabsf(q[3]) < 1e-3f);
    printf("PASS test_stationary_identity        q=[%+.6f %+.6f %+.6f %+.6f]\n",
           q[0], q[1], q[2], q[3]);
}

/* (b) Quaternion stays normalised (|q| ~= 1) after many iterations, even
 *     under continuous rotation + off-axis accel correction. */
static void test_stays_normalized(void) {
    float q[4];
    MahonyAHRSreset(q);

    for (int i = 0; i < 5000; i++) {
        MahonyAHRSupdateIMU(0.5f, -0.3f, 0.2f, 0.1f, -0.1f, 0.98f, q, DT);
        float n = quat_norm(q);
        assert(fabsf(n - 1.0f) < 1e-3f);
    }
    printf("PASS test_stays_normalized           final |q|=%.8f\n", quat_norm(q));
}

/* (c) Constant gyro rotation about Z, with accel held at the identity
 *     reference [0,0,1]. Gravity's direction in the body frame is
 *     invariant under pure yaw (rotation about the same axis gravity
 *     points along), so this feeds ~zero correction torque for the whole
 *     run — i.e. it isolates pure gyro integration and checks the sign/
 *     axis convention matches the expected rotation direction. */
static void test_gyro_rotation_direction(void) {
    float q[4];
    MahonyAHRSreset(q);

    const float gz = 1.0f; /* rad/s about Z */
    const int steps = 100; /* 1 second total */
    for (int i = 0; i < steps; i++) {
        MahonyAHRSupdateIMU(0.0f, 0.0f, gz, 0.0f, 0.0f, 1.0f, q, DT);
    }

    /* Expected: q ~= [cos(theta/2), 0, 0, sin(theta/2)], theta = gz * t */
    float theta = gz * steps * DT;
    float expected_w = cosf(theta / 2.0f);
    float expected_z = sinf(theta / 2.0f);

    printf("     test_gyro_rotation_direction    q=[%+.6f %+.6f %+.6f %+.6f] expected w=%+.6f z=%+.6f\n",
           q[0], q[1], q[2], q[3], expected_w, expected_z);

    assert(q[3] > 0.3f);         /* rotated positively about Z, as expected for +gz */
    assert(fabsf(q[1]) < 0.05f); /* negligible roll  */
    assert(fabsf(q[2]) < 0.05f); /* negligible pitch */
    assert(fabsf(q[0] - expected_w) < 0.02f);
    assert(fabsf(q[3] - expected_z) < 0.02f);
    printf("PASS test_gyro_rotation_direction\n");
}

int main(void) {
    test_stationary_identity();
    test_stays_normalized();
    test_gyro_rotation_direction();
    printf("All AHRS tests passed.\n");
    return 0;
}
