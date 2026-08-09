/**
 * @file ahrs.c
 * @brief Mahony AHRS filter implementation (IMU-only 6-DOF).
 *
 * Kp/Ki are tunable proportional/integral gains for the accel-correction
 * feedback loop. Kp=2.0f, Ki=0.005f below are typical starting points for a
 * hobby-grade IMU (per Mahony et al. 2008 and common open-source ports of
 * the algorithm) — tune against actual sensor noise once hardware arrives.
 */

#include "ahrs.h"
#include <math.h>

static float Kp = 2.0f;
static float Ki = 0.005f;

/* Integral feedback terms — persist across calls to correct sustained bias */
static float g_ix = 0.0f, g_iy = 0.0f, g_iz = 0.0f;

void MahonyAHRSreset(float *q) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    g_ix = g_iy = g_iz = 0.0f;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float *q, float dt) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    /* Only apply accel correction if the reading is usable (skip on
     * free-fall / sensor fault, where norm collapses to ~0) */
    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm > 1e-6f) {
        ax /= accel_norm;
        ay /= accel_norm;
        az /= accel_norm;

        /* Estimated gravity direction in body frame, from current quaternion */
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Error: cross product between measured and estimated gravity */
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        /* Integral feedback (corrects sustained gyro bias) */
        if (Ki > 0.0f) {
            g_ix += Ki * ex * dt;
            g_iy += Ki * ey * dt;
            g_iz += Ki * ez * dt;
            gx += g_ix;
            gy += g_iy;
            gz += g_iz;
        } else {
            g_ix = g_iy = g_iz = 0.0f;
        }

        /* Proportional feedback */
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    /* Integrate rate of change of quaternion: qdot = 0.5 * q (x) [0,gx,gy,gz] */
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * (0.5f * dt);
    q1 += (qa * gx + qc * gz - q3 * gy) * (0.5f * dt);
    q2 += (qa * gy - qb * gz + q3 * gx) * (0.5f * dt);
    q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * dt);

    /* Normalise quaternion */
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        q[0] = q0 * inv;
        q[1] = q1 * inv;
        q[2] = q2 * inv;
        q[3] = q3 * inv;
    } else {
        /* Degenerate case (shouldn't happen from a valid unit start) — reset */
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}
