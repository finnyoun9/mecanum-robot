/**
 * @file mecanum_ik.c
 * @brief Inverse kinematics for a 4-wheel mecanum drive robot.
 *
 * Wheel layout (top-down, robot front = +x, left = +y), matching
 * mcr.urdf.xacro and the ROS2-side MecanumKinematics class:
 *
 *        FRONT
 *    M1(FL)   M2(FR)
 *    M3(RL)   M4(RR)
 *        REAR
 *
 * Roller direction is the X-pattern: M1/M4 rollers "/", M2/M3 "\".
 */

#include "mecanum_ik.h"

#include <math.h>

void mecanum_ik(const mecanum_ik_config_t *cfg, float vx, float vy,
                float omega, float w[MECANUM_WHEEL_COUNT])
{
    float l = cfg->lx + cfg->ly;
    float inv_r = 1.0f / cfg->wheel_radius;

    w[0] = (vx - vy - l * omega) * inv_r;  /* FL */
    w[1] = (vx + vy + l * omega) * inv_r;  /* FR */
    w[2] = (vx + vy - l * omega) * inv_r;  /* RL */
    w[3] = (vx - vy + l * omega) * inv_r;  /* RR */
}

void mecanum_ik_scale_to_limit(float w[MECANUM_WHEEL_COUNT], float limit)
{
    float max_w = 0.0f;
    for (int i = 0; i < MECANUM_WHEEL_COUNT; i++) {
        float a = fabsf(w[i]);
        if (a > max_w) max_w = a;
    }

    if (max_w <= limit) return;

    float scale = limit / max_w;
    for (int i = 0; i < MECANUM_WHEEL_COUNT; i++) {
        w[i] *= scale;
    }
}
