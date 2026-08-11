/**
 * @file mecanum_ik.h
 * @brief Inverse kinematics for 4-wheel mecanum drive (C port of the
 *        ROS2-side MecanumKinematics class in mcr_bringup).
 *
 * Pure C, no HAL/RTOS deps — host-buildable for unit tests.
 *
 * Wheel order matches motor_id_t in motor.h:
 *   w[0]=FL, w[1]=FR, w[2]=RL, w[3]=RR
 */

#ifndef MECANUM_IK_H
#define MECANUM_IK_H

#include <stdbool.h>
#include <stdint.h>

#define MECANUM_WHEEL_COUNT 4

typedef struct {
    float wheel_radius;  /* Wheel radius (m) */
    float lx;            /* Half wheelbase: front-rear half-distance (m) */
    float ly;            /* Half track width: left-right half-distance (m) */
} mecanum_ik_config_t;

/**
 * @brief Inverse kinematics: desired twist → 4 wheel angular velocities.
 *
 *   w1(FL) = (vx - vy - (lx+ly)·omega) / R
 *   w2(FR) = (vx + vy + (lx+ly)·omega) / R
 *   w3(RL) = (vx + vy - (lx+ly)·omega) / R
 *   w4(RR) = (vx - vy + (lx+ly)·omega) / R
 *
 * @param cfg    Wheel geometry
 * @param vx     Forward velocity (m/s)
 * @param vy     Lateral velocity (m/s, positive = left)
 * @param omega  Angular velocity (rad/s, CCW positive)
 * @param w      Output: 4 wheel speeds in rad/s
 */
void mecanum_ik(const mecanum_ik_config_t *cfg, float vx, float vy,
                float omega, float w[MECANUM_WHEEL_COUNT]);

/**
 * @brief Uniformly scale wheel speeds so none exceeds `limit` (rad/s).
 * Used to saturate a twist whose corner wheel speed would exceed the
 * controller's output range.
 */
void mecanum_ik_scale_to_limit(float w[MECANUM_WHEEL_COUNT], float limit);

#endif /* MECANUM_IK_H */
