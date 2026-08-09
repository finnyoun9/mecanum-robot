/**
 * @file ahrs.h
 * @brief Mahony AHRS filter — IMU-only 6-DOF variant (gyro + accel, no magnetometer).
 *
 * Complementary filter that integrates gyro rate and corrects drift using
 * gravity direction sensed by the accelerometer, via proportional-integral
 * feedback on the cross-product error between measured and estimated
 * gravity vectors. Adapted from Mahony, R., Hamel, T., & Pflimlin, J.-M.
 * (2008), "Nonlinear Complementary Filters on the Special Orthogonal Group."
 *
 * Pure C math, no HAL dependency — unit-testable on a host machine
 * (see Test/test_ahrs.c).
 */

#ifndef AHRS_H
#define AHRS_H

/**
 * @brief One filter update step. Integrates gyro rate and applies PI
 *        feedback from the accelerometer-sensed gravity direction.
 *
 * @param gx,gy,gz  Gyroscope reading, rad/s (body frame)
 * @param ax,ay,az  Accelerometer reading, any consistent unit (normalised
 *                  internally, so m/s^2 or g both work)
 * @param q         Quaternion state [w,x,y,z], updated in place
 * @param dt        Sample time in seconds
 *
 * If the accelerometer vector norm is ~0 (free-fall or sensor fault),
 * accel correction is skipped for that step and the gyro is integrated
 * directly.
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float *q, float dt);

/** Reset quaternion to identity (w=1,x=y=z=0) and clear internal PI integral state. */
void MahonyAHRSreset(float *q);

#endif /* AHRS_H */
