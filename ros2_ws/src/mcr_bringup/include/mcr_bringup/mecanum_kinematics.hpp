/**
 * @file mecanum_kinematics.hpp
 * @brief Forward and inverse kinematics for a 4-wheel mecanum drive robot.
 *
 * Wheel layout (top-down view, robot front = +x, left = +y):
 *
 *        FRONT
 *    M1(FL)   M2(FR)
 *      │  ▲ y    │
 *      │  └─x    │
 *      │         │
 *    M3(RL)   M4(RR)
 *        REAR
 *
 * Roller direction: M1/M4 rollers form "/", M2/M3 rollers form "\"
 * (X-pattern — rollers point toward robot centre when viewed from above)
 *
 * Inverse kinematics:  [cmd_vel]  → [ω₁ ω₂ ω₃ ω₄]
 * Forward kinematics:  [ω₁ ω₂ ω₃ ω₄] → [cmd_vel]
 */

#ifndef MCR_BRINGUP__MECANUM_KINEMATICS_HPP_
#define MCR_BRINGUP__MECANUM_KINEMATICS_HPP_

#include <array>
#include <cmath>

namespace mcr_bringup
{

struct MecanumParams
{
  double wheel_radius;       /* Wheel radius (m) */
  double lx;                 /* Half wheelbase: front-rear half-distance */
  double ly;                 /* Half track width: left-right half-distance */
};

struct WheelSpeeds
{
  double w1;  /* Front-left  (rad/s) */
  double w2;  /* Front-right (rad/s) */
  double w3;  /* Rear-left   (rad/s) */
  double w4;  /* Rear-right  (rad/s) */
};

struct Twist2D
{
  double vx;     /* Forward velocity (m/s) */
  double vy;     /* Lateral velocity (m/s, positive = left) */
  double omega;  /* Angular velocity (rad/s, CCW positive) */
};

class MecanumKinematics
{
public:
  explicit MecanumKinematics(const MecanumParams & params);

  /**
   * @brief Inverse kinematics: desired twist → 4 wheel speeds.
   *
   *   ω₁ = (vx - vy - (lx+ly)·ω) / R
   *   ω₂ = (vx + vy + (lx+ly)·ω) / R
   *   ω₃ = (vx + vy - (lx+ly)·ω) / R
   *   ω₄ = (vx - vy + (lx+ly)·ω) / R
   */
  WheelSpeeds inverse(const Twist2D & twist) const;

  /**
   * @brief Forward kinematics: 4 wheel speeds → estimated twist.
   *
   *   vx   = ( ω₁ + ω₂ + ω₃ + ω₄) · R / 4
   *   vy   = (-ω₁ + ω₂ + ω₃ - ω₄) · R / 4
   *   ω    = (-ω₁ + ω₂ - ω₃ + ω₄) · R / (4·(lx+ly))
   */
  Twist2D forward(const WheelSpeeds & speeds) const;

  /** Maximum single-wheel speed for a given twist (used for saturation). */
  double max_wheel_speed(const Twist2D & twist) const;

  /** Scale all wheel speeds uniformly so none exceed `limit`. */
  static WheelSpeeds scale_to_limit(const WheelSpeeds & speeds, double limit);

  const MecanumParams & params() const { return params_; }

private:
  MecanumParams params_;
  double l_sum_;   /* lx + ly (cached) */
  double inv_r_;   /* 1 / wheel_radius (cached) */
};

}  // namespace mcr_bringup

#endif  // MCR_BRINGUP__MECANUM_KINEMATICS_HPP_
