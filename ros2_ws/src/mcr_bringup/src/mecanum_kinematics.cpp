/**
 * @file mecanum_kinematics.cpp
 * @brief Implementation of mecanum wheel kinematics.
 */

#include "mcr_bringup/mecanum_kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace mcr_bringup
{

MecanumKinematics::MecanumKinematics(const MecanumParams & params)
: params_(params)
{
  l_sum_ = params_.lx + params_.ly;
  inv_r_ = 1.0 / params_.wheel_radius;
}

WheelSpeeds MecanumKinematics::inverse(const Twist2D & twist) const
{
  WheelSpeeds ws;
  double vx = twist.vx;
  double vy = twist.vy;
  double om = twist.omega;
  double l  = l_sum_;

  ws.w1 = (vx - vy - l * om) * inv_r_;
  ws.w2 = (vx + vy + l * om) * inv_r_;
  ws.w3 = (vx + vy - l * om) * inv_r_;
  ws.w4 = (vx - vy + l * om) * inv_r_;

  return ws;
}

Twist2D MecanumKinematics::forward(const WheelSpeeds & speeds) const
{
  Twist2D t;
  double r = params_.wheel_radius;
  double w1 = speeds.w1, w2 = speeds.w2;
  double w3 = speeds.w3, w4 = speeds.w4;

  t.vx    = ( w1 + w2 + w3 + w4) * r * 0.25;
  t.vy    = (-w1 + w2 + w3 - w4) * r * 0.25;
  t.omega = (-w1 + w2 - w3 + w4) * r / (4.0 * l_sum_);

  return t;
}

double MecanumKinematics::max_wheel_speed(const Twist2D & twist) const
{
  WheelSpeeds ws = inverse(twist);
  double m = std::abs(ws.w1);
  m = std::max(m, std::abs(ws.w2));
  m = std::max(m, std::abs(ws.w3));
  m = std::max(m, std::abs(ws.w4));
  return m;
}

WheelSpeeds MecanumKinematics::scale_to_limit(const WheelSpeeds & speeds, double limit)
{
  double m = std::abs(speeds.w1);
  m = std::max(m, std::abs(speeds.w2));
  m = std::max(m, std::abs(speeds.w3));
  m = std::max(m, std::abs(speeds.w4));

  if (m <= limit) return speeds;

  double scale = limit / m;
  WheelSpeeds out;
  out.w1 = speeds.w1 * scale;
  out.w2 = speeds.w2 * scale;
  out.w3 = speeds.w3 * scale;
  out.w4 = speeds.w4 * scale;
  return out;
}

}  // namespace mcr_bringup
