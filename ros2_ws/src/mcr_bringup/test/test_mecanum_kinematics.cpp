/**
 * @file test_mecanum_kinematics.cpp
 * @brief Unit tests for mecanum wheel forward/inverse kinematics and scaling.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mcr_bringup/mecanum_kinematics.hpp"

using mcr_bringup::MecanumKinematics;
using mcr_bringup::MecanumParams;
using mcr_bringup::Twist2D;
using mcr_bringup::WheelSpeeds;

namespace
{
constexpr double kEps = 1e-9;

MecanumParams test_params()
{
  MecanumParams p;
  p.wheel_radius = 0.0325;
  p.lx = 0.0625;
  p.ly = 0.09;
  return p;
}
}  // namespace

class MecanumKinematicsTest : public ::testing::Test
{
protected:
  MecanumKinematicsTest()
  : kin_(test_params())
  {}

  MecanumKinematics kin_;
};

TEST_F(MecanumKinematicsTest, RoundTripPureForward)
{
  Twist2D t{0.3, 0.0, 0.0};
  Twist2D t2 = kin_.forward(kin_.inverse(t));
  EXPECT_NEAR(t2.vx, t.vx, kEps);
  EXPECT_NEAR(t2.vy, t.vy, kEps);
  EXPECT_NEAR(t2.omega, t.omega, kEps);
}

TEST_F(MecanumKinematicsTest, RoundTripPureLateral)
{
  Twist2D t{0.0, 0.2, 0.0};
  Twist2D t2 = kin_.forward(kin_.inverse(t));
  EXPECT_NEAR(t2.vx, t.vx, kEps);
  EXPECT_NEAR(t2.vy, t.vy, kEps);
  EXPECT_NEAR(t2.omega, t.omega, kEps);
}

TEST_F(MecanumKinematicsTest, RoundTripPureRotation)
{
  Twist2D t{0.0, 0.0, 0.8};
  Twist2D t2 = kin_.forward(kin_.inverse(t));
  EXPECT_NEAR(t2.vx, t.vx, kEps);
  EXPECT_NEAR(t2.vy, t.vy, kEps);
  EXPECT_NEAR(t2.omega, t.omega, kEps);
}

TEST_F(MecanumKinematicsTest, RoundTripCombinedMotion)
{
  Twist2D t{0.25, -0.15, 0.4};
  Twist2D t2 = kin_.forward(kin_.inverse(t));
  EXPECT_NEAR(t2.vx, t.vx, kEps);
  EXPECT_NEAR(t2.vy, t.vy, kEps);
  EXPECT_NEAR(t2.omega, t.omega, kEps);
}

// Per the implementation's inverse():
//   w1 = (vx - vy - l*om) / R   w4 = (vx - vy + l*om) / R   -> both carry -vy
//   w2 = (vx + vy + l*om) / R   w3 = (vx + vy - l*om) / R   -> both carry +vy
// so for pure lateral motion with vy > 0, w1/w4 must be negative and
// w2/w3 must be positive.
TEST_F(MecanumKinematicsTest, PureLateralWheelSignPattern)
{
  Twist2D t{0.0, 0.2, 0.0};
  WheelSpeeds ws = kin_.inverse(t);

  EXPECT_LT(ws.w1, 0.0);
  EXPECT_GT(ws.w2, 0.0);
  EXPECT_GT(ws.w3, 0.0);
  EXPECT_LT(ws.w4, 0.0);

  // With omega=0 and symmetric params, same-sign pairs match in magnitude.
  EXPECT_NEAR(ws.w1, ws.w4, kEps);
  EXPECT_NEAR(ws.w2, ws.w3, kEps);
  EXPECT_NEAR(std::abs(ws.w1), std::abs(ws.w2), kEps);
}

TEST_F(MecanumKinematicsTest, PureRotationEqualMagnitude)
{
  Twist2D t{0.0, 0.0, 0.8};
  WheelSpeeds ws = kin_.inverse(t);

  double m1 = std::abs(ws.w1);
  double m2 = std::abs(ws.w2);
  double m3 = std::abs(ws.w3);
  double m4 = std::abs(ws.w4);

  EXPECT_GT(m1, 0.0);
  EXPECT_NEAR(m1, m2, kEps);
  EXPECT_NEAR(m1, m3, kEps);
  EXPECT_NEAR(m1, m4, kEps);
}

TEST(MecanumScaleToLimit, ScalesDownToLimitPreservingRatios)
{
  WheelSpeeds in{10.0, -20.0, 15.0, -5.0};
  const double limit = 5.0;

  WheelSpeeds out = MecanumKinematics::scale_to_limit(in, limit);

  double max_out = std::max(
    {std::abs(out.w1), std::abs(out.w2), std::abs(out.w3), std::abs(out.w4)});
  EXPECT_NEAR(max_out, limit, kEps);

  // Uniform scale factor -> ratio between each wheel and its input preserved.
  double scale = out.w1 / in.w1;
  EXPECT_NEAR(out.w2 / in.w2, scale, kEps);
  EXPECT_NEAR(out.w3 / in.w3, scale, kEps);
  EXPECT_NEAR(out.w4 / in.w4, scale, kEps);
}
