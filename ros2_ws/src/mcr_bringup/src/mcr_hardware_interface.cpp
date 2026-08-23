/**
 * @file mcr_hardware_interface.cpp
 * @brief ros2_control hardware interface — bridges ROS2 ↔ STM32 via UART.
 */

#include "mcr_bringup/mcr_hardware_interface.hpp"

#include <cmath>
#include <cstring>
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"

namespace mcr_bringup
{

/* Defaults — override via URDF ros2_control xacro params */
static constexpr double WHEEL_RADIUS_DEFAULT  = 0.030;    /* 30mm radius (60mm mecanum wheels, measured) */
static constexpr double LX_DEFAULT            = 0.10;     /* Half wheelbase front-rear */
static constexpr double LY_DEFAULT            = 0.12;     /* Half track-width left-right */
static constexpr int    SERIAL_BAUD_DEFAULT   = 921600;

/* Encoder edges per wheel revolution — must match firmware encoder.h.
   Measured 2026-08-23 by hand-turning a wheel 10 revolutions: 224 edges/rev
   at 1x, doubled to 448 when the firmware moved to decoding both edges of
   channel A. Was 11 * 4 * 34 = 1496, derived from datasheet-typical
   constants that proved wrong twice over (gearbox is ~1:20, and software
   decode does not give 4x). Odometry scales directly off this, so a wrong
   value here misreports distance by that same factor. */
static constexpr double EDGES_PER_WHEEL_REV = 448.0;

MCRHardwareInterface::MCRHardwareInterface()
: serial_device_("/dev/ttyAMA10")   /* Pi 5 hardware UART (see docker/run.sh) */
, serial_baud_(SERIAL_BAUD_DEFAULT)
, joints_(4)
, odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
, first_read_(true)
{
  mecanum_params_.wheel_radius = WHEEL_RADIUS_DEFAULT;
  mecanum_params_.lx = LX_DEFAULT;
  mecanum_params_.ly = LY_DEFAULT;
}

MCRHardwareInterface::~MCRHardwareInterface()
{
  if (serial_ && serial_->is_open()) {
    serial_->send_emergency_stop();
  }
}

hardware_interface::CallbackReturn MCRHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto & info = params.hardware_info;

  /* Read parameters from hardware info */
  for (const auto & param : info.hardware_parameters) {
    if (param.first == "serial_device")
      serial_device_ = param.second;
    else if (param.first == "serial_baud")
      serial_baud_ = std::stoi(param.second);
    else if (param.first == "wheel_radius")
      mecanum_params_.wheel_radius = std::stod(param.second);
    else if (param.first == "lx")
      mecanum_params_.lx = std::stod(param.second);
    else if (param.first == "ly")
      mecanum_params_.ly = std::stod(param.second);
  }

  RCLCPP_INFO(rclcpp::get_logger("MCRHardwareInterface"),
    "Config: device=%s, baud=%d, R=%.4f, lx=%.4f, ly=%.4f",
    serial_device_.c_str(), serial_baud_,
    mecanum_params_.wheel_radius, mecanum_params_.lx, mecanum_params_.ly);

  kinematics_ = std::make_unique<MecanumKinematics>(mecanum_params_);

  /* Open serial */
  serial_ = std::make_unique<SerialProtocol>();
  if (!serial_->open(serial_device_, serial_baud_)) {
    RCLCPP_ERROR(rclcpp::get_logger("MCRHardwareInterface"),
      "Failed to open serial port: %s", serial_device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MCRHardwareInterface"),
    "Serial port opened: %s", serial_device_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<hardware_interface::StateInterface>
MCRHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;

  /* 4 wheel joints: position + velocity state */
  const std::array<const char *, 4> joint_names = {
    "wheel_front_left_joint", "wheel_front_right_joint",
    "wheel_rear_left_joint",  "wheel_rear_right_joint"
  };

  for (size_t i = 0; i < 4; ++i) {
    ifaces.emplace_back(joint_names[i],
      hardware_interface::HW_IF_POSITION,  &joints_[i].position);
    ifaces.emplace_back(joint_names[i],
      hardware_interface::HW_IF_VELOCITY,  &joints_[i].velocity);
  }

  /* IMU sensor */
  ifaces.emplace_back("imu_sensor", "orientation.w",
    &sensors_.imu_orientation[0]);
  ifaces.emplace_back("imu_sensor", "orientation.x",
    &sensors_.imu_orientation[1]);
  ifaces.emplace_back("imu_sensor", "orientation.y",
    &sensors_.imu_orientation[2]);
  ifaces.emplace_back("imu_sensor", "orientation.z",
    &sensors_.imu_orientation[3]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.x",
    &sensors_.imu_angular_vel[0]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.y",
    &sensors_.imu_angular_vel[1]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.z",
    &sensors_.imu_angular_vel[2]);
  ifaces.emplace_back("imu_sensor", "linear_acceleration.x",
    &sensors_.imu_linear_accel[0]);
  ifaces.emplace_back("imu_sensor", "linear_acceleration.y",
    &sensors_.imu_linear_accel[1]);
  ifaces.emplace_back("imu_sensor", "linear_acceleration.z",
    &sensors_.imu_linear_accel[2]);

  /* ToF sensor */
  ifaces.emplace_back("tof_sensor", "range", &sensors_.tof_range);

  return ifaces;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<hardware_interface::CommandInterface>
MCRHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;

  const std::array<const char *, 4> joint_names = {
    "wheel_front_left_joint", "wheel_front_right_joint",
    "wheel_rear_left_joint",  "wheel_rear_right_joint"
  };

  for (size_t i = 0; i < 4; ++i) {
    ifaces.emplace_back(joint_names[i],
      hardware_interface::HW_IF_VELOCITY, &joints_[i].command);
  }

  return ifaces;
}
#pragma GCC diagnostic pop

hardware_interface::CallbackReturn MCRHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Clear stale state */
  for (auto & j : joints_) {
    j.position = 0.0;
    j.velocity = 0.0;
    j.command  = 0.0;
  }
  std::memset(&sensors_, 0, sizeof(sensors_));
  sensors_.imu_orientation[0] = 1.0; /* Identity quaternion */

  first_read_ = true;

  RCLCPP_INFO(rclcpp::get_logger("MCRHardwareInterface"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MCRHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  /* Send emergency stop and zero commands */
  if (serial_) serial_->send_emergency_stop();
  for (auto & j : joints_) j.command = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("MCRHardwareInterface"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MCRHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!serial_ || !serial_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  /* Read odometry from STM32 */
  odom_feedback_t odom;
  if (serial_->read_odometry(odom, 5)) {
    /* Convert encoder counts → joint position (radians) */
    for (int i = 0; i < 4; ++i) {
      double revs = static_cast<double>(odom.encoder_counts[i]) / EDGES_PER_WHEEL_REV;
      joints_[i].position = revs * 2.0 * M_PI;
    }

    /* IMU data */
    for (int i = 0; i < 4; ++i) sensors_.imu_orientation[i] = odom.imu_q[i];
    for (int i = 0; i < 3; ++i) sensors_.imu_angular_vel[i] = odom.imu_gyro[i];

    /* ToF */
    sensors_.tof_range = odom.tof_distance_mm * 0.001; /* mm → m */

    /* Battery */
    sensors_.battery_voltage = odom.battery_pct * 0.01 * 12.6; /* Approx from % */
  }

  /* Compute joint velocities from position delta (if no direct velocity from MCU) */
  if (!first_read_) {
    double dt = period.seconds();
    if (dt > 0.0) {
      static double prev_pos[4] = {0};
      for (int i = 0; i < 4; ++i) {
        joints_[i].velocity = (joints_[i].position - prev_pos[i]) / dt;
        prev_pos[i] = joints_[i].position;
      }
    }
  } else {
    first_read_ = false;
  }

  /* Forward kinematics for odometry */
  WheelSpeeds ws;
  ws.w1 = joints_[0].velocity;
  ws.w2 = joints_[1].velocity;
  ws.w3 = joints_[2].velocity;
  ws.w4 = joints_[3].velocity;

  Twist2D twist = kinematics_->forward(ws);

  double dt = period.seconds();
  if (dt > 0.0) {
    /* Dead-reckoning: integrate twist → pose */
    double d_x = twist.vx * dt;
    double d_y = twist.vy * dt;
    double d_yaw = twist.omega * dt;

    /* Rotate body-frame displacement to world frame */
    double cos_t = std::cos(odom_yaw_);
    double sin_t = std::sin(odom_yaw_);
    odom_x_   += d_x * cos_t - d_y * sin_t;
    odom_y_   += d_x * sin_t + d_y * cos_t;
    odom_yaw_ += d_yaw;

    /* Normalise yaw */
    odom_yaw_ = angles::normalize_angle(odom_yaw_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MCRHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  if (!serial_ || !serial_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  /* Apply mecanum inverse kinematics if using a twist-based controller.
   * When using individual wheel velocity commands (e.g. from mecanum_drive_controller),
   * the controller already computed per-wheel targets.
   * Here we pass them directly: */
  float w1 = static_cast<float>(joints_[0].command);
  float w2 = static_cast<float>(joints_[1].command);
  float w3 = static_cast<float>(joints_[2].command);
  float w4 = static_cast<float>(joints_[3].command);

  if (!serial_->send_velocity_command(w1, w2, w3, w4)) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MCRHardwareInterface"),
      *rclcpp::Clock::make_shared(), 1000,
      "Failed to send velocity command via serial");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mcr_bringup

PLUGINLIB_EXPORT_CLASS(
  mcr_bringup::MCRHardwareInterface,
  hardware_interface::SystemInterface)
