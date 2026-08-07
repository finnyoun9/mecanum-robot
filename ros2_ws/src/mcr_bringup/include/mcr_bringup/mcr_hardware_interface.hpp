/**
 * @file mcr_hardware_interface.hpp
 * @brief ros2_control SystemInterface for the Mecanum Robot.
 *
 * Exports:
 *   - 4 velocity command interfaces (wheel_FL, wheel_FR, wheel_RL, wheel_RR)
 *   - 4 velocity state interfaces  (same joints)
 *   - 2 sensor interfaces (imu, tof)
 *
 * Communication: UART via SerialProtocol → STM32 FreeRTOS firmware.
 * Mecanum IK/forward kinematics performed here so ros2_control sees
 * individual wheel joints.
 */

#ifndef MCR_BRINGUP__MCR_HARDWARE_INTERFACE_HPP_
#define MCR_BRINGUP__MCR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mcr_bringup/serial_protocol.hpp"
#include "mcr_bringup/mecanum_kinematics.hpp"

namespace mcr_bringup
{

class MCRHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MCRHardwareInterface();
  ~MCRHardwareInterface() override;

  /* SystemInterface overrides */
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /* --- Serial --- */
  std::unique_ptr<SerialProtocol> serial_;
  std::string serial_device_;
  int serial_baud_;

  /* --- Kinematics --- */
  MecanumParams mecanum_params_;
  std::unique_ptr<MecanumKinematics> kinematics_;

  /* --- Joint state --- */
  struct JointState
  {
    double position;   /* Cumulative radians */
    double velocity;   /* rad/s */
    double command;    /* Target velocity rad/s (from controller) */
  };
  std::vector<JointState> joints_;

  /* --- Sensor state --- */
  struct SensorState
  {
    /* IMU */
    double imu_orientation[4];  /* quaternion w,x,y,z */
    double imu_angular_vel[3]; /* rad/s */

    /* ToF */
    double tof_range;           /* metres */

    /* Battery */
    double battery_voltage;     /* volts */
  } sensors_;

  /* --- Derived odometry (from wheel velocities) --- */
  double odom_x_;
  double odom_y_;
  double odom_yaw_;

  /* --- Timing --- */
  rclcpp::Time last_read_time_;
  bool first_read_;
};

}  // namespace mcr_bringup

#endif  // MCR_BRINGUP__MCR_HARDWARE_INTERFACE_HPP_
