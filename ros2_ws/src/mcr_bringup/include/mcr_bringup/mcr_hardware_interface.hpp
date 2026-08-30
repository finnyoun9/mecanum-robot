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

// The export_state_interfaces()/export_command_interfaces() overrides below
// use the legacy ros2_control export API, which Jazzy marks [[deprecated]] in
// favour of on_export_state_interfaces(). ResourceManager still dispatches the
// legacy entry point first and only falls back to the on_export_* methods when
// it returns an empty list, so overriding it is the supported backward-compat
// path. Silence the deprecation warnings accordingly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#pragma GCC diagnostic pop

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

  /* SystemInterface lifecycle (ros2_control >= 4.x API) */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /* Interface export (legacy API, still dispatched in Jazzy) */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#pragma GCC diagnostic pop

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
    double imu_linear_accel[3]; /* m/s² (zeros until firmware streams accel) */

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

  /* --- Serial link health --- */
  /* A single failed write is backpressure, not a dead link; only a
   * sustained run of them deactivates the component. At the 100 Hz
   * control rate 100 failures is ~1 s. */
  static constexpr int MAX_CONSECUTIVE_WRITE_FAILURES = 100;
  int consecutive_write_failures_;

  /* --- Command send throttling ---
   * write() runs every control cycle (100 Hz). Sending on every cycle
   * unconditionally means the Pi silently outguns any other writer of
   * the STM32's shared target_w[] -- the NRF24 handset only sends at
   * 10 Hz, so an idle Pi (nothing publishing to /cmd_vel, command
   * sitting at zero) permanently stomped the handset's joystick input
   * back to zero every ~10 ms. Only resend on an actual command change,
   * or periodically as a keep-alive comfortably under the firmware's
   * COMM_WATCHDOG_MS (250 ms) so a genuinely held Pi-driven command
   * (e.g. a steady Nav2 cruise) doesn't itself starve the watchdog. */
  double last_sent_w_[4];
  rclcpp::Time last_sent_time_;
  bool has_sent_once_;
  static constexpr double COMMAND_CHANGE_EPS = 1e-4;
  static constexpr double KEEPALIVE_INTERVAL_S = 0.1;
};

}  // namespace mcr_bringup

#endif  // MCR_BRINGUP__MCR_HARDWARE_INTERFACE_HPP_
