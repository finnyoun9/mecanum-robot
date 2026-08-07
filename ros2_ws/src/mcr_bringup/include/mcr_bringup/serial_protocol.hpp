/**
 * @file serial_protocol.hpp
 * @brief C++ wrapper for the shared protocol (Pi side).
 *
 * Wraps the C protocol.h/c for use in ROS2 C++ nodes.
 * Handles serial port I/O via termios (Linux).
 */

#ifndef MCR_BRINGUP__SERIAL_PROTOCOL_HPP_
#define MCR_BRINGUP__SERIAL_PROTOCOL_HPP_

#include <string>
#include <cstdint>
#include <vector>

extern "C" {
#include "protocol.h"
}

namespace mcr_bringup
{

/**
 * @brief RAII serial port wrapper with protocol framing.
 */
class SerialProtocol
{
public:
  SerialProtocol();
  ~SerialProtocol();

  /**
   * @brief Open serial port.
   * @param device   e.g. "/dev/ttyAMA0" or "/dev/ttyUSB0"
   * @param baudrate e.g. B115200, B921600
   * @return true on success
   */
  bool open(const std::string & device, int baudrate);

  /** Close port. */
  void close();

  /** True if port is open. */
  bool is_open() const { return fd_ >= 0; }

  /**
   * @brief Send velocity command to STM32.
   * Encodes CMD_VEL_CTRL frame and writes to UART.
   */
  bool send_velocity_command(float w1, float w2, float w3, float w4);

  /** Send emergency stop command. */
  bool send_emergency_stop();

  /**
   * @brief Read and parse incoming frames.
   * Blocks up to timeout_ms. Returns true if valid CMD_ODOM_FEEDBACK decoded.
   * Populates `odom` with decoded data.
   */
  bool read_odometry(odom_feedback_t & odom, int timeout_ms = 10);

private:
  /** Write raw bytes to serial. */
  bool write_bytes(const uint8_t *data, size_t len);

  /** Read raw bytes from serial (non-blocking). Returns bytes read. */
  int read_bytes(uint8_t *buf, size_t max_len);

  /** Write a protocol frame. */
  bool send_frame(uint8_t cmd, const uint8_t *payload, uint8_t pay_len);

  int fd_;
  uint8_t seq_tx_;
  uint8_t seq_rx_;

  /* RX buffer */
  std::vector<uint8_t> rx_buf_;
  size_t rx_buf_len_;
};

}  // namespace mcr_bringup

#endif  // MCR_BRINGUP__SERIAL_PROTOCOL_HPP_
