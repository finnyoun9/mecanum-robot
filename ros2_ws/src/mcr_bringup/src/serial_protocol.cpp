/**
 * @file serial_protocol.cpp
 * @brief Serial protocol implementation — termios + shared protocol framing.
 */

#include "mcr_bringup/serial_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>

namespace mcr_bringup
{

SerialProtocol::SerialProtocol()
: fd_(-1), seq_tx_(0), seq_rx_(0), rx_buf_(256), rx_buf_len_(0)
{}

SerialProtocol::~SerialProtocol()
{
  close();
}

bool SerialProtocol::open(const std::string & device, int baudrate)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  cfsetospeed(&tty, baudrate);
  cfsetispeed(&tty, baudrate);

  /* 8N1, raw mode */
  tty.c_cflag = CS8 | CREAD | CLOCAL;
  tty.c_iflag = IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  /* Non-blocking read: VMIN=0, VTIME=0 */
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &tty);

  return true;
}

void SerialProtocol::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialProtocol::write_bytes(const uint8_t *data, size_t len)
{
  if (fd_ < 0) return false;
  ssize_t written = ::write(fd_, data, len);
  return (written == static_cast<ssize_t>(len));
}

int SerialProtocol::read_bytes(uint8_t *buf, size_t max_len)
{
  if (fd_ < 0) return -1;
  ssize_t n = ::read(fd_, buf, max_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  return static_cast<int>(n);
}

bool SerialProtocol::send_frame(uint8_t cmd, const uint8_t *payload, uint8_t pay_len)
{
  uint8_t frame[PROTO_MAX_FRAME];
  uint8_t frame_len;
  int ret = proto_encode(cmd, payload, pay_len, frame, &frame_len, seq_tx_++);
  if (ret < 0) return false;
  return write_bytes(frame, frame_len);
}

bool SerialProtocol::send_velocity_command(float w1, float w2, float w3, float w4)
{
  cmd_vel_ctrl_t vel;
  vel.w1 = w1;
  vel.w2 = w2;
  vel.w3 = w3;
  vel.w4 = w4;
  return send_frame(CMD_VEL_CTRL, reinterpret_cast<const uint8_t *>(&vel), sizeof(vel));
}

bool SerialProtocol::send_emergency_stop()
{
  return send_frame(CMD_EMERGENCY_STOP, nullptr, 0);
}

bool SerialProtocol::read_odometry(odom_feedback_t & odom, int timeout_ms)
{
  /* In a full implementation, we'd use select() with the timeout.
   * Here we do a simple non-blocking read loop. */

  /* Read available bytes into rx buffer */
  uint8_t tmp[128];
  int n = read_bytes(tmp, sizeof(tmp));
  if (n < 0) return false;

  /* Append to ring buffer */
  for (int i = 0; i < n; i++) {
    if (rx_buf_len_ < rx_buf_.size()) {
      rx_buf_[rx_buf_len_++] = tmp[i];
    } else {
      /* Ring buffer full — shift left (simple, not optimal) */
      std::memmove(rx_buf_.data(), rx_buf_.data() + 1, rx_buf_.size() - 1);
      rx_buf_[rx_buf_.size() - 1] = tmp[i];
    }
  }

  /* Try to parse frames from buffer */
  if (rx_buf_len_ < PROTO_FRAME_OVERHEAD) return false;

  uint8_t cmd, payload[PROTO_MAX_PAYLOAD], pay_len, seq;
  int ret = proto_decode(rx_buf_.data(), static_cast<uint8_t>(rx_buf_len_),
                         &cmd, payload, &pay_len, &seq);
  if (ret < 0) {
    /* No valid frame at head — scan for sync */
    for (size_t i = 1; i < rx_buf_len_; i++) {
      if (rx_buf_[i] == PROTO_SYNC0) {
        std::memmove(rx_buf_.data(), rx_buf_.data() + i, rx_buf_len_ - i);
        rx_buf_len_ -= i;
        return false;
      }
    }
    rx_buf_len_ = 0; /* No sync found, discard all */
    return false;
  }

  /* Remove parsed frame from buffer */
  uint8_t frame_total = PROTO_FRAME_OVERHEAD + pay_len;
  std::memmove(rx_buf_.data(), rx_buf_.data() + frame_total, rx_buf_len_ - frame_total);
  rx_buf_len_ -= frame_total;

  seq_rx_ = seq;

  if (cmd == CMD_ODOM_FEEDBACK && pay_len == sizeof(odom_feedback_t)) {
    std::memcpy(&odom, payload, sizeof(odom));
    return true;
  }

  return false;
}

}  // namespace mcr_bringup
