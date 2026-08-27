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

  /* termios speed fields MUST be set with B<baud> constants. Passing the
   * raw baud integer (as this code used to do) masked against CBAUD gives
   * B0 — the kernel treats B0 as a modem hangup, which leaves the port in
   * a hung state; writes then fail after the output buffer fills
   * (~seconds at 100 Hz), observed as EAGAIN/ENOTTY and a deactivated
   * hardware component. */
  speed_t speed = B0;
  switch (baudrate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:
      close();
      errno = EINVAL;
      return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  /* 8N1, raw mode */
  tty.c_cflag = CS8 | CREAD | CLOCAL;
  tty.c_iflag = IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  /* Non-blocking read: VMIN=0, VTIME=0 */
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  /* Baud LAST: on Linux the speed lives in the CBAUD bits of c_cflag, so
   * the plain `c_cflag = ...` assignment above wipes whatever cfsetospeed
   * stored. Setting the flags first and the speed after is what keeps the
   * speed — doing it the other way round silently left CBAUD at 0 (= B0,
   * a modem hangup), which is exactly the failure the comment above warns
   * about: the port hangs, the 16 KB kernel TX queue fills, and every
   * write then returns EAGAIN. */
  if (cfsetospeed(&tty, speed) < 0 || cfsetispeed(&tty, speed) < 0) {
    close();
    return false;
  }

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  /* Confirm the speed actually took: tcsetattr can succeed while silently
   * ignoring an unsupported speed, and a wrong baud here is invisible
   * until the link mysteriously produces framing errors. */
  struct termios verify;
  if (tcgetattr(fd_, &verify) != 0 ||
      cfgetospeed(&verify) != speed || cfgetispeed(&verify) != speed) {
    close();
    return false;
  }

  tcflush(fd_, TCIFLUSH);

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

  /* The port is O_NONBLOCK, so ::write() may consume only part of the frame
   * (or return EAGAIN outright) whenever the kernel TX buffer is full —
   * which happens as soon as the STM32 stops draining, e.g. while it sits
   * in its power-on safety latch. Treating that as a fatal error is what
   * made controller_manager deactivate MCRSystem a few seconds into every
   * run: a partial write is not a broken link, it is backpressure.
   *
   * Loop over the remainder with a short poll so a frame is written whole
   * or not at all. The bound keeps a genuinely wedged port from blocking
   * the 100 Hz control cycle; a frame that still cannot be flushed is
   * dropped (the STM32's own comms watchdog covers a lost command). */
  const uint8_t *p = data;
  size_t remaining = len;
  constexpr int MAX_ATTEMPTS = 8;   /* ~2 ms worst case at 250 us/poll */
  constexpr int POLL_US = 250;

  for (int attempt = 0; attempt < MAX_ATTEMPTS && remaining > 0; ++attempt) {
    ssize_t written = ::write(fd_, p, remaining);
    if (written > 0) {
      p += written;
      remaining -= static_cast<size_t>(written);
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return false;   /* a real error: ENXIO, EIO, EBADF, ... */
    }
    usleep(POLL_US);
  }

  return remaining == 0;
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
