#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace {
constexpr std::size_t kMaxPacketBytes = 65507;

class UdpDetectionBridge final : public rclcpp::Node {
 public:
  UdpDetectionBridge() : Node("udp_detection_bridge") {
    const auto bind_address = declare_parameter<std::string>("bind_address", "127.0.0.1");
    const auto port = declare_parameter<int>("port", 12000);
    const auto topic = declare_parameter<std::string>("topic", "/perception/detections");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/perception/status");
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 1500);
    if (port < 1 || port > 65535 || stale_timeout_ms_ < 1) {
      throw std::invalid_argument("port must be 1..65535 and stale_timeout_ms must be positive");
    }
    detections_publisher_ = create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(1).transient_local());
    open_socket(bind_address, static_cast<uint16_t>(port));
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { poll_socket(); });
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_status(); });
    RCLCPP_INFO(get_logger(), "bridging UDP %s:%d to %s", bind_address.c_str(), port, topic.c_str());
  }

  ~UdpDetectionBridge() override {
    if (socket_fd_ >= 0) close(socket_fd_);
  }

 private:
  void open_socket(const std::string &bind_address, uint16_t port) {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) throw std::runtime_error("cannot create UDP socket: " + std::string(std::strerror(errno)));
    const int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error("cannot make UDP socket non-blocking");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
      throw std::invalid_argument("bind_address must be an IPv4 address");
    }
    if (bind(socket_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      throw std::runtime_error("cannot bind UDP socket: " + std::string(std::strerror(errno)));
    }
  }

  static bool is_detection_packet(const std::string &packet) {
    return packet.size() >= 40 && packet.front() == '{' && packet.back() == '}' &&
           packet.find("\"schema\":\"mcr.perception.detections.v1\"") != std::string::npos;
  }

  void poll_socket() {
    char buffer[kMaxPacketBytes];
    for (;;) {
      const auto length = recv(socket_fd_, buffer, sizeof(buffer), 0);
      if (length < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) RCLCPP_WARN(get_logger(), "UDP receive error: %s", std::strerror(errno));
        return;
      }
      const std::string packet(buffer, static_cast<std::size_t>(length));
      if (!is_detection_packet(packet)) {
        ++invalid_packets_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ignored invalid perception UDP packet");
        continue;
      }
      std_msgs::msg::String message;
      message.data = packet;
      detections_publisher_->publish(message);
      ++received_packets_;
      last_packet_ = now();
    }
  }

  void publish_status() {
    const auto age_ms = last_packet_.nanoseconds() == 0 ? -1 : (now() - last_packet_).nanoseconds() / 1000000;
    const bool streaming = age_ms >= 0 && age_ms <= stale_timeout_ms_;
    std_msgs::msg::String status;
    status.data = "{\"schema\":\"mcr.perception.status.v1\",\"state\":\"" +
                  std::string(streaming ? "streaming" : "waiting") + "\",\"age_ms\":" +
                  std::to_string(age_ms) + ",\"received_packets\":" + std::to_string(received_packets_) +
                  ",\"invalid_packets\":" + std::to_string(invalid_packets_) + "}";
    status_publisher_->publish(status);
  }

  int socket_fd_{-1};
  int stale_timeout_ms_{1500};
  uint64_t received_packets_{0};
  uint64_t invalid_packets_{0};
  rclcpp::Time last_packet_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpDetectionBridge>());
  rclcpp::shutdown();
  return 0;
}
