cmake_minimum_required(VERSION 3.8)
project(mcr_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(udp_detection_bridge src/udp_detection_bridge.cpp)
ament_target_dependencies(udp_detection_bridge rclcpp std_msgs)
target_compile_features(udp_detection_bridge PRIVATE cxx_std_17)
target_compile_options(udp_detection_bridge PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS udp_detection_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
