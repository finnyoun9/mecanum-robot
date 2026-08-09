cmake_minimum_required(VERSION 3.8)
project(mcr_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)

# --- Shared protocol library (C) ---
add_library(protocol STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/protocol.c
)
target_include_directories(protocol PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared
)
# Prevent C++ name mangling for the C functions
set_target_properties(protocol PROPERTIES
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
)

# --- Mecanum kinematics library ---
add_library(mecanum_kinematics
  src/mecanum_kinematics.cpp
)
target_include_directories(mecanum_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_kinematics rclcpp)

# --- Serial protocol library ---
add_library(serial_protocol
  src/serial_protocol.cpp
)
target_include_directories(serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial_protocol protocol)
ament_target_dependencies(serial_protocol rclcpp)

# --- Hardware interface plugin ---
add_library(mcr_hardware_interface
  src/mcr_hardware_interface.cpp
)
target_include_directories(mcr_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mcr_hardware_interface
  mecanum_kinematics
  serial_protocol
  protocol
)
ament_target_dependencies(mcr_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  angles
)

# Mark as a plugin library
pluginlib_export_plugin_description_file(
  hardware_interface mcr_hardware_plugin.xml
)

# --- Install ---
install(TARGETS protocol mecanum_kinematics serial_protocol mcr_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mecanum_kinematics test/test_mecanum_kinematics.cpp)
  target_link_libraries(test_mecanum_kinematics mecanum_kinematics)
endif()

ament_package()
