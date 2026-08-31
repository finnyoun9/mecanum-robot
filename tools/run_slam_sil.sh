#!/bin/bash
# SLAM SIL 全栈：STM32模拟器 + 底盘 + EKF + 假雷达 + slam_toolbox
# 用于在无硬件情况下验证 SLAM 链路与参数（M5 地图质量调试）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /ros2_ws/install/setup.bash

# 1. STM32 模拟器
python3 /tools/stm32_uart_sim.py > /tmp/sim.log 2>&1 &
sleep 3
PTY=$(grep -oP '/dev/pts/\d+' /tmp/sim.log | head -1)
echo "STM32 sim pty: $PTY"

# 2. 底盘栈（含 EKF / 控制器 / robot_state_publisher）
#    lidar 节点也会起但连不上 /dev/ttyUSB0，无害（会一直重试）
ros2 launch mcr_bringup robot.launch.py "serial_device:=$PTY" > /tmp/robot.log 2>&1 &
echo "robot bringup starting..."
sleep 15

# 3. 假雷达（提供 /scan）
python3 /tools/fake_lidar_sil.py > /tmp/fake_lidar.log 2>&1 &
echo "fake lidar starting..."
sleep 3

# 4. slam_toolbox 建图
ros2 launch mcr_navigation navigation.launch.py nav2:=false rviz:=false > /tmp/slam.log 2>&1 &
echo "slam_toolbox starting..."
sleep 8

echo "=== SLAM SIL 全栈就绪 ==="
wait
