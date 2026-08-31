#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "===== 建图前 /map 状态 ====="
timeout 8 ros2 topic echo /map --once --field info 2>/dev/null | grep -E "width|height|resolution"

echo ""
echo "===== 驱动小车走矩形轨迹建图 ====="
# 前进 1.5m -> 左转90 -> 前进 1m -> 左转90 -> 前进1.5m -> 左转90 -> 前进1m
drive() {
  local lx=$1 az=$2 dur=$3 desc=$4
  echo "  $desc (${dur}s)"
  timeout "$dur" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $lx, y: 0.0, z: 0.0}, angular: {z: $az}}" >/dev/null 2>&1
}

drive 0.25 0.0  6  "前进"
drive 0.0  0.5  3  "左转"
drive 0.25 0.0  4  "前进"
drive 0.0  0.5  3  "左转"
drive 0.25 0.0  6  "前进"
drive 0.0  0.5  3  "左转"
drive 0.25 0.0  4  "前进"
drive 0.0  0.5  3  "左转(回到起点朝向)"

# 停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
echo "  停车"
sleep 5

echo ""
echo "===== 建图后 EKF 位姿(应回到起点附近) ====="
timeout 6 ros2 topic echo /odometry/filtered --once --field pose.pose.position 2>/dev/null | head -4

echo ""
echo "===== map->odom 修正量(SLAM 认为里程计漂了多少) ====="
timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 -A6 Translation

echo ""
echo "===== 最终地图 ====="
timeout 8 ros2 topic echo /map --once --field info 2>/dev/null | grep -E "width|height|resolution"

echo ""
echo "===== 保存地图 ====="
mkdir -p /ros2_ws/maps
cd /ros2_ws/maps
timeout 30 ros2 run nav2_map_server map_saver_cli -f sil_room --ros-args -p save_map_timeout:=20.0 2>&1 | tail -5
ls -la /ros2_ws/maps/
