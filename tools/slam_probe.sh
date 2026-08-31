#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "===== 节点列表 ====="
ros2 node list 2>/dev/null

echo ""
echo "===== 关键话题频率 ====="
for t in /scan /odom /odometry/filtered /map; do
  printf "%-22s " "$t"
  timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "(无数据)"
done

echo ""
echo "===== TF 树关键链 ====="
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -m1 -A3 Translation || echo "map->odom 无"
echo "--- odom->base_link ---"
timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -m1 -A3 Translation || echo "odom->base_link 无"

echo ""
echo "===== slam_toolbox 生命周期 ====="
ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "(非lifecycle或未起)"

echo ""
echo "===== /map 元信息 ====="
timeout 8 ros2 topic echo /map --once --field info 2>/dev/null | head -12 || echo "(尚无地图)"
