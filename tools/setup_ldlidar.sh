#!/bin/bash
# 初始化 ldlidar submodule 并应用本地必需的编译修复。
#
# 为什么需要这个脚本：
#   1. ldlidar_stl_ros2 是 git submodule，fresh clone 后是空目录，
#      不初始化会导致 `ros2 launch mcr_bringup robot.launch.py` 报
#      "package 'ldlidar_stl_ros2' not found"。
#   2. 上游驱动 (v3.0.3) 的 log_module.h 里 `#include <pthread.h>` 被注释掉，
#      老编译器靠间接包含能过，但 ROS2 Jazzy / gcc 13+ 下报
#      "'pthread_mutex_unlock' was not declared in this scope"，整个包编不过。
#      修复见 patches/ldlidar-pthread-include.patch（已提 issue 给上游前先本地打）。
#
# 用法（在仓库根目录）：
#   ./tools/setup_ldlidar.sh
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

SUB=ros2_ws/src/ldlidar_stl_ros2
PATCH=patches/ldlidar-pthread-include.patch

echo "[1/3] 初始化 submodule..."
if [ -z "$(ls -A $SUB 2>/dev/null)" ]; then
  git submodule update --init --recursive "$SUB"
else
  echo "      已存在，跳过 clone"
fi

echo "[2/3] 检查 pthread 修复..."
HDR="$SUB/ldlidar_driver/include/logger/log_module.h"
if grep -q '^//#include <pthread.h>' "$HDR"; then
  echo "      应用 $PATCH"
  git -C "$SUB" apply "$REPO_ROOT/$PATCH"
  echo "      ✓ 已打补丁"
elif grep -q '^#include <pthread.h>' "$HDR"; then
  echo "      ✓ 已是修复状态，跳过"
else
  echo "      ⚠ log_module.h 结构与预期不符，请手动检查 pthread.h 包含"
  exit 1
fi

echo "[3/3] 验证..."
grep -n 'pthread.h' "$HDR"
echo ""
echo "完成。现在可以 colcon build 了："
echo "  cd ros2_ws && colcon build --symlink-install"
