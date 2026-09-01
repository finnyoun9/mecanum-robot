#!/bin/bash
# 在容器内把 RViz2 弹到 WSL 的 xrdp 远程桌面（:10），实时看 SLAM 建图。
#
# 前提：
#   1. xrdp 桌面已登录（Windows 上 mstsc 连 localhost:3390，finn/1233）
#   2. SLAM 栈在跑（tools/run_slam_sil.sh）
#   3. 容器用 --network host 启动（xrdp 的 Xorg :10 是 abstract socket，
#      只有共享网络命名空间才连得到；/tmp/.X11-unix 里看不到 X10 是正常的）
#
# 用法（宿主机 WSL 里）：
#   bash tools/rviz_to_xrdp.sh                 # 用 live_view.rviz
#   bash tools/rviz_to_xrdp.sh <容器名>
#
# 原理：xrdp 会话是 Xorg :10，用 ~/.Xauthority 里的 cookie 授权；
# 把 cookie 挂进容器，设 DISPLAY=:10 启动 rviz2。
set -e

CONTAINER="${1:-mcr_slam}"
DISP=":10"
COOKIE_TOOLS="/tools/.Xauth_xrdp"

# 1. 确认 xrdp 会话存在
if ! pgrep -f "Xorg :10" >/dev/null 2>&1; then
  echo "✗ 没找到 xrdp Xorg :10 会话。请先用 mstsc 登录 localhost:3390 再跑本脚本。"
  exit 1
fi
echo "[i] 发现 xrdp Xorg :10 会话"

# 2. 拷 cookie 到挂载目录（容器能读到）
cp "$HOME/.Xauthority" "$HOME/mecanum-robot/tools/.Xauth_xrdp"
chmod 644 "$HOME/mecanum-robot/tools/.Xauth_xrdp"

# 3. 容器内启动 rviz2
docker exec -u ubuntu "$CONTAINER" bash -c '
  source /opt/ros/jazzy/setup.bash
  source /ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export DISPLAY="'"$DISP"'"
  export XAUTHORITY="'"$COOKIE_TOOLS"'"
  # ogre vendor 库（镜像里 rviz2 正常 source 后一般已含，这行兜底）
  export LD_LIBRARY_PATH=/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:$LD_LIBRARY_PATH
  export XDG_RUNTIME_DIR=/tmp/xdg-rviz
  mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

  CFG=/ros2_ws/install/mcr_navigation/share/mcr_navigation/config/live_view.rviz
  echo "[i] 启动 rviz2 -> DISPLAY=$DISPLAY"
  if [ -f "$CFG" ]; then
    exec rviz2 -d "$CFG"
  else
    echo "[!] live_view.rviz 不存在，启动空白 rviz2"
    exec rviz2
  fi
'
