# ROS2 Jazzy environment for the mecanum robot, running as an Ubuntu 24.04
# container on top of the Pi's native Raspberry Pi OS (Debian 12 Bookworm).
#
# Why: ROS2's official apt packages only target Ubuntu, not Debian. Rather
# than reflashing the Pi or building ROS2 from source on Debian (slow,
# rosdep's OS database doesn't cover Debian well), we run the official
# arm64 ROS2 Jazzy image as a container — no emulation needed since the
# host is already arm64, so this is effectively native-speed.
#
# The camera (CSI ribbon, libcamera/rp1-cfe) is intentionally NOT wired
# into this container — libcamera's userspace stack is tightly coupled to
# the host kernel driver ABI, which makes containerizing it a real can of
# worms. Camera-based perception (perception/detection, laser_triangulation)
# runs natively on the host instead; see docker/README.md.

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
