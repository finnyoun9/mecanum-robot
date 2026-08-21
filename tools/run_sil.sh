#!/bin/bash
# SIL (software-in-the-loop) launcher — runs inside the mcr-ros2 container.
# Starts the STM32 UART simulator on a pty, then brings up the full
# mcr_bringup stack pointed at that pty. No real hardware needed.
set -e

# WSL: use CycloneDDS (more robust than FastDDS under WSL mirrored
# networking) with DEFAULT multicast discovery — loopback multicast works,
# while the unicast-peer config hangs discovery for fresh processes.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 /tools/stm32_uart_sim.py > /tmp/sim.log 2>&1 &
sleep 2

PTY=$(grep -oP '/dev/pts/\d+' /tmp/sim.log | head -1)
echo "=== STM32 sim pty: $PTY ==="

source /ros2_ws/install/setup.bash
exec ros2 launch mcr_bringup robot.launch.py "serial_device:=$PTY"
