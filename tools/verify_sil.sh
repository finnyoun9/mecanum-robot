#!/bin/bash
# SIL verification — run inside the mcr-ros2 container with --network host.
# Uses the same unicast-discovery config as run_sil.sh so it can see the
# bringup nodes under WSL mirrored networking.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ParticipantIndex=100: under WSL, all participants otherwise share the same
# SPDP port via SO_REUSEPORT, and the kernel only delivers unicast discovery
# packets to ONE of the sharing sockets — so a fresh CLI never sees the
# bringup nodes. A distinct index gives this process its own discovery port.
export CYCLONEDDS_URI='<CycloneDDS xmlns="https://cyclonedds.io/schemas/cyclonedds.xsd"><Domain><Discovery><Peers><Peer address="127.0.0.1"/></Peers><ParticipantIndex>100</ParticipantIndex></Discovery><Internal><Verbosity><Category>TRACE</Category></Verbosity></Internal></Domain></CycloneDDS>'
source /opt/ros/jazzy/setup.bash

echo "=== 节点 ==="
timeout 15 ros2 node list 2>&1
echo "=== 话题 ==="
timeout 15 ros2 topic list 2>&1 | head -25
echo "=== /odom 采样 ==="
timeout 12 ros2 topic echo /odom --once 2>&1 | head -22
