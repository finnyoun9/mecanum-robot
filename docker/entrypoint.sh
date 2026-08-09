#!/bin/bash
# Sources the ROS2 environment (and the workspace overlay, if it's been
# built yet) before handing off to whatever command was requested.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
