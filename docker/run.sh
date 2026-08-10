#!/bin/bash
# Build (if needed) and start the mecanum-robot ROS2 Jazzy container.
#
# Usage:
#   ./docker/run.sh                 -> interactive bash shell in the container
#   ./docker/run.sh --build         -> force a rebuild first
#   ./docker/run.sh colcon build    -> run a one-off command instead of a shell
#
# The container uses --network host so ROS2's DDS discovery works exactly
# like a native install (no bridging needed between container and host
# network namespaces — DDS relies on UDP multicast, which host networking
# passes through untouched).
#
# Hardware devices (STM32 UART, LD06 LiDAR USB-serial) are probed at
# startup and only passed through if actually present, so this script
# doesn't fail before you've wired everything up.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="mcr-ros2:jazzy"

# --network host during the BUILD (not just at runtime) matters here too:
# navigation2's dependency tree is huge (500+ packages), and under that much
# apt traffic Docker's default bridge-network embedded DNS (127.0.0.11)
# started failing resolution mid-build in testing, even though it resolved
# fine at first. --network host skips Docker's DNS layer entirely and uses
# the host's own resolver for the RUN steps, which doesn't have that problem.
if [[ "$1" == "--build" ]]; then
    docker build --network host -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
    shift
elif ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Image '$IMAGE_NAME' not found, building it now..."
    docker build --network host -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
fi

# Candidate serial devices — STM32 link and LD06 LiDAR. Pi 5's UART
# enumerates as ttyAMA10 (not ttyAMA0, unlike earlier Pi models, because
# of the RP1 I/O chip); ttyACM0 covers boards that expose USB-CDC serial
# directly instead of going through a USB-to-serial adapter.
DEVICE_ARGS=()
for dev in /dev/ttyAMA10 /dev/ttyAMA0 /dev/ttyUSB0 /dev/ttyACM0; do
    if [ -e "$dev" ]; then
        DEVICE_ARGS+=(--device "$dev:$dev")
    fi
done

docker run -it --rm \
    --name mcr_ros2 \
    --network host \
    "${DEVICE_ARGS[@]}" \
    -v "$REPO_ROOT/ros2_ws:/ros2_ws" \
    "$IMAGE_NAME" \
    "${@:-bash}"
