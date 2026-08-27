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
#
# The Pi's mihomo proxy (127.0.0.1:7890) makes those apt downloads
# dramatically faster, so pass it through as build args when it's actually
# listening — otherwise the build falls back to the direct connection.
BUILD_ARGS=()
if timeout 2 bash -c 'exec 3<>/dev/tcp/127.0.0.1/7890' 2>/dev/null; then
    BUILD_ARGS+=(--build-arg http_proxy=http://127.0.0.1:7890)
    BUILD_ARGS+=(--build-arg https_proxy=http://127.0.0.1:7890)
fi

build() {
    docker build --network host "${BUILD_ARGS[@]}" -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
}

if [[ "$1" == "--build" ]]; then
    build
    shift
elif ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Image '$IMAGE_NAME' not found, building it now..."
    build
fi

# Candidate serial devices — STM32 link and LD06 LiDAR. On this Pi 5,
# GPIO14/15 is /dev/ttyAMA0 (/dev/serial0); /dev/ttyAMA10 is RP1-internal
# and does not reach the header. ttyACM0 covers boards that expose USB-CDC.
DEVICE_ARGS=()
for dev in /dev/ttyAMA0 /dev/ttyUSB0 /dev/ttyACM0; do
    if [ -e "$dev" ]; then
        DEVICE_ARGS+=(--device "$dev:$dev")
    fi
done

# mcr_bringup's CMake references the shared protocol sources via a path that
# resolves to /shared inside the container, so that directory must be mounted
# for the workspace to build.
docker run -it --rm \
    --name mcr_ros2 \
    --network host \
    "${DEVICE_ARGS[@]}" \
    -v "$REPO_ROOT/ros2_ws:/ros2_ws" \
    -v "$REPO_ROOT/shared:/shared" \
    -v "$REPO_ROOT/tools:/tools" \
    "$IMAGE_NAME" \
    "${@:-bash}"
