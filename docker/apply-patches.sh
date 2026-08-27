#!/bin/bash
# Apply local patches to git submodules before colcon build.
#
# The ldlidar_stl_ros2 submodule is pinned to an upstream commit (bf668a8)
# with a real GCC-13 build bug: log_module.cpp calls pthread_mutex_*
# without #include <pthread.h>, and GCC 13 (the ros:jazzy-ros-base
# compiler) no longer pulls it in transitively. We patch it locally
# rather than forking — the patch is small and the pinned commit is fixed.
#
# Prefers `git apply` (needs git, which the dev host has); falls back to
# `patch` so it also works in the CI container (ros:jazzy-ros-base has
# no git by default — the CI workflow installs `patch` instead).
#
# Idempotent: a patch already applied is skipped, so re-running colcon
# build won't error. Run from the repo root, or set MCR_REPO_ROOT.

set -euo pipefail

REPO_ROOT="${MCR_REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PATCH_DIR="$REPO_ROOT/docker/patches"
SUBMODULE="$REPO_ROOT/ros2_ws/src/ldlidar_stl_ros2"

apply_one() {
    local patch_file="$1"
    local target_dir="$2"
    local name
    name="$(basename "$patch_file")"

    # --forward --batch: don't ask, don't reverse an already-applied hunk.
    if git -C "$target_dir" apply --check -p1 "$patch_file" 2>/dev/null; then
        git -C "$target_dir" apply -p1 "$patch_file"
        echo "  applied (git): $name"
    elif (cd "$target_dir" && patch --forward --batch -p1 --dry-run --silent < "$patch_file" >/dev/null 2>&1); then
        (cd "$target_dir" && patch --forward --batch -p1 --silent < "$patch_file" >/dev/null 2>&1)
        echo "  applied (patch): $name"
    else
        echo "  skip (already applied or no-match): $name"
    fi
}

echo "Applying local patches to submodules..."

apply_one "$PATCH_DIR/ldlidar-include-pthread.patch" "$SUBMODULE"

echo "done."
