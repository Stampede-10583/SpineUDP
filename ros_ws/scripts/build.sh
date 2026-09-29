#!/usr/bin/env bash
# Build the workspace: install missing package deps with rosdep, then colcon build.
#   scripts/build.sh [extra colcon build args...]
# Set SPINE_SKIP_ROSDEP=1 to skip the rosdep step (e.g. offline).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "${WS}"

if [ "${SPINE_SKIP_ROSDEP:-0}" != "1" ]; then
  # Only calls apt (with sudo outside a container) when something is actually missing.
  rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}"
fi

# compile_commands.json is used by scripts/check_gcc13.sh and editors. Passing your own
# --cmake-args replaces these (colcon keeps the last --cmake-args).
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "$@"
