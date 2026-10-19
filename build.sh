#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-core}"
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
  core)
    cmake -S "$project_root/rlsim/inference" -B "$project_root/tools/build/core" -DRLSIM_BUILD_TESTS=ON "$@"
    cmake --build "$project_root/tools/build/core" -j2
    ;;
  ros1)
    if [ "${ROS_DISTRO:-}" != noetic ]; then
      echo 'Source /opt/ros/noetic/setup.bash before building the ROS1 adapter.' >&2
      exit 2
    fi
    ros_workspace="${RLSIM_ROS_WORKSPACE:-$project_root/tools/build/ros1}"
    python3 "$project_root/tools/setup_ros1_workspace.py" "$ros_workspace"
    torch_args=()
    if [ -n "${Torch_DIR:-}" ]; then torch_args+=("-DTorch_DIR=$Torch_DIR"); fi
    catkin config --workspace "$ros_workspace" --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 "${torch_args[@]}" "$@"
    catkin build --workspace "$ros_workspace" --no-status -j1
    ;;
  *) echo 'Usage: ./build.sh core|ros1 [CMake options...]' >&2; exit 2 ;;
esac
