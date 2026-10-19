#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f /opt/ros/noetic/setup.bash ] || [ ! -f "$project_root/tools/build/ros1/devel/setup.bash" ] || [ ! -x "$project_root/tools/build/core/rlsim_policy_run" ]; then
  echo 'Build first: ./build.sh core && source /opt/ros/noetic/setup.bash && ./build.sh ros1 -DTorch_DIR=/path/to/libtorch/share/cmake/Torch' >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$project_root/tools/build/ros1/devel/setup.bash"
cd "$project_root"
exec "${RLSIM_WEB_PYTHON:-/usr/bin/python3}" tools/rlsim.py serve \
  --policy-run "$project_root/tools/build/core/rlsim_policy_run" \
  --ros-python /usr/bin/python3 --xacro /opt/ros/noetic/bin/xacro "$@"
