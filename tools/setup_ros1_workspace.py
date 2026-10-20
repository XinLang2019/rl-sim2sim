#!/usr/bin/env python3
"""Create an isolated catkin source view of adapters; resources stay in-place."""
import argparse
from pathlib import Path


def setup(root, workspace):
    root, workspace = Path(root).resolve(), Path(workspace).resolve()
    sources = workspace / "src"
    sources.mkdir(parents=True, exist_ok=True)
    targets = {name: root / "rlsim/interfaces/gazebo" / name for name in ("robot_msgs", "robot_controller", "rlsim_gazebo")}
    targets["realsense_ros_gazebo"] = root / "third/realsense_ros_gazebo"
    # Preflight everything before writing. Never replace someone else's checkout.
    for name, target in targets.items():
        path = sources / name
        if not (target / "package.xml").is_file():
            raise ValueError("Missing ROS1 package: " + str(target))
        if (path.exists() or path.is_symlink()) and path.resolve() != target:
            raise ValueError("Workspace entry already points elsewhere: " + str(path))
    for name, target in targets.items():
        path = sources / name
        if not path.is_symlink():
            path.symlink_to(target, target_is_directory=True)
    return workspace


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    args = parser.parse_args()
    print(setup(Path(__file__).resolve().parents[1], args.workspace))
