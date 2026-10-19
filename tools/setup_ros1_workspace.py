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
    # Migrate only source links created by the preceding layout. User checkouts
    # and links to any other location are never replaced.
    retired = {name: root / "rlsim/ros1" / name for name in ("robot_msgs", "robot_joint_controller", "rl_sar")}
    owned = []
    for name, target in retired.items():
        path = sources / name
        if path.is_symlink() and path.resolve() == target:
            owned.append(path)
        elif (path.exists() or path.is_symlink()) and name not in targets:
            raise ValueError("Workspace entry already points elsewhere: " + str(path))
    # Preflight everything before writing. Never replace someone else's checkout.
    for name, target in targets.items():
        path = sources / name
        if not (target / "package.xml").is_file():
            raise ValueError("Missing ROS1 package: " + str(target))
        if (path.exists() or path.is_symlink()) and path.resolve() != target and path not in owned:
            raise ValueError("Workspace entry already points elsewhere: " + str(path))
    for path in owned:
        path.unlink()
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
