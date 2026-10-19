"""Explicit relocation of historical paths; no runtime fallback into src/."""
from pathlib import Path
from functools import lru_cache
import json


@lru_cache(maxsize=1)
def layout_files():
    path = Path(__file__).resolve().parents[2] / "tools/archive/layout-migration.json"
    if not path.is_file():
        return {}
    return {record["from"]: record["to"] for record in json.loads(path.read_text())["files"]}


@lru_cache(maxsize=1)
def gazebo_files():
    path = Path(__file__).resolve().parents[2] / "tools/archive/gazebo-layout-migration.json"
    if not path.is_file():
        return {}
    return {record["from"]: record["to"] for record in json.loads(path.read_text())["files"]}


@lru_cache(maxsize=1)
def robot_directories():
    path = Path(__file__).resolve().parents[2] / "tools/archive/robot-category-migration.json"
    if not path.is_file():
        return {}
    return {record["from"]: record["to"] for record in json.loads(path.read_text())["directories"]}


def robot_asset_path(name):
    """Relocate package directories, including files absent before the move."""
    name = str(name)
    parts = name.split("/", 2)
    prefix = "/".join(parts[:2])
    target = robot_directories().get(prefix)
    return target + ("/" + parts[2] if len(parts) == 3 else "") if target else name


@lru_cache(maxsize=1)
def policy_config_directories():
    path = Path(__file__).resolve().parents[2] / "tools/archive/policy-config-category-migration.json"
    if not path.is_file():
        return {}
    return {record["from"]: record["to"] for record in json.loads(path.read_text())["directories"]}


def policy_config_path(name):
    for before, after in policy_config_directories().items():
        if name == before or name.startswith(before + "/"):
            return after + name[len(before):]
    return name


@lru_cache(maxsize=1)
def profile_files():
    path = Path(__file__).resolve().parents[2] / "tools/archive/profile-configuration-migration.json"
    if not path.is_file():
        return {}
    return json.loads(path.read_text())["paths"]


def current_path(name):
    """Translate the preceding layout without modifying immutable history."""
    name = str(name)
    if name in layout_files():
        name = layout_files()[name]
    if name in profile_files():
        name = profile_files()[name]
    for before, after in (("assets/robots/", "robots/"), ("adapters/ros1/", "rlsim/ros1/"),
                          ("adapters/gazebo_classic_ros1/", "rlsim/interfaces/gazebo_classic_ros1/"),
                          ("core/", "rlsim/inference/"), ("third_party/", "third/"),
                          ("legacy/", "tools/archive/"), ("baselines/", "tools/baselines/"),
                          ("tests/", "tools/tests/"), ("docs/", "tools/docs/"),
                          ("services/", "tools/web/services/"), ("web/", "tools/web/frontend/"),
                          ("runs/", "tools/runs/"), ("schemas/", "configs/schemas/")):
        if name.startswith(before):
            name = after + name[len(before):]
            break
    return policy_config_path(robot_asset_path(gazebo_files().get(name, name)))


def relocated(name):
    name = str(name)
    if name == "src/CMakeLists.txt":
        return current_path("legacy/catkin-source/CMakeLists.txt")
    old = "src/rl_sar/policy/"
    if name.startswith(old):
        rest = name[len(old):]
        prefix = "legacy/policy/" if Path(rest).suffix in (".yaml", ".hpp") else "policies/"
        return current_path(prefix + rest)
    for before, after in (("src/robots/robot_msgs/", "adapters/ros1/robot_msgs/"),
                          ("src/robots/", "assets/robots/"),
                          ("src/rl_sar/library/thirdparty/", "third_party/"),
                          ("src/rl_sar/", "adapters/ros1/rl_sar/"),
                          ("src/robot_joint_controller/", "adapters/ros1/robot_joint_controller/"),
                          ("src/realsense_ros_gazebo-master/", "third_party/realsense_ros_gazebo/")):
        if name.startswith(before):
            return current_path(after + name[len(before):])
    return current_path(name)


def relocate_contract(value):
    if isinstance(value, dict):
        return {key: relocate_contract(item) for key, item in value.items()}
    if isinstance(value, list):
        return [relocate_contract(item) for item in value]
    return relocated(value) if isinstance(value, str) else value
