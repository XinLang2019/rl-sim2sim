"""Validate self-contained robot geometry and its Gazebo Classic integration."""
from pathlib import Path
from urllib.parse import unquote
import xml.etree.ElementTree as ET

from .catalog import ConfigError, identifier, load_yaml, path_in


def mesh_dependencies(package, model):
    """Follow URDF mesh references and Collada image files, including textures."""
    package = Path(package).resolve()
    result = set()

    def local_file(value, parent):
        path = (parent / unquote(value)).resolve()
        try:
            path.relative_to(package)
        except ValueError as error:
            raise ConfigError("Mesh or texture escapes robot package: " + value) from error
        if not path.is_file():
            raise ConfigError("Missing robot mesh or texture: " + str(path))
        return path

    for mesh in model.findall(".//mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://" + package.name + "/"
        if not uri.startswith(prefix):
            raise ConfigError("Mesh must use its robot package URI: " + uri)
        result.add(local_file(uri[len(prefix):], package))
    for mesh in list(result):
        if mesh.suffix.lower() != ".dae":
            continue
        stack = []
        for event, node in ET.iterparse(str(mesh), events=("start", "end")):
            tag = node.tag.rsplit("}", 1)[-1]
            if event == "start":
                stack.append(tag)
                continue
            if tag == "init_from" and "image" in stack and node.text:
                result.add(local_file(node.text.strip(), mesh.parent))
            stack.pop()
            node.clear()
    return result


def inspect_package(root, directory, render=True, xacro=None):
    root = Path(root).resolve()
    package = path_in(root, directory)
    required = ("package.xml", "CMakeLists.txt", "urdf/robot.urdf", "urdf/gazebo.xacro", "config/controllers.yaml")
    for name in required:
        if not (package / name).is_file() or (package / name).is_symlink():
            raise ConfigError("Missing canonical asset file: " + str(package / name))
    if not (package / "meshes").is_dir():
        raise ConfigError("Missing meshes directory: " + str(package))
    manifest = ET.parse(package / "package.xml").getroot()
    if manifest.findtext("name") != package.name:
        raise ConfigError("ROS package name does not match its asset directory")
    model = ET.parse(package / "urdf/robot.urdf").getroot()
    if model.tag != "robot" or model.findall("gazebo"):
        raise ConfigError("robot.urdf must contain geometry; put Gazebo settings in gazebo.xacro")
    links = [node.get("name") for node in model.findall("link")]
    if not links or len(set(links)) != len(links) or not all(links):
        raise ConfigError("Robot links must have unique nonempty names")
    joints = model.findall("joint")
    joint_names = [node.get("name") for node in joints]
    if len(set(joint_names)) != len(joint_names) or not all(joint_names):
        raise ConfigError("Robot joints must have unique nonempty names")
    parents = {}
    for joint in joints:
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None or parent.get("link") not in links or child.get("link") not in links:
            raise ConfigError("Joint references an unknown link: " + str(joint.get("name")))
        if child.get("link") in parents:
            raise ConfigError("Link has more than one parent: " + child.get("link"))
        parents[child.get("link")] = parent.get("link")
    if len(set(links) - set(parents)) != 1:
        raise ConfigError("Robot must have exactly one root link")
    for link in links:
        visited = set()
        while link in parents:
            if link in visited:
                raise ConfigError("Robot joint graph contains a cycle")
            visited.add(link)
            link = parents[link]
    controlled = {node.get("name") for node in joints if node.get("type") != "fixed"}
    transmitted = []
    for transmission in model.findall("transmission"):
        joint = transmission.find("joint")
        if joint is None or joint.findtext("hardwareInterface") != "hardware_interface/EffortJointInterface":
            raise ConfigError("Controlled joints require effort transmissions")
        transmitted.append(joint.get("name"))
    if set(transmitted) != controlled or len(transmitted) != len(controlled):
        raise ConfigError("Effort transmissions must cover controlled joints exactly once")
    controllers = load_yaml(package / "config/controllers.yaml")
    if not isinstance(controllers, dict) or len(controllers) != 1:
        raise ConfigError("controllers.yaml requires exactly one robot namespace")
    namespace, entries = next(iter(controllers.items()))
    identifier(namespace)
    if not isinstance(entries, dict) or entries.get("joint_state_controller", {}).get("type") != "joint_state_controller/JointStateController":
        raise ConfigError("Missing joint-state controller")
    bound = []
    for name, entry in entries.items():
        identifier(name)
        if name == "joint_state_controller":
            continue
        if not isinstance(entry, dict) or entry.get("type") != "robot_controller/RobotJointController":
            raise ConfigError("Unsupported controller type: " + name)
        bound.append(entry.get("joint"))
    if set(bound) != controlled or len(bound) != len(controlled):
        raise ConfigError("Controller mapping must cover controlled joints exactly once")
    dependencies = mesh_dependencies(package, model)
    available = {p.resolve() for p in (package / "meshes").rglob("*") if p.is_file()}
    if dependencies != available:
        raise ConfigError("meshes/ contains files outside the geometry/texture dependency set")
    plugins = []
    if render:
        from .experiment import render_robot
        relative = package.relative_to(root).as_posix()
        robot = {"package_path": relative, "xacro": relative + "/urdf/gazebo.xacro", "namespace": "/" + namespace + "/", "excluded_plugins": []}
        rendered = ET.fromstring(render_robot(root, robot, xacro))
        if {node.get("name") for node in rendered.findall("link")} != set(links):
            raise ConfigError("Gazebo integration unexpectedly adds or removes geometry")
        for element in rendered.findall("gazebo"):
            reference = element.get("reference")
            if reference and reference not in set(links) | set(joint_names):
                raise ConfigError("Gazebo settings reference an unknown link or joint: " + reference)
        plugins = sorted(node.get("filename") for node in rendered.findall(".//plugin"))
        if plugins != ["libgazebo_ros_control.so", "libgazebo_ros_imu_sensor.so"]:
            raise ConfigError("Unexpected plugins in the canonical Gazebo integration")
    return {"package": package.relative_to(root).as_posix(), "links": len(links), "controlled_joints": len(controlled),
            "mesh_files": len(dependencies), "render_checked": render, "plugins": plugins}


def check_packages(root, render=True, xacro=None):
    root = Path(root).resolve()
    reports = []
    for package in sorted(Path(root).glob("robots/*/*_description")):
        try:
            reports.append({"ok": True, **inspect_package(root, package, render, xacro)})
        except (ConfigError, OSError, ET.ParseError) as error:
            reports.append({"ok": False, "package": package.relative_to(root).as_posix(), "error": str(error)})
    return {"ok": bool(reports) and all(item["ok"] for item in reports), "packages": reports}
