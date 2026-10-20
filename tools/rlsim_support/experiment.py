"""Compile resource selections into a reproducible, standalone ROS1 run."""
import copy
from contextlib import nullcontext
import warnings
import datetime
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import uuid
import xml.etree.ElementTree as ET

import yaml

from .catalog import Catalog, ConfigError, identifier, keys, load_resource, load_yaml, path_in, validate_resource, merge
from .yaml_format import dumps
from .validation import integer, number, sha256, vector
from .references import resolve_values
from .terrain import geometry, sdf_world, load_world
from .retention import output_guard, prune_task_outputs
from .robot_assets import mesh_dependencies


def ros_environment(root, robot):
    env = os.environ.copy()
    # ROS Noetic tools such as spawn_model use /usr/bin/env python3. A Conda
    # interpreter cannot import system rospy/tf/numpy extensions reliably.
    env["PATH"] = os.pathsep.join(["/opt/ros/noetic/bin", "/usr/bin", "/bin", env.get("PATH", "")])
    env.setdefault("GAZEBO_MODEL_DATABASE_URI", "")  # Generated worlds/assets are local.
    packages = str(path_in(root, robot["package_path"]).parent)
    asset_roots = [str(Path(root) / "robots" / category) for category in ("wheel", "legged")
                   if (Path(root) / "robots" / category).is_dir()]
    env["ROS_PACKAGE_PATH"] = os.pathsep.join(dict.fromkeys(filter(None, [packages, *asset_roots, str(Path(root) / "rlsim/interfaces/gazebo"),
        str(Path(root) / "third/realsense_ros_gazebo"), env.get("ROS_PACKAGE_PATH", "")])))
    env.setdefault("ROS_VERSION", "1")
    env.setdefault("ROS_DISTRO", "noetic")
    return env


def resolve(root, experiment, terrain_id=None, terrain_parameters=None, policy_run=None, control_override=None, backend_override=None, policy_pd_override=None):
    root = Path(root).resolve()
    catalog = Catalog(root)
    if isinstance(experiment, dict):
        from .task import complete_task
        source, exp = None, complete_task(root, experiment)
    elif "/" not in str(experiment) and not str(experiment).endswith(".yaml"):
        source, exp = catalog.get("experiment", str(experiment))
        exp = copy.deepcopy(exp)
    else:
        filename, separator, pointer = str(experiment).partition("#")
        source = path_in(root, filename)
        exp = copy.deepcopy(load_resource(str(source) + ("#" + pointer if separator else ""), "experiment"))
    robot_path, robot = catalog.get("robot", exp["robot"])
    policy_path, policy_asset = catalog.get("policy", exp["policy"])
    backend_path, backend = catalog.get("backend", exp["backend"])
    backend = copy.deepcopy(backend)
    # CLI terrain overrides deliberately switch a file world to a generated one.
    if terrain_id:
        exp.pop("world", None)
        exp.update(terrain=terrain_id, seed=exp.get("seed", 0), terrain_parameters=exp.get("terrain_parameters", {}))
    world_file = path_in(root, exp["world"]) if "world" in exp else None
    if world_file:
        if terrain_parameters:
            raise ConfigError("Edit terrain in the world file; --terrain-param requires --terrain")
        terrain_path = world_file
        terrain = {"id": world_file.stem, "generator": "world", "parameters": {"file": exp["world"]}}
    else:
        terrain_path, terrain = catalog.get("terrain", exp["terrain"])
    overrides = exp.get("overrides", {})
    keys(overrides, set(), {"control", "backend"}, "experiment overrides")
    if "backend" in overrides:
        keys(overrides["backend"], set(), {"physics"}, "backend overrides")
        backend = merge(backend, overrides["backend"])
    if backend_override is not None:
        override = copy.deepcopy(validate_resource(backend_override, "backend"))
        if override["id"] != backend["id"]:
            raise ConfigError("Backend override id must match selection")
        backend = override
    if world_file:
        world, world_physics = load_world(world_file)
        if (backend_override is not None or "backend" in overrides) and world_physics != backend["physics"]:
            raise ConfigError("World physics is defined in the world file; edit that file instead of backend overrides")
        backend["physics"] = world_physics
    plugins = robot["excluded_plugins"]
    if not isinstance(plugins, list) or any(not isinstance(p, str) or not p or "/" in p for p in plugins) or len(set(plugins)) != len(plugins):
        raise ConfigError("excluded_plugins must be a list of unique plugin filenames")
    if not isinstance(policy_asset["robots"], list) or not policy_asset["robots"]:
        raise ConfigError("Policy compatibility must be a nonempty list of robot ids")
    if backend["adapter"] != "gazebo_classic_ros1":
        raise ConfigError("Backend is not implemented: " + backend["adapter"])
    if robot["id"] not in policy_asset["robots"]:
        raise ConfigError("Selected policy does not declare compatibility with " + robot["id"])
    filename, separator, pointer = policy_asset["config"].partition("#")
    policy_file = path_in(root, filename)
    policy_location = str(policy_file) + ("#" + pointer if separator else "")
    policy = load_yaml(policy_location)
    policy.pop("deployment", None)
    metadata = catalog.deployments[exp["policy"]]
    if metadata and metadata["status"] != "ready":
        raise ConfigError("Policy is {}: {}".format(metadata["status"], "; ".join(metadata["diagnostics"])))
    if "model_file" in exp:
        from .policy_assets import selected_model
        selected_model(root, robot["id"], exp["policy"], exp["model_file"], catalog)
        policy["model"]["file"] = exp["model_file"]
    reference_sources = set()
    policy = resolve_values(policy, root, reference_sources)
    keys(policy, {"schema_version", "id", "model", "joints", "observations", "inputs", "states", "actions"}, context="policy")
    if type(policy["schema_version"]) is not int or policy["schema_version"] != 1:
        raise ConfigError("Unsupported policy schema")
    if not isinstance(policy["observations"], dict):
        raise ConfigError("Policy observations must be a mapping")
    for group in policy["observations"].values():
        if not isinstance(group, dict) or not isinstance(group.get("terms"), list):
            raise ConfigError("Policy observation group requires terms")
        for term in group["terms"]:
            if not isinstance(term, dict) or not isinstance(term.get("source"), str):
                raise ConfigError("Policy observation term requires a source")
            if term["source"].startswith("signal:"):
                raise ConfigError("Current Gazebo adapter does not supply external observation " + term["source"])
    controls_path = path_in(root, exp["control"])
    controls = load_resource(controls_path, "control")
    if "control" in overrides:
        keys(overrides["control"], set(), {"timing", "startup", "transition"}, "control overrides")
        controls = merge(controls, overrides["control"])
    if control_override is not None:
        override = copy.deepcopy(validate_resource(control_override, "control"))
        if override["id"] != controls["id"]:
            raise ConfigError("Control override id must match selection")
        controls = override
    names = robot["joints"]
    if not isinstance(names, list) or not 0 < len(names) <= 32:
        raise ConfigError("Current ROS adapter supports 1..32 controlled joints")
    vector(names, len(names), "robot joints", strings=True)
    if controls["joints"] != names:
        raise ConfigError("Control profile joint order must match robot canonical order")
    if not isinstance(policy["joints"], list) or set(policy["joints"]) != set(names) or len(policy["joints"]) != len(names):
        raise ConfigError("Policy joints must be a permutation of selected robot joints")
    original_reference = list(policy["actions"]["reference"])
    if policy_pd_override is not None:
        keys(policy_pd_override, {"joints", "kp", "kd"}, {"vel_scale", "reference"}, context="policy PD override")
        if policy_pd_override["joints"] != policy["joints"]:
            raise ConfigError("Policy PD joint order must match the selected policy")
        for gain in ("kp", "kd"):
            values = vector(policy_pd_override[gain], len(policy["joints"]), "policy PD " + gain)
            if any(value < 0 or value > 10000 for value in values):
                raise ConfigError("Policy PD gains must be in 0..10000")
            policy["actions"][gain] = list(values)
        if "reference" in policy_pd_override:
            policy["actions"]["reference"] = list(vector(policy_pd_override["reference"], len(policy["joints"]), "policy default joint position"))
    mapping = [names.index(name) for name in policy["joints"]]
    keys(robot["groups"], {"wheels", "legs"}, context="robot groups")
    groups = robot["groups"]
    if set(groups["wheels"]) & set(groups["legs"]) or sorted(groups["wheels"] + groups["legs"]) != sorted(names):
        raise ConfigError("Robot groups must partition controlled joints")
    if policy_pd_override is not None and "vel_scale" in policy_pd_override:
        scale = number(policy_pd_override["vel_scale"], "wheel vel_scale")
        if not 0 <= scale <= 10000:
            raise ConfigError("Wheel vel_scale must be in 0..10000")
        wheel_channels = [channel for channel in policy["actions"]["channels"]
                          if channel["mode"] == "velocity" and policy["joints"][channel["joint"]] in groups["wheels"]]
        if not wheel_channels:
            raise ConfigError("Selected policy has no wheel velocity channels for vel_scale")
        for channel in wheel_channels:
            channel["scale"] = scale
    identifier(robot["robot_name"]); identifier(robot["package"]); identifier(robot["model_name"])
    if robot["namespace"] != "/" + robot["model_name"] + "/":
        raise ConfigError("Current assets require namespace /MODEL_NAME/")
    urdf_path = path_in(root, robot["urdf"])
    urdf = ET.parse(urdf_path).getroot()
    urdf_joints = {j.get("name"): j for j in urdf.findall("joint") if j.get("type") != "fixed"}
    if set(names) != set(urdf_joints):
        raise ConfigError("Robot manifest joints do not match the URDF actuated joints")
    if policy_pd_override is not None and "reference" in policy_pd_override:
        for i, name in enumerate(policy["joints"]):
            value = policy["actions"]["reference"][i]
            if value == original_reference[i]:
                continue  # Preserve legacy presets when only gains are edited.
            joint = urdf_joints[name]
            limit = joint.find("limit")
            if joint.get("type") != "continuous" and limit is not None:
                lower, upper = limit.get("lower"), limit.get("upper")
                if ((lower is not None and value < float(lower) - 1e-7) or
                        (upper is not None and value > float(upper) + 1e-7)):
                    raise ConfigError("Default joint position outside URDF limits: " + name)
    sources = {p for p in (source, robot_path, policy_path, policy_file, controls_path, backend_path, terrain_path, urdf_path) if p is not None}
    if (root / "configs/task.yaml").is_file():
        sources.add(root / "configs/task.yaml")
    sources.update(reference_sources)
    # A code change can alter generated terrain or command semantics with identical YAML.
    sources.update(p.resolve() for p in (root / "tools/rlsim_support").glob("*.py"))
    # Include headers, build definitions, ROS messages and engine adapters.
    sources.update(p.resolve() for p in (root / "rlsim").rglob("*") if p.is_file())
    sources.update(p.resolve() for p in (root / "tools/web/services").glob("*.py"))
    package = path_in(root, robot["package_path"])
    sources.update(p.resolve() for p in package.rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    sources.update(mesh_dependencies(package, urdf))
    xacro = path_in(root, robot["xacro"])
    controller_file = path_in(root, robot["controllers"])
    controller_root = load_yaml(controller_file)
    namespace = robot["namespace"].strip("/")
    if not isinstance(controller_root, dict) or namespace not in controller_root:
        raise ConfigError("Controller YAML namespace does not match robot manifest")
    by_joint = {}
    for controller, item in controller_root[namespace].items():
        identifier(controller)
        if isinstance(item, dict) and "joint" in item:
            if item.get("type") not in ("robot_controller/RobotJointController", "robot_joint_controller/RobotJointController"):
                raise ConfigError("Unsupported joint controller type: " + controller)
            joint = item["joint"]
            if joint in by_joint:
                raise ConfigError("Duplicate controller for joint " + joint)
            by_joint[joint] = controller
    if set(by_joint) != set(names):
        raise ConfigError("Controller joints do not match robot manifest")
    if "joint_state_controller" not in controller_root[namespace]:
        raise ConfigError("Missing joint_state_controller")
    keys(controls["timing"], {"control_dt", "decimation"}, context="control timing")
    dt = number(controls["timing"]["control_dt"], "control_dt", positive=True)
    decimation = integer(controls["timing"]["decimation"], "decimation", positive=True)
    keys(backend["physics"], {"step_size", "real_time_factor", "gravity"}, context="physics")
    step = number(backend["physics"]["step_size"], "physics.step_size", positive=True)
    if number(backend["physics"]["real_time_factor"], "physics.real_time_factor", positive=True) != 1:
        raise ConfigError("Current wall-clock ROS adapter requires real_time_factor=1")
    ratio = dt / step
    if not math.isfinite(ratio) or not math.isclose(ratio, round(ratio), rel_tol=0, abs_tol=1e-8) or dt < step:
        raise ConfigError("Control period must be an integer multiple of the physics step")
    keys(controls["startup"], {"position", "kp", "kd", "torque_limits"}, context="startup")
    keys(controls["transition"], {"kp", "kd", "prone", "pre_stand_s", "stand_s", "sit_s", "passive_damping"}, context="transition")
    for key in ("position", "kp", "kd", "torque_limits"):
        vector(controls["startup"][key], len(names), "startup." + key)
    for key in ("kp", "kd", "prone"):
        vector(controls["transition"][key], len(names), "transition." + key)
    for key in ("pre_stand_s", "stand_s", "sit_s", "passive_damping"):
        number(controls["transition"][key], key, positive=True)
    for group, fields in ((controls["startup"], ("kp", "kd", "torque_limits")), (controls["transition"], ("kp", "kd"))):
        for key in fields:
            if any(v < 0 or (key == "torque_limits" and v == 0) for v in group[key]):
                raise ConfigError("Control gains/limits are invalid")
    keys(exp["spawn"], {"position", "rpy"}, context="spawn")
    vector(exp["spawn"]["position"], 3, "spawn.position"); vector(exp["spawn"]["rpy"], 3, "spawn.rpy")
    keys(exp["runtime"], {"gui", "paused", "duration_s"}, context="runtime")
    if type(exp["runtime"]["gui"]) is not bool or type(exp["runtime"]["paused"]) is not bool:
        raise ConfigError("gui and paused must be booleans")
    number(exp["runtime"]["duration_s"], "duration_s", positive=True)
    parameters = dict(terrain["parameters"])
    if world_file:
        terrain_geometry = {"schema_version": 1, "generator": "world", "file": exp["world"], "boxes": []}
    else:
        integer(exp["seed"], "seed")
        if not isinstance(exp["terrain_parameters"], dict) or not isinstance(terrain_parameters or {}, dict):
            raise ConfigError("Terrain overrides must be a mapping")
        parameters.update(exp["terrain_parameters"])
        parameters.update(terrain_parameters or {})
        terrain_geometry = geometry(terrain["generator"], parameters, exp["seed"])
        world = sdf_world(terrain_geometry, backend["physics"])
    model_file = path_in(root, policy["model"]["file"])
    sources.update((xacro, controller_file, model_file))
    diagnostics = []
    if robot["excluded_plugins"]:
        diagnostics.append({"level": "warning", "code": "ADAPTER_PLUGIN_EXCLUDED", "message": "Prepared URDF excludes explicitly configured plugins: " + ", ".join(robot["excluded_plugins"])})
    if policy_run:
        try:
            # Validate the actual task-selected model, not the original default checkpoint.
            with tempfile.TemporaryDirectory(prefix="rlsim-policy-check-") as directory:
                manifest = Path(directory) / "policy.yaml"
                manifest.write_text(dumps(policy), encoding="utf-8")
                checked = subprocess.run([str(Path(policy_run).resolve()), str(manifest), str(root), "--check"],
                                         capture_output=True, text=True, timeout=60)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise ConfigError("Core policy checker failed: " + str(error)) from error
        if checked.returncode:
            raise ConfigError("Core policy check failed: " + checked.stderr.strip())
    else:
        diagnostics.append({"level": "warning", "code": "MODEL_UNCHECKED", "message": "Supply --policy-run to validate actual model ports and policy transforms"})
    limits = vector(policy["actions"]["torque_limits"], len(names), "policy torque limits")
    for i, name in enumerate(policy["joints"]):
        limit_node = urdf_joints[name].find("limit")
        if limit_node is None or "effort" not in limit_node.attrib:
            raise ConfigError("Missing URDF effort limit: " + name)
        physical = number(float(limit_node.get("effort")), name + ".effort", positive=True)
        if limits[i] > physical:
            diagnostics.append({"level": "warning", "code": "URDF_EFFORT_CLAMP", "message": "{}: policy limit {} exceeds URDF {}; existing Gazebo controller clamps to URDF".format(name, limits[i], physical)})
    # Convert canonical robot controls into policy order exactly once.
    def ordered(values):
        return [values[i] for i in mapping]
    startup = {name: ordered(values) for name, values in controls["startup"].items()}
    transition = copy.deepcopy(controls["transition"])
    for name in ("kp", "kd", "prone"):
        transition[name] = ordered(transition[name])
    robot_runtime = {"schema_version": 1, "robot_name": robot["robot_name"], "dt": dt, "decimation": decimation,
                     "joint_names": names, "controller_names": [by_joint[name] for name in names],
                     "joint_mapping": mapping, "wheel_indices": [i for i, name in enumerate(policy["joints"]) if name in groups["wheels"]],
                     "startup": startup, "transition": transition}
    # The generic FSM can send feedforward torque; actuator modes are checked by the core.
    resolved = {"schema_version": 1, "id": exp["id"], "workspace_root": str(root),
                "selection": {"robot": robot["id"], "policy": policy_asset["id"], "backend": backend["id"], "terrain": terrain["id"]},
                "experiment": exp, "robot": robot, "policy": policy, "control": controls, "backend": backend,
                "terrain": {"id": terrain["id"], "generator": terrain["generator"], "parameters": parameters, "seed": exp.get("seed")},
                "diagnostics": diagnostics, "model_checked": bool(policy_run), "robot_runtime": robot_runtime,
                "fingerprints": {str(p.relative_to(root)): sha256(p) for p in sorted(sources)}}
    return resolved, world, terrain_geometry


def render_robot(root, robot, executable=None):
    program = executable or shutil.which("xacro")
    if not program:
        raise ConfigError("xacro is required to prepare a ROS1 run; source the ROS environment first")
    command = [str(program), str(path_in(root, robot["xacro"]))]
    try:
        result = subprocess.run(command, capture_output=True, text=True, env=ros_environment(root, robot), timeout=60)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ConfigError("xacro rendering failed: " + str(error)) from error
    if result.returncode:
        raise ConfigError("xacro rendering failed: " + result.stderr.strip())
    rendered = ET.fromstring(result.stdout)
    excluded = set(robot["excluded_plugins"])
    removed = set()
    for parent in rendered.iter():
        for child in list(parent):
            if child.tag == "plugin" and child.get("filename") in excluded:
                removed.add(child.get("filename")); parent.remove(child)
    if removed != excluded:
        raise ConfigError("Excluded plugin declaration does not match rendered robot")
    controls = [plugin for plugin in rendered.findall(".//plugin") if plugin.get("filename") == "libgazebo_ros_control.so"]
    namespaces = [plugin.findtext("robotNamespace", "") for plugin in controls]
    if len(controls) != 1 or any(value.rstrip("/") != robot["namespace"].rstrip("/") for value in namespaces):
        raise ConfigError("Rendered Gazebo plugin namespace does not match the robot manifest")
    return ET.tostring(rendered, encoding="unicode") + "\n"


def launch_xml(directory, resolved):
    directory = Path(directory).resolve()
    robot, runtime, spawn = resolved["robot"], resolved["experiment"]["runtime"], resolved["experiment"]["spawn"]
    launch = ET.Element("launch")
    for name, value in {"robot_name": robot["robot_name"], "ros_namespace": robot["namespace"],
                        "gazebo_model_name": robot["model_name"], "policy_workspace_root": resolved["workspace_root"],
                        "policy_manifest": str(directory / "policy.yaml"), "robot_runtime_manifest": str(directory / "robot_runtime.yaml")}.items():
        ET.SubElement(launch, "param", name=name, type="str", value=value)
    ET.SubElement(launch, "param", name="robot_description", textfile=str(directory / "robot.urdf"))
    ET.SubElement(launch, "param", name="simulation_running", type="bool", value=str(not runtime["paused"]).lower())
    ET.SubElement(launch, "param", name="control_topic", type="str", value=robot["namespace"] + "control")
    include = ET.SubElement(launch, "include", file="$(find gazebo_ros)/launch/empty_world.launch")
    for name, value in {"world_name": str(directory / "world.sdf"), "gui": str(runtime["gui"]).lower(),
                        "paused": "true", "use_sim_time": "true", "headless": str(not runtime["gui"]).lower()}.items():
        ET.SubElement(include, "arg", name=name, value=value)
    ET.SubElement(launch, "rosparam", file=str(directory / "controllers.yaml"), command="load")
    args = ["-urdf", "-param", "robot_description", "-model", robot["model_name"]]
    for flag, value in zip(("-x", "-y", "-z", "-R", "-P", "-Y"), spawn["position"] + spawn["rpy"]):
        args += [flag, str(value)]
    # Start in the configured posture. Motor zeros can put a heavy robot's
    # straight legs through the ground before its controllers become ready.
    robot_runtime = resolved["robot_runtime"]
    for index, position in zip(robot_runtime["joint_mapping"], robot_runtime["startup"]["position"]):
        args += ["-J", robot_runtime["joint_names"][index], str(position)]
    if not runtime["paused"]:
        # spawn_model applies -J after spawning; physics must remain paused
        # throughout that service call or the zero-pose robot can overturn.
        args += ["-unpause"]
    ET.SubElement(launch, "node", pkg="gazebo_ros", type="spawn_model", name="robot_spawner", args=" ".join(args), output="screen")
    controllers = ["joint_state_controller"] + resolved["robot_runtime"]["controller_names"]
    ET.SubElement(launch, "node", pkg="controller_manager", type="spawner", name="controller_spawner", ns=robot["namespace"], args=" ".join(controllers), output="screen")
    publisher = ET.SubElement(launch, "node", pkg="robot_state_publisher", type="robot_state_publisher", name="robot_state_publisher", output="screen")
    ET.SubElement(publisher, "remap", **{"from": "/joint_states", "to": robot["namespace"] + "joint_states"})
    ET.SubElement(launch, "node", pkg="rlsim_gazebo", type="rlsim_gazebo_node", name="rlsim_gazebo", output="screen", required="true")
    return ET.tostring(launch, encoding="unicode") + "\n"


def prepare(root, experiment, output=None, terrain_id=None, terrain_parameters=None, policy_run=None, xacro=None, control_override=None, backend_override=None, policy_pd_override=None, runtime_root=None):
    if not policy_run:
        raise ConfigError("prepare/run requires --policy-run for actual model validation")
    resolved, world, terrain = resolve(root, experiment, terrain_id, terrain_parameters, policy_run, control_override, backend_override, policy_pd_override)
    urdf = render_robot(root, resolved["robot"], xacro)
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    run_id = resolved["id"] + "-" + stamp.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
    parent = Path(runtime_root).resolve() if runtime_root else Path(root).resolve() / "tools/runs"
    directory = Path(output).resolve() if output else parent / run_id
    managed_output = directory.parent == Path(root).resolve() / "tools/runs"
    with output_guard(root) if managed_output else nullcontext():
        directory.mkdir(parents=True, exist_ok=False)
        resolved["run_id"] = run_id
        files = {"resolved_config.json": json.dumps(resolved, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                 "policy.yaml": dumps(resolved["policy"]),
                 "robot_runtime.yaml": dumps(resolved["robot_runtime"]),
                 "controllers.yaml": path_in(root, resolved["robot"]["controllers"]).read_text(),
                 "robot.urdf": urdf, "world.sdf": world,
                 "terrain.json": json.dumps(terrain, indent=2, allow_nan=False) + "\n"}
        files["simulation.launch"] = launch_xml(directory, resolved)
        for name, contents in files.items():
            (directory / name).write_text(contents, encoding="utf-8")
        metadata = {"schema_version": 1, "run_id": run_id, "state": "prepared", "created_at": stamp.isoformat(),
                    "artifacts": {name: sha256(directory / name) for name in files}, "exit_code": None}
        (directory / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
        if managed_output:
            try:
                prune_task_outputs(root, protected={directory.name})
            except OSError as error:
                warnings.warn("Task generated, but old task cleanup failed: " + str(error), RuntimeWarning)
    return directory
