"""Versioned resource registry shared by the CLI and future session service."""
import copy
import re
from pathlib import Path

import yaml

from .legacy import ConfigError, UniqueLoader
from .references import resolve_values

FIELDS = {
    "robot": {"schema_version", "kind", "id", "robot_name", "variant", "package", "package_path", "urdf", "xacro", "controllers", "namespace", "model_name", "joints", "groups", "excluded_plugins"},
    "policy": {"schema_version", "kind", "id", "robots", "config"},
    "terrain": {"schema_version", "kind", "id", "generator", "parameters"},
    "backend": {"schema_version", "kind", "id", "adapter", "physics"},
    "control": {"schema_version", "kind", "id", "joints", "timing", "startup", "transition"},
    "experiment": {"schema_version", "kind", "id", "robot", "policy", "control", "backend", "terrain", "seed", "spawn", "runtime", "terrain_parameters"},
    "robot_profile": {"schema_version", "kind", "id", "robot", "control", "policies", "experiments"},
    "simulation": {"schema_version", "kind", "backends", "terrains", "recording"},
}


def keys(data, required, optional=(), context="configuration"):
    if not isinstance(data, dict):
        raise ConfigError(context + " must be a mapping")
    missing = set(required) - set(data)
    unknown = set(data) - set(required) - set(optional)
    if missing or unknown:
        raise ConfigError("{}: missing fields {}; unknown fields {}".format(context, sorted(missing), sorted(unknown)))
    return data


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise ConfigError("Invalid resource identifier: " + repr(value))
    return value


def path_in(root, value, exists=True):
    if not isinstance(value, (str, Path)) or not str(value):
        raise ConfigError("Expected a non-empty path")
    root = Path(root).resolve()
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ConfigError("Resource path escapes workspace: " + str(value)) from error
    if exists and not path.exists():
        raise ConfigError("Resource does not exist: " + str(value))
    return path


def load_yaml(path):
    try:
        filename, separator, pointer = str(path).partition("#")
        data = yaml.load(Path(filename).read_text(encoding="utf-8"), Loader=UniqueLoader)
        if separator:
            if not pointer.startswith("/"):
                raise ConfigError("YAML selection requires #/mapping/key")
            for token in pointer[1:].split("/"):
                key = token.replace("~1", "/").replace("~0", "~")
                if not isinstance(data, dict) or key not in data:
                    raise ConfigError("Unknown YAML selection: " + str(path))
                data = data[key]
        return data
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(str(error)) from error


def load_resource(path, kind):
    data = load_yaml(path)
    if isinstance(data, dict) and data.get("kind") == "robot_profile" and kind == "experiment":
        validate_resource(data, "robot_profile")
        if not isinstance(data.get("experiments", {}), dict) or len(data.get("experiments", {})) != 1:
            raise ConfigError("Select an experiment id or #/experiments/ID")
        return validate_resource(next(iter(data.get("experiments", {}).values())), "experiment")
    if isinstance(data, dict) and data.get("kind") == "robot_profile" and kind in ("robot", "control"):
        resources = profile_resources(data)
        for resource in resources:
            if resource["kind"] == kind:
                return resource
        raise ConfigError("Robot has no reviewed control profile: " + str(path))
    return validate_resource(data, kind)


def validate_resource(data, kind):
    if kind == "experiment":
        common = FIELDS[kind] - {"terrain", "seed", "terrain_parameters"}
        keys(data, common, {"world", "terrain", "seed", "terrain_parameters", "overrides", "model_file"}, context=kind)
        if ("world" in data) == ("terrain" in data):
            raise ConfigError("Experiment requires exactly one of world or terrain")
        if "world" in data:
            if not isinstance(data["world"], str) or not data["world"].endswith(".world"):
                raise ConfigError("world must be a workspace .world file")
            if "terrain_parameters" in data or "seed" in data:
                raise ConfigError("World terrain is defined in the world file; remove seed/terrain_parameters")
        else:
            keys(data, FIELDS[kind], {"overrides", "model_file"}, context=kind)
    else:
        keys(data, FIELDS[kind] - {"experiments"} if kind == "robot_profile" else FIELDS[kind], {"diagnostics", "experiments"} if kind == "robot_profile" else (), context=kind)
    if type(data["schema_version"]) is not int or data["schema_version"] != 1 or data["kind"] != kind:
        raise ConfigError("Unsupported {} schema".format(kind))
    if kind != "simulation":
        identifier(data["id"])
    return data


def profile_resources(data):
    """Project one editable robot profile onto the existing runtime contracts."""
    data = validate_resource(data, "robot_profile")
    common = {"schema_version", "kind", "id"}
    keys(data["robot"], FIELDS["robot"] - common, context="robot profile assets")
    robot = {"schema_version": 1, "kind": "robot", "id": data["id"], **copy.deepcopy(data["robot"])}
    resources = [validate_resource(robot, "robot")]
    if data["control"] is not None:
        keys(data["control"], FIELDS["control"] - {"schema_version", "kind", "joints"}, context="robot profile control")
        control = {"schema_version": 1, "kind": "control", "joints": copy.deepcopy(robot["joints"]), **copy.deepcopy(data["control"])}
        resources.append(validate_resource(control, "control"))
    return resources


def simulation_config(root):
    path = Path(root) / "configs/simulation.yaml"
    return (path, load_resource(path, "simulation")) if path.exists() else (None, None)


def merge(base, patch):
    if not isinstance(patch, dict):
        raise ConfigError("Configuration overrides must be a mapping")
    result = copy.deepcopy(base)
    for key, value in patch.items():
        if key not in base:
            raise ConfigError("Unknown override field: " + key)
        result[key] = merge(base[key], value) if isinstance(base[key], dict) else copy.deepcopy(value)
    return result


def difference(base, value):
    return {key: difference(base[key], item) if isinstance(item, dict) and isinstance(base.get(key), dict) else copy.deepcopy(item)
            for key, item in value.items() if key not in base or item != base[key]}


class Catalog:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.entries = {}
        self.deployments = {}
        def register(path, data):
            key = (data["kind"], data["id"])
            if key in self.entries:
                raise ConfigError("Duplicate {} id: {}".format(*key))
            self.entries[key] = (path, data)
        profiles = sorted(self.root.glob("configs/wheel/*.yaml")) + sorted(self.root.glob("configs/legged/*.yaml"))
        documents = []
        for path in profiles:
            document = load_yaml(path)
            resources = profile_resources(document)
            if bool(document.get("robot", {}).get("groups", {}).get("wheels")) != (path.parent.name == "wheel"):
                raise ConfigError("Robot wheel group does not match its configuration category")
            for data in resources:
                register(path, data)
            for section in ("policies", "experiments"):
                if not isinstance(document.get(section, {}), dict):
                    raise ConfigError("Robot profile " + section + " must be a mapping")
            for policy_id, policy in document["policies"].items():
                identifier(policy_id)
                documents.append((path, document["id"], policy_id, policy))
            for experiment_id, experiment in document.get("experiments", {}).items():
                identifier(experiment_id)
                data = validate_resource(experiment, "experiment")
                if data["id"] != experiment_id or data["robot"] != document["id"] or data["control"] != path.relative_to(self.root).as_posix():
                    raise ConfigError("Experiment must belong to its robot profile and use its control")
                if data["policy"] not in document["policies"]:
                    raise ConfigError("Experiment policy must belong to its robot profile")
                register(path, data)
        for path, owner, policy_id, document in documents:
            metadata = document.get("deployment") if isinstance(document, dict) else None
            keys(metadata, {"id", "robots", "status", "diagnostics", "model_directory"}, context="policy deployment")
            directory = path_in(self.root, metadata["model_directory"], exists=metadata["status"] != "blocked")
            if self.root / "policies" not in directory.parents:
                raise ConfigError("Model directory must be inside policies/")
            if directory.exists() and not directory.is_dir():
                raise ConfigError("Model directory must be a directory")
            identifier(metadata["id"])
            if metadata["id"] != policy_id:
                raise ConfigError("Policy key must match deployment.id")
            if metadata["status"] not in ("ready", "blocked", "unconfigured"):
                raise ConfigError("Unknown policy status: " + str(metadata["status"]))
            if not isinstance(metadata["diagnostics"], list) or not isinstance(metadata["robots"], list):
                raise ConfigError("Policy robots/diagnostics must be lists")
            if not metadata["robots"]:
                raise ConfigError("Policy robots must be nonempty")
            if owner not in metadata["robots"]:
                raise ConfigError("Policy must declare compatibility with its profile robot")
            if any(not isinstance(reason, str) or not reason for reason in metadata["diagnostics"]):
                raise ConfigError("Policy diagnostics must contain nonempty strings")
            if metadata["status"] == "ready" and document.get("id") != metadata["id"]:
                raise ConfigError("Policy id must match deployment.id")
            for robot_id in metadata["robots"]:
                identifier(robot_id)
                if ("robot", robot_id) not in self.entries:
                    raise ConfigError("Unknown policy robot: " + robot_id)
            if len(set(metadata["robots"])) != len(metadata["robots"]):
                raise ConfigError("Policy robots must be unique")
            register(path, {"schema_version": 1, "kind": "policy", "id": metadata["id"],
                            "robots": metadata["robots"], "config": path.relative_to(self.root).as_posix() + "#/policies/" + policy_id})
            self.deployments[policy_id] = metadata
        simulation_path, simulation = simulation_config(self.root)
        if simulation:
            for kind, section in (("backend", "backends"), ("terrain", "terrains")):
                if not isinstance(simulation[section], dict):
                    raise ConfigError(section + " must be a mapping")
                for resource_id, values in simulation[section].items():
                    if not isinstance(values, dict):
                        raise ConfigError(kind + " must be a mapping")
                    data = validate_resource({**values, "schema_version": 1, "kind": kind, "id": resource_id}, kind)
                    # Metadata belongs to the registry, never to the nested values.
                    keys(values, FIELDS[kind] - {"schema_version", "kind", "id"}, context=kind)
                    register(simulation_path, data)
        patterns = {"robot": "robots/*/*/manifest.yaml", "terrain": "configs/terrains/*/manifest.yaml",
                    "backend": "configs/backends/*.yaml", "control": "configs/controls/*.yaml"}
        for kind, pattern in patterns.items():
            for path in sorted(self.root.glob(pattern)):
                data = load_resource(path, kind)
                register(path, data)

    def get(self, kind, resource_id):
        identifier(resource_id)
        try:
            path, data = self.entries[(kind, resource_id)]
            path_in(self.root, path)
            return path, data
        except KeyError as error:
            raise ConfigError("Unknown {}: {}".format(kind, resource_id)) from error

    def list(self):
        return [{"kind": kind, "id": resource_id,
                 "manifest": data["config"] if kind == "policy" else path.relative_to(self.root).as_posix() + ("#/experiments/" + resource_id if kind == "experiment" else ""),
                 "data": data, **({"deployment": self.deployments[resource_id]} if kind == "policy" else {})}
                for (kind, resource_id), (path, data) in sorted(self.entries.items())]
