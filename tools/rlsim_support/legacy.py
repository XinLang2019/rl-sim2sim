"""Audit the archived C++ loader's configuration without rewriting it.

The exported contract describes legacy behaviour, including its limitations.
It is not yet a new deployment format accepted by the C++ runtime.
"""
import hashlib
import json
import math
import subprocess
from pathlib import Path

import yaml
from .relocation import current_path

RUNNERS = ("rl_sim", "rl_wheel", "rl_amp")
POLICY_TYPES = ("dreamwaq", "ts", "ts_se")


class ConfigError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if not isinstance(key, str):
            raise ConfigError("YAML keys must be strings")
        if key in result:
            raise ConfigError("Duplicate YAML key: " + key)
        result[key] = loader.construct_object(value_node, deep=True)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def keyed_yaml(path, key):
    data = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
    if not isinstance(data, dict) or key not in data:
        raise ConfigError("{}: expected top-level key {!r}, found {}".format(
            path.name, key, list(data) if isinstance(data, dict) else type(data).__name__))
    if not isinstance(data[key], dict):
        raise ConfigError("{}: {!r} must be a mapping".format(path.name, key))
    return data[key]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(root, path):
    return path.resolve().relative_to(root.resolve()).as_posix()


def number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(name + " must be a finite number")
    if positive and value <= 0:
        raise ConfigError(name + " must be positive")
    return value


def integer(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, int) or value < (1 if positive else 0):
        raise ConfigError(name + (" must be a positive integer" if positive else " must be a non-negative integer"))
    return value


def vector(value, width, name, strings=False):
    if not isinstance(value, list) or len(value) != width:
        raise ConfigError("{} must contain {} elements".format(name, width))
    for item in value:
        if strings:
            if not isinstance(item, str) or not item:
                raise ConfigError(name + " must contain non-empty names")
        else:
            number(item, name)
    if strings and len(set(value)) != len(value):
        raise ConfigError(name + " contains duplicate names")
    return value


def indices(value, width, name):
    if not isinstance(value, list):
        raise ConfigError(name + " must be a list")
    for item in value:
        integer(item, name)
        if item >= width:
            raise ConfigError(name + " contains out-of-range index " + str(item))
    if len(set(value)) != len(value):
        raise ConfigError(name + " contains duplicates")
    return value


def probe_model(probe, model, sample=None, shapes=None):
    args = [str(Path(probe).resolve()), str(model.resolve())]
    if sample:
        args += ["--sample", sample]
    for name, shape in sorted((shapes or {}).items()):
        args += ["--shape", name + "=" + ",".join(map(str, shape))]
    try:
        result = subprocess.run(args, text=True, capture_output=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ConfigError("Model probe unavailable: " + str(error)) from error
    if result.returncode:
        raise ConfigError(result.stderr.strip() or "Model probe failed")
    try:
        return json.loads(result.stdout)
    except ValueError as error:
        raise ConfigError("Model probe returned invalid JSON") from error


def _model_contract(metadata, contract, issue):
    """Validate the exact legacy bindings, not inferred model semantics."""
    kind = contract["policy_type"]
    obs_width = contract["observation"]["model_input_width"]
    n = contract["robot"]["dofs"]
    inputs, outputs = metadata["inputs"], metadata["outputs"]
    if kind == "dreamwaq":
        # OnnxInference supplies buffers by model input order.
        if len(inputs) != 2:
            issue("error", "MODEL_INPUTS", "rl_wheel dreamwaq requires exactly two ordered model inputs")
        expected_inputs = {t["name"]: [1, width] for t, width in zip(inputs, [obs_width, 365])}
        expected_outputs = {"action": [1, n]}
        contract["bindings"] = {"inputs_by_position": ["current_observation", "history"], "outputs": ["action"]}
    else:
        names = ["obs", "h", "c"] + (["mh", "mc"] if kind == "ts_se" else [])
        out_names = ["actions", "he", "ce"] + (["mhe", "mce"] if kind == "ts_se" else [])
        expected_inputs = {name: ([1, obs_width] if name == "obs" else None) for name in names}
        expected_outputs = {name: ([1, n] if name == "actions" else None) for name in out_names}
        contract["bindings"] = {"inputs": names, "outputs": out_names,
                                "state_feedback": {"he": "h", "ce": "c"}}
        if kind == "ts_se":
            issue("error", "LEGACY_STATE_ALIAS", "ts_se reuses h/c for mh/mc and reads mhe/mce from output indices 1/2; independent state semantics are not established")
    for direction, actual, expected in (("input", inputs, expected_inputs), ("output", outputs, expected_outputs)):
        names = [item["name"] for item in actual]
        if set(names) != set(expected):
            issue("error", "MODEL_BINDING", "{} names {} do not match legacy bindings {}".format(direction, names, list(expected)))
        for item in actual:
            if item["dtype"] != "float32":
                issue("error", "MODEL_DTYPE", item["name"] + " must be float32 for the legacy runtime")
            shape = expected.get(item["name"])
            if shape is not None and (len(shape) != len(item["shape"]) or any(
                    a >= 0 and a != b for a, b in zip(item["shape"], shape))):
                issue("error", "MODEL_SHAPE", "{} shape {} does not match {}".format(item["name"], item["shape"], shape))
    if kind != "dreamwaq":
        ins = {item["name"]: item["shape"] for item in inputs}
        outs = {item["name"]: item["shape"] for item in outputs}
        for output, input_name in (("he", "h"), ("ce", "c")):
            if input_name in ins and output in outs and ins[input_name] != outs[output]:
                issue("error", "MODEL_STATE_SHAPE", output + " and " + input_name + " shapes differ")
        if "h" in ins and "c" in ins and ins["h"] != ins["c"]:
            issue("error", "MODEL_STATE_SHAPE", "Legacy hidden/cell initialization expects matching shapes")


def audit(root, profile, runner="rl_wheel", policy_type=None, probe=None):
    root = Path(root).resolve()
    diagnostics = []
    result = {"schema_version": 1, "profile": profile, "runner": runner,
              "contract": None, "diagnostics": diagnostics, "model_check": "not_run"}

    def issue(level, code, message):
        diagnostics.append({"level": level, "code": code, "message": message})

    try:
        parts = profile.split("/")
        if len(parts) != 2 or any(part in ("", ".", "..") for part in parts) or "\\" in profile:
            raise ConfigError("Profile must be ROBOT/PROFILE")
        if runner not in RUNNERS:
            raise ConfigError("Unknown legacy runner: " + runner)
        robot, _ = parts
        directory = root / "tools/archive/policy" / profile
        base_path = directory.parent / "base.yaml"
        config_path = directory / "config.yaml"
        base = keyed_yaml(base_path, robot)
        config = keyed_yaml(config_path, profile)
        if "policy_type" not in base:
            issue("error", "MISSING_POLICY_TYPE", "base.yaml lacks policy_type required by ReadYamlBase; explicit inspection override does not fix runtime loading")
        kind = policy_type or base.get("policy_type")
        if policy_type and policy_type != base.get("policy_type"):
            issue("warning", "INSPECTION_OVERRIDE", "Inspecting {} instead of configured {}; source files are unchanged".format(kind, base.get("policy_type")))
        if kind not in POLICY_TYPES:
            raise ConfigError("Select a supported policy_type: " + ", ".join(POLICY_TYPES))
        dt = number(base["dt"], "dt", positive=True)
        decimation = integer(base["decimation"], "decimation", positive=True)
        base_n = integer(base["num_of_dofs"], "base.num_of_dofs", positive=True)
        n = integer(config["num_of_dofs"], "num_of_dofs", positive=True)
        if n != base_n:
            raise ConfigError("Policy and robot DOF counts differ")
        # Preserve both copies: ReadYamlRL overrides these after startup.
        for key in ("fixed_kp", "fixed_kd", "torque_limits", "default_dof_pos"):
            vector(base[key], n, "base." + key)
        joint_names = vector(base["joint_names"], n, "joint_names", strings=True)
        controllers = vector(base["joint_controller_names"], n, "joint_controller_names", strings=True)
        for label, data in (("base", base), ("policy", config)):
            mapping = indices(data["joint_mapping"], n, label + ".joint_mapping")
            if len(mapping) != n:
                raise ConfigError(label + ".joint_mapping must be a permutation")
        wheel = indices(config.get("wheel_indices") or [], n, "wheel_indices")
        joints = indices(config.get("joint_indices") or [i for i in range(n) if i not in wheel], n, "joint_indices")
        if set(wheel) & set(joints) or set(wheel + joints) != set(range(n)):
            raise ConfigError("Joint and wheel groups must partition all DOFs")
        base_wheel = indices(base.get("wheel_indices") or [], n, "base.wheel_indices")
        base_joints = indices(base.get("joint_indices") or [i for i in range(n) if i not in base_wheel], n, "base.joint_indices")
        if set(base_wheel) & set(base_joints) or set(base_wheel + base_joints) != set(range(n)):
            raise ConfigError("Base joint and wheel groups must partition all DOFs")
        values = {}
        for key in ("action_scale", "rl_kp", "rl_kd", "fixed_kp", "fixed_kd", "torque_limits", "default_dof_pos"):
            values[key] = vector(config[key], n, key)
        for key in ("rl_kp", "rl_kd", "fixed_kp", "fixed_kd", "torque_limits"):
            if any(v < 0 or (key == "torque_limits" and v == 0) for v in values[key]):
                raise ConfigError(key + " contains invalid negative or zero values")
        for key in ("lin_vel_scale", "ang_vel_scale", "dof_pos_scale", "dof_vel_scale"):
            values[key] = number(config[key], key)
        values["commands_scale"] = vector(config["commands_scale"], 3, "commands_scale")
        clip = number(config["clip_obs"], "clip_obs", positive=True)
        lower, upper = config.get("clip_actions_lower"), config.get("clip_actions_upper")
        if lower is not None or upper is not None:
            vector(lower, n, "clip_actions_lower")
            vector(upper, n, "clip_actions_upper")
            if any(a > b for a, b in zip(lower, upper)):
                raise ConfigError("Action lower bound exceeds upper bound")
        suffix = "" if kind == "dreamwaq" else "_ts"
        terms = config["observations" + suffix]
        if not isinstance(terms, list) or not terms:
            raise ConfigError("observations must be a non-empty list")
        dims = {"lin_vel": 3, "ang_vel": 3, "ang_vel_body": 3, "ang_vel_world": 3,
                "gravity_vec": 3, "commands": 3, "dof_pos": n, "dof_pos_ori": n,
                "dof_vel": n, "actions": n, "phase": 6, "g1_phase": 2, "g1_mimic_phase": 1}
        unknown = [term for term in terms if not isinstance(term, str) or term not in dims]
        if unknown:
            raise ConfigError("Unknown observation terms: " + repr(unknown))
        width = sum(dims[term] for term in terms)
        declared = integer(config["num_observations" + suffix], "num_observations", positive=True)
        if width != declared:
            issue("error", "OBSERVATION_WIDTH", "Declared observation width {} != computed {}".format(declared, width))
        offsets = config.get("observations_history" + suffix) or []
        if not isinstance(offsets, list):
            raise ConfigError("History offsets must be a list")
        for offset in offsets:
            integer(offset, "history offset")
        priority = config["observations_history_priority"]
        if priority not in ("time", "term"):
            raise ConfigError("History priority must be time or term")
        model_name = config["model_name" if kind == "dreamwaq" else "model_name_ts"]
        if not isinstance(model_name, str) or not model_name:
            raise ConfigError("Model filename must be a non-empty string")
        model_path = (root / current_path("policies/" + profile + "/" + model_name)).resolve()
        model_rel = relative(root, model_path)
        if not model_path.is_file():
            issue("error", "MODEL_MISSING", model_rel + " does not exist")
        elif model_path.suffix.lower() != ".onnx":
            issue("error", "LEGACY_MODEL_FORMAT", "Current InitRL/InitRL_TS uses ONNX Runtime but selected file is " + model_path.suffix)
        if runner == "rl_sim" and kind == "dreamwaq":
            issue("error", "LEGACY_EXECUTOR_MISMATCH", "InitRL loads ONNX but RL_Sim::Forward calls the TorchScript model")
        if runner == "rl_amp" and kind == "dreamwaq":
            issue("error", "RUNNER_POLICY", "rl_amp rejects dreamwaq")
        if kind == "ts_se" and runner != "rl_amp":
            issue("error", "RUNNER_POLICY", "Only rl_amp dispatches ts_se inference; FSM initialization also needs migration")
        history = {"offsets": offsets, "layout": priority + "_major", "includes_current": 0 in offsets,
                   "initialization": "zeros", "sampling": "policy_tick"}
        input_width = width * (len(offsets) if offsets else 1)
        if kind == "dreamwaq" and runner == "rl_wheel":
            history = {"offsets": [4, 3, 2, 1, 0], "layout": "time_major", "includes_current": True,
                       "initialization": "uninitialized_legacy_heap", "sampling": "policy_tick"}
            input_width = width
            issue("warning", "LEGACY_HISTORY_INITIALIZATION", "rl_wheel allocates histPtr without initializing; first four policy ticks have no deterministic history baseline")
            if width != 73:
                issue("error", "LEGACY_HISTORY_WIDTH", "rl_wheel dreamwaq memcpy requires 73 features, configuration computes " + str(width))
        if kind == "ts_se":
            issue("error", "LEGACY_TS_SE_INIT", "Existing FSM Enter only initializes dreamwaq/ts, not ts_se")
            if offsets:
                issue("error", "LEGACY_TS_SE_HISTORY", "rl_amp with history calls three-input TS inference even for ts_se")
        if "lin_vel" in terms:
            issue("warning", "LINEAR_VELOCITY_SOURCE", "Current RunModel does not populate obs.lin_vel; it remains at initialization")
        if "dof_pos_ori" in terms and wheel:
            issue("warning", "LEGACY_OBSERVATION_ALIAS", "dof_pos_ori aliases obs.dof_pos and zeroes wheel positions in place")
        if "g1_mimic_phase" in terms:
            issue("error", "MOTION_LENGTH_UNRESOLVED", "g1_mimic_phase requires motion_length not defined by this YAML loader")
        fsm_path = directory.parent / "fsm.hpp"
        if fsm_path.is_file() and 'rl.config_name = "legged_gym"' in fsm_path.read_text():
            if parts[1] != "legged_gym":
                issue("error", "FSM_PROFILE", "Existing robot FSM hardcodes legged_gym; requested profile is inspection-only")
        overrides = [key for key in config if key in base and config[key] != base[key]]
        if overrides:
            issue("warning", "POLICY_OVERRIDES_BASE", "ReadYamlRL overrides base values: " + ", ".join(sorted(overrides)))
        contract = {
            "schema_version": 1, "kind": "legacy_policy_contract", "profile": profile,
            "runner": runner, "policy_type": kind, "model": model_rel,
            "timing": {"control_dt": dt, "decimation": decimation, "policy_dt": dt * decimation,
                       "scheduler": "independent_wall_clock_threads"},
            "robot": {"id": robot, "dofs": n, "joint_names": joint_names,
                      "controller_names": controllers, "joint_mapping": config["joint_mapping"],
                      "wheel_indices": wheel, "joint_indices": joints},
            "observation": {"terms": [{"name": term, "width": dims[term]} for term in terms],
                            "width": width, "declared_width": declared, "model_input_width": input_width,
                            "quaternion_order": "wxyz", "ang_vel_source_frame": "world",
                            "history": history, "clip": clip,
                            "previous_action_stage": "clipped_model_output"},
            "action": {"joint_mode": "position_offset", "wheel_mode": "velocity",
                       "clip_lower": lower, "clip_upper": upper},
            "parameters": values, "base_parameters": base,
            "sources": [relative(root, base_path), relative(root, config_path)],
            "bindings": None,
            "reset": {"world_reset": "not_coordinated_with_policy_state",
                      "policy_entry": "reinitializes_model_and_buffers"},
        }
        result["contract"] = contract
        if probe and model_path.is_file() and model_path.suffix.lower() == ".onnx":
            metadata = probe_model(probe, model_path)
            result["model"] = metadata
            _model_contract(metadata, contract, issue)
            result["model_check"] = "inspected"
        elif not probe:
            issue("warning", "MODEL_UNCHECKED", "Model I/O not inspected; supply --probe to check actual names, dtypes and shapes")
    except (ConfigError, KeyError, OSError, yaml.YAMLError, ValueError, TypeError) as error:
        issue("error", "CONFIG_INVALID", str(error))
    result["ok"] = not any(item["level"] == "error" for item in diagnostics)
    return result


def inventory(root, runner="rl_wheel", probe=None):
    root = Path(root).resolve()
    reports = [audit(root, str(path.parent.relative_to(root / "tools/archive/policy")), runner, probe=probe)
               for path in sorted((root / "tools/archive/policy").glob("*/*/config.yaml"))]
    return {"schema_version": 1, "runner": runner, "reports": reports,
            "summary": {"profiles": len(reports), "passed": sum(item["ok"] for item in reports),
                        "failed": sum(not item["ok"] for item in reports)}}
