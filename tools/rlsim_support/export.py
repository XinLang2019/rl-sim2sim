"""Explicit legacy-to-v1 conversion; unsupported semantics fail closed."""
from .legacy import ConfigError, audit


def export_policy(root, profile, runner, policy_type, probe, joint_names=None):
    report = audit(root, profile, runner, policy_type, probe)
    if not report["ok"] or report["model_check"] != "inspected":
        raise ConfigError("Export requires a passing configuration and actual model inspection")
    contract = report["contract"]
    if contract["policy_type"] != "ts":
        raise ConfigError("Automatic migration currently supports validated TS contracts only; author other v1 interfaces explicitly")
    model = report["model"]
    port_shapes = {item["name"]: item["shape"] for item in model["inputs"]}
    if any(dim <= 0 for shape in port_shapes.values() for dim in shape):
        raise ConfigError("Resolve dynamic model dimensions explicitly before migration")
    params = contract["parameters"]
    if joint_names is not None:
        from .legacy import vector
        vector(joint_names, contract["robot"]["dofs"], "explicit policy joint order", strings=True)
        if set(joint_names) != set(contract["robot"]["joint_names"]):
            raise ConfigError("Explicit policy joint order must name the same robot joints")
    terms = []
    sources = {
        "ang_vel": ("angular_velocity", "ang_vel_scale", True),
        "ang_vel_world": ("angular_velocity", "ang_vel_scale", True),
        "ang_vel_body": ("angular_velocity", "ang_vel_scale", False),
        "gravity_vec": ("gravity", None, True),
        "commands": ("command", "commands_scale", False),
        "dof_pos": ("joint_position_relative", "dof_pos_scale", False),
        "dof_vel": ("joint_velocity", "dof_vel_scale", False),
        "actions": ("previous_action", None, False),
    }
    for term in contract["observation"]["terms"]:
        if term["name"] not in sources:
            raise ConfigError("Source requires an explicit migration decision: " + term["name"])
        source, scale, rotate = sources[term["name"]]
        item = {"source": source, "width": term["width"]}
        if scale:
            item["scale"] = params[scale]
        if rotate:
            item["rotate_inverse"] = True
        if term["name"] == "dof_pos":
            item["zero_indices"] = contract["robot"]["wheel_indices"]
        terms.append(item)
    history = contract["observation"]["history"]
    if history["initialization"] != "zeros":
        raise ConfigError("Uninitialized history cannot become a deterministic migration contract")
    actions = {
        "count": contract["robot"]["dofs"], "output": "actions",
        "reference": params["default_dof_pos"], "kp": params["rl_kp"], "kd": params["rl_kd"],
        "torque_limits": params["torque_limits"], "diagnostic": "legacy_reference_pd",
        "channels": [{"joint": i, "action": i,
                      "mode": "velocity" if i in contract["robot"]["wheel_indices"] else "position_offset",
                      "scale": scale} for i, scale in enumerate(params["action_scale"])],
    }
    if contract["action"]["clip_lower"] is not None:
        actions["clip_lower"] = contract["action"]["clip_lower"]
        actions["clip_upper"] = contract["action"]["clip_upper"]
    return {
        "schema_version": 1, "id": contract["robot"]["id"] + "-ts-v001",
        "model": {"backend": "onnxruntime", "file": contract["model"]},
        "joints": list(joint_names) if joint_names is not None else [contract["robot"]["joint_names"][i] for i in contract["robot"]["joint_mapping"]],
        "observations": {"proprio": {"terms": terms, "clip": contract["observation"]["clip"],
                                     "history": {"offsets": history["offsets"], "layout": history["layout"], "initialization": "zeros"}}},
        "inputs": {"obs": "proprio"},
        "states": [{"input": name, "output": output, "shape": port_shapes[name]}
                   for name, output in (("h", "he"), ("c", "ce"))],
        "actions": actions,
    }
