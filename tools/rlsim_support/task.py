"""Copy a shared, unbound template into one fully selected task configuration."""
import copy
from pathlib import Path

from .catalog import Catalog, ConfigError, FIELDS, keys, load_yaml, validate_resource


def task_template(root):
    document = load_yaml(Path(root) / "configs/task.yaml")
    keys(document, {"schema_version", "kind", "task"}, context="task template")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1 or document["kind"] != "task_template":
        raise ConfigError("Unsupported task template schema")
    template = document["task"]
    required = FIELDS["experiment"] - {"terrain", "seed", "terrain_parameters"}
    keys(template, required | {"world"}, context="task defaults")
    if template["schema_version"] != 1 or template["kind"] != "experiment":
        raise ConfigError("Task defaults require experiment schema 1")
    # A template must not silently select a resource or a task identity.
    for key in ("id", "robot", "policy", "control", "world"):
        if template[key] is not None:
            raise ConfigError("Task template selection must be null: " + key)
    return copy.deepcopy(template)


def complete_task(root, selections):
    keys(selections, set(), FIELDS["experiment"] | {"world", "overrides", "model_file"}, "task selections")
    task = task_template(root)
    for key, value in selections.items():
        if key in ("spawn", "runtime") and isinstance(value, dict):
            task[key].update(copy.deepcopy(value))
        else:
            task[key] = copy.deepcopy(value)
    for key in ("robot", "policy"):
        if not task.get(key):
            raise ConfigError("Task requires selection: " + key)
    if not task.get("world"):
        task.pop("world", None)
        if not task.get("terrain"):
            raise ConfigError("Task requires a world or terrain selection")
        task.setdefault("seed", 0)
        task.setdefault("terrain_parameters", {})
    if not task.get("control"):
        path, _ = Catalog(root).get("robot", task["robot"])
        task["control"] = path.relative_to(Path(root).resolve()).as_posix()
    if not task.get("id"):
        task["id"] = task["robot"] + "-task"
    return validate_resource(task, "experiment")
