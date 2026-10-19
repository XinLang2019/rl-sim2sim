"""Workspace-relative YAML value references, shared by editable resource files.

Only whole-value {$ref: path#/mapping/key} substitutions are supported. References
are resolved before schema validation; prepared snapshots contain plain values.
"""
from pathlib import Path

from .legacy import ConfigError, UniqueLoader
import yaml


def resolve_values(value, root, dependencies=None, stack=(), depth=0):
    if depth > 64:
        raise ConfigError("YAML reference nesting exceeds 64")
    if isinstance(value, dict):
        if "$ref" in value:
            if set(value) != {"$ref"} or not isinstance(value["$ref"], str):
                raise ConfigError("$ref must be the only field and contain a string")
            reference = value["$ref"]
            filename, separator, pointer = reference.partition("#")
            if not separator or not filename or not pointer.startswith("/"):
                raise ConfigError("Expected $ref: workspace/path.yaml#/mapping/key")
            if reference in stack:
                raise ConfigError("Cyclic YAML reference: " + reference)
            root = Path(root).resolve()
            path = (root / filename).resolve()
            if Path(filename).is_absolute() or root not in path.parents:
                raise ConfigError("Reference escapes workspace: " + filename)
            if dependencies is not None:
                dependencies.add(path)
            try:
                selected = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
                for part in pointer[1:].split("/"):
                    key = part.replace("~1", "/").replace("~0", "~")
                    if not isinstance(selected, dict) or key not in selected:
                        raise ConfigError("Unknown reference pointer: " + reference)
                    selected = selected[key]
            except (OSError, yaml.YAMLError) as error:
                raise ConfigError(str(error)) from error
            return resolve_values(selected, root, dependencies, stack + (reference,), depth + 1)
        return {key: resolve_values(item, root, dependencies, stack, depth + 1) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_values(item, root, dependencies, stack, depth + 1) for item in value]
    return value
