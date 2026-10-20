"""Shared strict YAML loading, numeric validation and resource fingerprints."""
import hashlib
import math

import yaml


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


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
