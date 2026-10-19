"""Owned child shutdown and the allowed robot CSV data sources."""
import os
from pathlib import Path
import signal
import subprocess

from rlsim_support.catalog import ConfigError, keys, load_yaml, simulation_config


def stop_child(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            process.wait(); return
        except subprocess.TimeoutExpired:
            continue
    raise ConfigError("Child did not terminate")


def recording_topics(root, resolved):
    _, simulation = simulation_config(root)
    config = ({"schema_version": 1, **simulation["recording"]} if simulation else load_yaml(Path(root) / "configs/recordings/default.yaml"))
    keys(config, {"schema_version", "topics", "include_joint_topics", "buffer_mb"}, context="recording")
    if type(config["schema_version"]) is not int or config["schema_version"] != 1 or type(config["include_joint_topics"]) is not bool:
        raise ConfigError("Invalid recording profile")
    if type(config["buffer_mb"]) is not int or not 1 <= config["buffer_mb"] <= 512:
        raise ConfigError("Recording buffer must be 1..512 MB")
    if not isinstance(config["topics"], list) or not config["topics"]:
        raise ConfigError("Recording requires explicit topics")
    namespace = resolved["robot"]["namespace"]
    topics = []
    for topic in config["topics"]:
        if not isinstance(topic, str):
            raise ConfigError("Invalid recording topic")
        topic = topic.replace("{namespace}", namespace)
        if not topic.startswith("/") or any(c not in "/_0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ" for c in topic):
            raise ConfigError("Invalid recording topic")
        topics.append(topic)
    if config["include_joint_topics"]:
        for controller in resolved["robot_runtime"]["controller_names"]:
            topics.extend(namespace + controller + suffix for suffix in ("/command", "/state"))
    return sorted(set(topics)), config["buffer_mb"]


# The public factory remains stable for the session API. No rosbag subprocess.
from .csv_recording import CsvRecorder as Recorder
