"""Disposable ROS directories and log suppression for owned simulation children."""
import json
from pathlib import Path


def quiet_ros_environment(env, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    python_config = directory / "python_logging.yaml"
    python_config.write_text(json.dumps({"version": 1, "disable_existing_loggers": True,
        "handlers": {"discard": {"class": "logging.NullHandler"}},
        "root": {"level": "CRITICAL", "handlers": ["discard"]},
        "loggers": {"rosout": {"level": "CRITICAL", "handlers": ["discard"], "propagate": False}}}))
    cpp_config = directory / "rosconsole.config"
    cpp_config.write_text("log4j.logger.ros=OFF\n")
    env.update(ROS_HOME=str(directory), ROS_LOG_DIR=str(directory / "log"),
               GAZEBO_LOG_PATH=str(directory / "gazebo"),
               ROS_PYTHON_LOG_CONFIG_FILE=str(python_config), ROSCONSOLE_CONFIG_FILE=str(cpp_config))
    return env
