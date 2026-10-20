"""Foreground session lifecycle. Only terminates the child it starts."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import tempfile

from .catalog import ConfigError
from .experiment import ros_environment
from .validation import sha256
from .runtime import quiet_ros_environment
from .retention import output_guard, output_lease

COMMANDS = ("stand", "policy", "sit", "passive", "reset", "toggle")


def verify_run(directory):
    directory = Path(directory).resolve()
    metadata = json.loads((directory / "run.json").read_text())
    resolved = json.loads((directory / "resolved_config.json").read_text())
    if metadata.get("schema_version") != 1 or resolved.get("schema_version") != 1:
        raise ConfigError("Unsupported run metadata schema")
    if metadata["state"] != "prepared":
        raise ConfigError("Only a prepared run may be started; prepare a new directory to rerun")
    required = {"resolved_config.json", "policy.yaml", "robot_runtime.yaml", "controllers.yaml", "robot.urdf", "world.sdf", "terrain.json", "simulation.launch"}
    if set(metadata["artifacts"]) != required:
        raise ConfigError("Prepared run artifact inventory is incomplete")
    for name, digest in metadata["artifacts"].items():
        path = (directory / name).resolve()
        if path.parent != directory or not path.is_file() or sha256(path) != digest:
            raise ConfigError("Prepared artifact changed: " + name)
    root = Path(resolved["workspace_root"]).resolve()
    for name, digest in resolved["fingerprints"].items():
        path = (root / name).resolve()
        if root not in path.parents or not path.is_file() or sha256(path) != digest:
            raise ConfigError("Source resource changed since preparation: " + name)
    return metadata, resolved


def _state(directory, metadata, state, **fields):
    metadata.update(fields, state=state)
    temporary = directory / "run.json.tmp"
    temporary.write_text(json.dumps(metadata, indent=2) + "\n")
    temporary.replace(directory / "run.json")


def launch_run(directory, stop_event=None, roslaunch=None):
    directory = Path(directory).resolve()
    metadata, resolved = verify_run(directory)
    program = roslaunch or shutil.which("roslaunch")
    if not program:
        raise ConfigError("roslaunch not found; source the built ROS workspace")
    env = ros_environment(resolved["workspace_root"], resolved["robot"])
    with output_guard(resolved["workspace_root"]):
        lease = output_lease(resolved["workspace_root"], directory.name)
    if lease is None:
        raise ConfigError("Task output is being cleaned up; prepare a new task")
    with lease, tempfile.TemporaryDirectory(prefix="rlsim-ros-") as scratch:
        quiet_ros_environment(env, scratch)
        # Current plugin namespaces/global Gazebo topics support one managed run per ROS master.
        lock_root = Path(tempfile.gettempdir()) / ("rlsim-locks-" + str(os.getuid()))
        lock_root.mkdir(parents=True, exist_ok=True)
        master = env.get("ROS_MASTER_URI", "http://localhost:11311")
        lock_path = lock_root / (hashlib.sha256(master.encode()).hexdigest() + ".lock")
        with lock_path.open("a+") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise ConfigError("Another managed simulation already owns this ROS master") from error
            # A second contender may have validated before the first acquired this lock.
            metadata, resolved = verify_run(directory)
            _state(directory, metadata, "starting", ros_master_uri=master)
            process = None
            reason = "process_exit"
            try:
                with open(os.devnull, "w") as log:
                    # Let roslaunch finish its own node shutdown/escalation before
                    # the outer process-group watchdog expires.
                    command = [str(program)] + ([] if roslaunch else ["--screen", "--sigint-timeout", "5", "--sigterm-timeout", "3"]) + [str(directory / "simulation.launch")]
                    process = subprocess.Popen(command,
                                               stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                    _state(directory, metadata, "running", pid=process.pid)
                    deadline = time.monotonic() + resolved["experiment"]["runtime"]["duration_s"]
                    try:
                        while process.poll() is None:
                            if stop_event is not None and stop_event.is_set():
                                reason = "requested_stop"; break
                            if time.monotonic() >= deadline:
                                reason = "duration_elapsed"; break
                            time.sleep(.1)
                    except KeyboardInterrupt:
                        reason = "interrupt"
                    if process.poll() is None:
                        _state(directory, metadata, "stopping", stop_reason=reason)
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGTERM)
                            try:
                                process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL); process.wait()
                    code = process.wait()
                    successful_stop = reason != "process_exit" and code in (0, -signal.SIGINT, -signal.SIGTERM)
                    state = "finished" if code == 0 or successful_stop else "failed"
                    _state(directory, metadata, state, exit_code=code, stop_reason=reason)
            except Exception as error:
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
                _state(directory, metadata, "failed", error=str(error), exit_code=process.returncode if process else None)
                raise
        return metadata


def send_command(directory, command, rostopic=None):
    if command not in COMMANDS:
        raise ConfigError("Unknown control command")
    directory = Path(directory).resolve()
    metadata = json.loads((directory / "run.json").read_text())
    if metadata.get("state") != "running":
        raise ConfigError("Control commands require a running session")
    resolved = json.loads((directory / "resolved_config.json").read_text())
    program = rostopic or shutil.which("rostopic")
    if not program:
        raise ConfigError("rostopic not found")
    env = ros_environment(resolved["workspace_root"], resolved["robot"])
    env["ROS_MASTER_URI"] = metadata["ros_master_uri"]
    try:
        result = subprocess.run([str(program), "pub", "-1", resolved["robot"]["namespace"] + "control", "std_msgs/String", json.dumps({"data": command})],
                                env=env, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ConfigError("Command publication failed: " + str(error)) from error
    if result.returncode:
        raise ConfigError("Command publication failed: " + result.stderr.strip())
    return {"command": command, "publication": "completed", "transition_acknowledged": False}
