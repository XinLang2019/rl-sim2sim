"""Single-owner local sessions shared by HTTP handlers; no ROS imports here."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid
import tempfile
import shutil
from collections import defaultdict, deque
import xml.etree.ElementTree as ET

from rlsim_support.catalog import Catalog, ConfigError, identifier, keys, load_yaml, path_in
from rlsim_support.experiment import prepare, resolve, ros_environment
from rlsim_support.session import COMMANDS, launch_run, verify_run
from rlsim_support.task import task_template
from rlsim_support.policy_assets import model_inventory
from .reset_pose import validate_reset_pose
from .render_config import validate_camera, camera_defaults
from .video_recording import VideoRecorder
from .controller_lifecycle import ControllerLifecycle
from .processes import Recorder, stop_child, recording_topics
from rlsim_support.references import resolve_values
from rlsim_support.runtime import quiet_ros_environment
from rlsim_support.retention import output_guard, output_lease, prune_task_outputs, task_output_limit, set_task_output_limit
from rlsim_support.catalog import TASK_OUTPUT_LIMIT_OPTIONS

DOWNLOADS = {"resolved_config.json", "parameters.log", "recording.log", "recording.json"}

def read_json(path):
    return json.loads(Path(path).read_text())


def tail(path, limit=32000):
    if not Path(path).is_file():
        return ""
    with Path(path).open("rb") as stream:
        stream.seek(0, 2); size = stream.tell()
        stream.seek(max(0, size - limit))
        return stream.read(limit).decode("utf-8", errors="replace")


class Manager(ControllerLifecycle):
    def __init__(self, root, policy_run, ros_python="/usr/bin/python3", xacro=None, launcher=launch_run, worker_script=None, recorder_factory=Recorder):
        self.root = Path(root).resolve()
        self.policy_run, self.ros_python, self.xacro = policy_run, ros_python, xacro
        self.launcher, self.recorder_factory = launcher, recorder_factory
        self.worker_script = Path(worker_script) if worker_script else self.root / "tools/web/services/ros_worker.py"
        self.lock = threading.RLock()
        self.frame_changed = threading.Condition(self.lock)
        self.sessions = {}
        self.closed = False
        self.scratch = tempfile.TemporaryDirectory(prefix="rlsim-workbench-")
        self.runtime_paths = {}
        self.terminal_metadata = {}
        self.terrain_cache = {}
        self.event_buffer = defaultdict(lambda: deque(maxlen=200))

    def directory(self, run_id):
        identifier(run_id)
        with self.lock:
            if run_id in self.runtime_paths:
                if not (self.archive(run_id) / "resolved_config.json").is_file():
                    raise ConfigError("Task output was removed by retention; generate a new task")
                return self.runtime_paths[run_id]
        directory = path_in(self.root / "tools/runs", run_id)
        if directory.parent != self.root / "tools/runs" or not (directory / "resolved_config.json").is_file():
            raise ConfigError("Unknown run")
        return directory

    def archive(self, run_id):
        identifier(run_id)
        return path_in(self.root / "tools/runs", run_id, exists=False)

    def metadata(self, run_id, directory):
        if (directory / "run.json").is_file():
            return read_json(directory / "run.json")
        return self.terminal_metadata.get(run_id, {"schema_version": 1, "run_id": run_id, "state": "archived"})

    def catalog(self):
        catalog = Catalog(self.root)
        items = catalog.list()
        details, limits = {}, {}
        for item in items:
            try:
                if item["kind"] == "policy" and item["deployment"]["status"] == "ready":
                    details[item["id"]] = resolve_values(load_yaml(self.root / item["manifest"]), self.root)
                if item["kind"] == "robot":
                    robot = item["data"]
                    tree = ET.parse(path_in(self.root, robot["urdf"])).getroot()
                    limits[item["id"]] = {joint.get("name"): {key: float(value) for key, value in joint.find("limit").attrib.items()}
                        for joint in tree.findall("joint") if joint.find("limit") is not None}
                    item["availability"] = {"exists": all(path_in(self.root, robot[key], exists=False).is_file() for key in ("urdf", "controllers") if robot[key]), "error": None}
            except (ConfigError, OSError, ET.ParseError, ValueError) as error:
                item["availability"] = {"exists": False, "error": str(error)}
        return {"resources": items, "task_template": task_template(self.root),
                "policy_details": details, "models": model_inventory(self.root, catalog=catalog, details=details), "joint_limits": limits,
                "capabilities": {"video_recording": bool(shutil.which("ffmpeg")), "renderer": "gazebo-camera-jpeg", "web_renderer": True, "render_frame_wait": True, "replay": False,
                    "policy_pd": True, "policy_reference": True, "robot_reset": True, "single_step": False, "hot_parameters": False, "controller_reload": True, "recording_options": True, "markers": True,
                    "recording_format": "csv", "recording_sample_hz": 10, "csv_analysis": True,
                    "task_output_limit": task_output_limit(self.root), "task_output_limit_options": list(TASK_OUTPUT_LIMIT_OPTIONS),
                    "watchdog_s": .6, "velocity_limits": [2, 1.5, 3]},
                "worlds": [file.relative_to(self.root).as_posix() for file in sorted((self.root / "rlsim/interfaces/gazebo/worlds").glob("*.world"))],
                "controls": [{"path": item["manifest"], "data": item["data"]} for item in items if item["kind"] == "control"],
                "experiments": [{"path": item["manifest"], "data": item["data"]} for item in items if item["kind"] == "experiment"]}

    def output_settings(self, payload=None):
        if payload is not None:
            keys(payload, {"task_output_limit"}, context="output settings")
            if self.closed:
                raise ConfigError("The service is closing")
            value = set_task_output_limit(self.root, payload["task_output_limit"])
        else:
            value = task_output_limit(self.root)
        return {"task_output_limit": value, "options": list(TASK_OUTPUT_LIMIT_OPTIONS), "applies_on": "next_task"}

    def compile(self, payload, write=False):
        if self.closed:
            raise ConfigError("The service is closing")
        keys(payload, {"experiment"}, {"control", "backend", "policy_pd"}, "experiment request")
        if not isinstance(payload["experiment"], dict):
            raise ConfigError("experiment must be a configuration object")
        args = dict(policy_run=self.policy_run, control_override=payload.get("control"), backend_override=payload.get("backend"), policy_pd_override=payload.get("policy_pd"))
        if write:
            directory = prepare(self.root, payload["experiment"], xacro=self.xacro, runtime_root=self.scratch.name, **args)
            with self.lock, output_guard(self.root):
                archive = self.archive(directory.name)
                try:
                    archive.mkdir(parents=True, exist_ok=False)
                    shutil.copyfile(directory / "resolved_config.json", archive / "resolved_config.json")
                    self.runtime_paths[directory.name] = directory
                except Exception:
                    shutil.rmtree(directory)
                    raise
                self.audit(directory, "prepared_from_web", overrides={key: payload[key] for key in ("control", "backend", "policy_pd") if key in payload})
                result = {"run_id": directory.name, "state": "prepared"}
                try:
                    protected = {key for key, session in self.sessions.items() if session["thread"].is_alive()}
                    removed = prune_task_outputs(self.root, protected | {directory.name})
                    for run_id in removed:
                        self.release_runtime(run_id)
                        self.sessions.pop(run_id, None)
                        self.terminal_metadata.pop(run_id, None)
                        self.terrain_cache.pop(run_id, None)
                        self.event_buffer.pop(run_id, None)
                    result["removed_tasks"] = removed
                except OSError as error:
                    result["cleanup_error"] = "当前任务已生成，但旧任务清理失败：" + str(error)
                return result
        resolved, _, terrain = resolve(self.root, payload["experiment"], **args)
        return {"resolved": resolved, "terrain": terrain}

    def save_experiment(self, payload):
        # Compatibility endpoint: generate only a per-task snapshot; configs are read-only.
        return self.compile(payload, write=True)

    def list_runs(self):
        result = []
        for file in sorted((self.root / "tools/runs").glob("*/resolved_config.json"), reverse=True):
            try:
                directory = self.directory(file.parent.name)
                metadata = self.metadata(directory.name, directory)
                with self.lock:
                    session = self.sessions.get(directory.name)
                    owned = bool(session and session["thread"].is_alive() and not session["stop"].is_set())
                result.append({"run_id": directory.name, "state": metadata["state"], "created_at": metadata.get("created_at"), "owned": owned})
            except (ConfigError, OSError, ValueError, KeyError):
                continue
        return result

    def audit(self, directory, action, **fields):
        event = {"wall_time": time.time(), "action": action, **fields}
        with self.lock:
            self.event_buffer[directory.name].append(event)
            if action == "prepared_from_web":
                with (self.archive(directory.name) / "parameters.log").open("a", encoding="utf-8") as stream:
                    stream.write(time.strftime("%Y-%m-%d %H:%M:%S") + " [INFO] 已生成当前任务参数快照：resolved_config.json；参数修改：" + json.dumps(fields.get("overrides", {}), ensure_ascii=False, separators=(",", ":")) + "\n")
            elif action == "marker":
                session = self.sessions.get(directory.name)
                recorder = session.get("recorder") if session else None
                if recorder and recorder.status()["state"] == "recording":
                    with (self.archive(directory.name) / "recording.log").open("a", encoding="utf-8") as stream:
                        stream.write(time.strftime("%Y-%m-%d %H:%M:%S") + " [INFO] " + json.dumps(fields, ensure_ascii=False) + "\n")

    def start(self, run_id):
        with self.lock, output_guard(self.root):
            directory = self.directory(run_id)
            if not (directory / "run.json").is_file():
                raise ConfigError("This is a parameter/recording archive; generate a new task to start")
            _, resolved = verify_run(directory)
            if Path(resolved["workspace_root"]).resolve() != self.root:
                raise ConfigError("Run belongs to another workspace")
            if self.closed or any(s["thread"].is_alive() for s in self.sessions.values()):
                raise ConfigError("A managed session is already active or the service is closing")
            lease = output_lease(self.root, run_id)
            if lease is None:
                raise ConfigError("Task output is being cleaned up; refresh the task list")
            session = {"stop": threading.Event(), "bridge": None, "latest": None, "received_at": None,
                       "error": None, "recorder": None, "video": None, "video_lock": threading.RLock(), "camera": None, "render": None, "io_lock": threading.RLock(), "requests": {}, "output_lease": lease}
            session["thread"] = threading.Thread(target=self._run, args=(run_id, directory, resolved, session), daemon=False)
            self.sessions[run_id] = session
            self.audit(directory, "start_requested")
            try:
                session["thread"].start()
            except Exception:
                lease.close()
                self.sessions.pop(run_id, None)
                raise
        return {"accepted": True}

    def _read_bridge(self, directory, process, session):
        while True:
            line = process.stdout.readline(1024 * 1024)
            if not line:
                break
            try:
                packet = json.loads(line)
                if packet.get("type") == "render_status":
                    with self.frame_changed:
                        session["render"] = {**packet,"received_at":time.time()}
                        self.frame_changed.notify_all()
                    continue
                if packet.get("type") == "command_result":
                    with self.lock:
                        pending = session["requests"].get(packet.get("request_id"))
                        if pending:
                            pending["result"] = packet
                            pending["event"].set()
                    continue
                if packet.get("type") != "telemetry" or packet.get("schema_version") != 1 or not isinstance(packet.get("signals"), dict):
                    continue
                json.dumps(packet, allow_nan=False)
                received = time.time()
                packet["service_received_at"] = received
                with self.lock:
                    session["latest"], session["received_at"] = packet, received
                with session["io_lock"]:
                    if session["recorder"]:
                        session["recorder"].feed(packet)
            except (ValueError, TypeError, AttributeError):
                continue
        with self.lock:
            self.frame_changed.notify_all()
            for pending in session["requests"].values():
                pending["result"] = {"success": False, "message": "ROS bridge disconnected; robot reset state unknown"}
                pending["event"].set()

    def _run(self, run_id, directory, resolved, session):
        def monitor():
            # Wait for launch_run to own the master before connecting the sidecar.
            while not session["stop"].wait(.1):
                state = read_json(directory / "run.json")["state"]
                if state == "running":
                    break
                if state in ("failed", "finished"):
                    return
            if session["stop"].is_set():
                return
            try:
                env = ros_environment(self.root, resolved["robot"])
                env["ROS_MASTER_URI"] = read_json(directory / "run.json")["ros_master_uri"]
                quiet_ros_environment(env, Path(self.scratch.name) / ("bridge-" + run_id))
                with open(os.devnull, "w") as log:
                    with session["io_lock"]:
                        if session["stop"].is_set():
                            return
                        process = subprocess.Popen([str(self.ros_python), str(self.worker_script), str(directory)], env=env,
                                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True, bufsize=1, start_new_session=True)
                        session["bridge"] = process
                    self._read_bridge(directory, process, session)
                    if not session["stop"].is_set():
                        session["error"] = "ROS bridge exited"
            except Exception as error:
                session["error"] = "ROS bridge: " + str(error)
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        try:
            self.launcher(directory, stop_event=session["stop"])
        except Exception as error:
            session["error"] = str(error)
            self.audit(directory, "launch_error", error=str(error))
        finally:
            session["stop"].set()
            with self.frame_changed:self.frame_changed.notify_all()
            with session["video_lock"]:
                if session.get("video"):
                    try: session["video"].stop()
                    except Exception as error: session["error"] = "Video cleanup: " + str(error)
            with session["io_lock"]:
                for cleanup in ((session["recorder"].stop if session["recorder"] else lambda: None), lambda: stop_child(session["bridge"])):
                    try:
                        cleanup()
                    except Exception as error:
                        session["error"] = "Cleanup: " + str(error)
            monitor_thread.join(timeout=5)
            process = session["bridge"]
            if process is not None:
                process.stdin.close()
                process.stdout.close()
            self.audit(directory, "session_released")
            try:
                self.release_runtime(run_id)
            finally:
                session["output_lease"].close()

    def release_runtime(self, run_id):
        # All children and their readers must be stopped before deleting scratch.
        with self.lock:
            directory = self.runtime_paths.pop(run_id, None)
            if directory:
                self.terminal_metadata[run_id] = read_json(directory / "run.json")
                self.terrain_cache[run_id] = read_json(directory / "terrain.json")
                shutil.rmtree(directory)
            bridge_home = Path(self.scratch.name) / ("bridge-" + run_id)
            if bridge_home.exists():
                shutil.rmtree(bridge_home)

    def owned(self, run_id):
        self.directory(run_id)
        with self.lock:
            session = self.sessions.get(run_id)
            if session is None or not session["thread"].is_alive() or session["stop"].is_set():
                raise ConfigError("This service does not own an active session")
            return session

    def stop(self, run_id):
        session = self.owned(run_id)
        session["stop"].set()
        with self.frame_changed:self.frame_changed.notify_all()
        with session["io_lock"]:
            if session["recorder"]:
                session["recorder"].stop()
        self.audit(self.directory(run_id), "stop_requested")
        return {"accepted": True}

    def command(self, run_id, payload):
        keys(payload, {"type", "value"}, context="command")
        if payload["type"] == "control":
            if payload["value"] not in COMMANDS:
                raise ConfigError("Unknown control command")
        elif payload["type"] == "velocity":
            values = payload["value"]
            if not isinstance(values, list) or len(values) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > limit for v, limit in zip(values, (2, 1.5, 3))):
                raise ConfigError("Velocity limits: |vx|<=2, |vy|<=1.5, |yaw|<=3")
        elif payload["type"] == "reset_robot":
            try: validate_reset_pose(payload["value"])
            except ValueError as error: raise ConfigError(str(error)) from error
        else:
            raise ConfigError("Unknown command type")
        session = self.owned(run_id)
        request = None
        request_id = None
        with session["io_lock"]:
            if session["stop"].is_set():
                raise ConfigError("Session is stopping")
            if session["requests"] and payload["type"] == "reset_robot":
                raise ConfigError("Model pose reset in progress; wait before another pose reset")
            process = session["bridge"]
            packet = session["latest"] or {}
            controller = packet.get("signals", {}).get("controller", {})
            if not process or process.poll() is not None or time.time() - controller.get("received_at", 0) > 2 or time.time() - (session["received_at"] or 0) > 2:
                raise ConfigError("Controller telemetry is not live")
            outgoing = payload
            running = controller.get("data", {}).get("controller_running", True)
            ack = session.get("controller_state", {})
            if ack.get("received_at", 0) > controller.get("received_at", 0): running = ack["running"]
            if (not running or session.get("controller_unknown")) and (
                    payload["type"] == "control" or payload["type"] == "velocity" and any(payload["value"])):
                raise ConfigError("控制器已停止，不接收运动或状态机指令")
            if payload["type"] == "reset_robot":
                if controller.get("data", {}).get("reset_robot_mode") != "model_pose":
                    raise ConfigError("Current controller has not connected robot reset in model-pose-only mode; rebuild/restart the adapter")
                request_id = uuid.uuid4().hex
                request = {"event": threading.Event(), "result": None}
                with self.lock: session["requests"][request_id] = request
                outgoing = {**payload, "request_id": request_id}
            try:
                process.stdin.write(json.dumps(outgoing, allow_nan=False) + "\n"); process.stdin.flush()
            except (BrokenPipeError, OSError) as error:
                if request_id:
                    with self.lock: session["requests"].pop(request_id, None)
                raise ConfigError("ROS bridge disconnected") from error
        if request:
            try:
                if not request["event"].wait(12):
                    raise ConfigError("Robot reset response timed out; state unknown, inspect telemetry")
                result = request["result"] or {}
                if result.get("success") is not True:
                    raise ConfigError(result.get("message") or "Robot reset failed")
                self.audit(self.directory(run_id), "robot_reset", pose=payload["value"], reset_count=result.get("reset_count"))
                return {"publication": "completed", "transition_acknowledged": True,
                        "reset_count": result.get("reset_count"), "message": result.get("message")}
            finally:
                with self.lock: session["requests"].pop(request_id, None)
        if payload["type"] == "control":
            self.audit(self.directory(run_id), "command_queued", value=payload["value"])
        return {"publication": "queued", "transition_acknowledged": False}

    def view(self, run_id, payload):
        try: value=validate_camera(payload)
        except ValueError as error: raise ConfigError(str(error)) from error
        session=self.owned(run_id)
        recording = bool(session.get("video") and session["video"].status()["state"] == "recording")
        with self.lock:
            if value["enabled"]:
                session["camera"] = copy.deepcopy(value)
            if recording:
                value = {**(session.get("camera") or value), "enabled": True}
        with session["io_lock"]:
            process=session["bridge"]
            if session["stop"].is_set() or not process or process.poll() is not None:
                raise ConfigError("Render bridge is not connected")
            try:
                process.stdin.write(json.dumps({"type":"view","value":value},allow_nan=False)+"\n");process.stdin.flush()
            except (BrokenPipeError,OSError) as error:raise ConfigError("Render bridge disconnected") from error
        return {"accepted":True,"state":"requested"}

    def view_frame(self, run_id, metadata=False, after=None, wait_ms=0):
        if after is not None and (type(after) is not int or not -1<=after<=2**53-1):
            raise ConfigError("Invalid frame sequence")
        if type(wait_ms) is not int or not 0<=wait_ms<=1000:
            raise ConfigError("Frame wait must be 0..1000 ms")
        session=self.owned(run_id)
        deadline=time.monotonic()+wait_ms/1000
        with self.frame_changed:
            while True:
                if session["stop"].is_set() or not session["thread"].is_alive():
                    raise ConfigError("This service does not own an active session")
                render=dict(session.get("render") or {})
                if after is None or render.get("sequence",0)>after:break
                if render.get("state")=="failed":raise ConfigError(render.get("error") or "Render bridge failed")
                remaining=deadline-time.monotonic()
                if remaining<=0:return None
                self.frame_changed.wait(min(remaining,.2))
        if session["bridge"] is None or session["bridge"].poll() is not None or time.time()-render.get("frame_received_at",0)>3:
            raise ConfigError("Render frame is unavailable or stale")
        file=path_in(self.directory(run_id),"view/frame.jpg")
        if file.stat().st_size>2*1024*1024:raise ConfigError("Render frame exceeds size limit")
        return (file,render) if metadata else file

    def video(self, run_id, payload):
        if set(payload) != {"action"} or payload["action"] not in ("start", "stop"):
            raise ConfigError("Expected video action start/stop")
        session = self.owned(run_id)
        with session["video_lock"]:
            if payload["action"] == "start":
                if session["stop"].is_set():
                    raise ConfigError("Session is stopping")
                directory = self.directory(run_id)
                def frame():
                    with self.lock:
                        render = dict(session.get("render") or {})
                        controller = (session.get("latest") or {}).get("signals", {}).get("controller", {})
                    paused = (time.time() - controller.get("received_at", 0) < 2 and
                              controller.get("data", {}).get("simulation_running") is False)
                    if render.get("state") == "failed" or (not paused and time.time() - render.get("frame_received_at", 0) > 3):
                        raise ConfigError("仿真画面不可用或已过期")
                    data = (directory / "view/frame.jpg").read_bytes()
                    if len(data) > 2 * 1024 * 1024 or not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
                        raise ConfigError("仿真图像格式无效")
                    return data
                def keepalive():
                    if not session["stop"].is_set():
                        config = session.get("camera") or camera_defaults([0, 0, .65])
                        self.view(run_id, {**config, "enabled": True})
                if session["video"] is None:
                    session["video"] = VideoRecorder(self.archive(run_id), frame, keepalive, output_directory=self.root / "videos")
                return session["video"].start()
            result = session["video"].stop() if session["video"] else {"state": "idle", "format": "mp4", "files": []}
            return result

    def recording(self, run_id, action, options=None):
        session = self.owned(run_id)
        with session["io_lock"]:
            if session["stop"].is_set():
                raise ConfigError("Session is stopping")
            if action == "start":
                operation = session.get("controller_lock")
                if operation and operation.locked():
                    raise ConfigError("控制器正在切换，请等待参数应用完成后开始录包")
                if read_json(self.directory(run_id) / "run.json")["state"] != "running":
                    raise ConfigError("Recording requires a running session")
                if time.time() - (session["received_at"] or 0) > 2:
                    raise ConfigError("ROS bridge is not connected")
                if session["recorder"] is None:
                    directory = self.directory(run_id)
                    session["recorder"] = self.recorder_factory(self.root, self.archive(run_id), read_json(directory / "resolved_config.json"))
                result = session["recorder"].start(**options) if options else session["recorder"].start()
            elif action == "stop":
                result = session["recorder"].stop() if session["recorder"] else {"state": "idle", "files": []}
            else:
                raise ConfigError("Unknown recording action")
        self.audit(self.directory(run_id), "recording_" + action)
        return result

    def events(self, run_id):
        self.directory(run_id)
        with self.lock:
            return list(self.event_buffer[run_id])

    def recordings(self):
        """List completed CSVs only, without selecting/changing a live session."""
        rows = []
        for run in self.list_runs():
            run_id = run["run_id"]
            try:
                status = self.status(run_id)
                for item in status["recording"].get("files", []):
                    name = item.get("name", "")
                    if not isinstance(name, str) or Path(name).name != name or not name.lower().endswith(".csv"):
                        continue
                    path = self.download(run_id, name)
                    rows.append({"run_id": run_id, "name": name, "bytes": path.stat().st_size,
                                 "rows": item.get("rows"), "modified_at": path.stat().st_mtime})
            except (ConfigError, OSError, ValueError, KeyError, TypeError, AttributeError):
                continue
        return sorted(rows, key=lambda item: item["modified_at"], reverse=True)

    def logs(self, run_id):
        self.directory(run_id)
        directory = self.archive(run_id)
        return {"parameters": tail(directory / "parameters.log"), "recording": tail(directory / "recording.log")}

    def marker(self, run_id, payload):
        keys(payload, {"label"}, context="marker")
        if not isinstance(payload["label"], str) or not 1 <= len(payload["label"].strip()) <= 120:
            raise ConfigError("Marker label must contain 1..120 characters")
        directory = self.directory(run_id)
        with self.lock:
            session = self.sessions.get(run_id)
            clock = (session.get("latest") or {}).get("signals", {}).get("clock", {}) if session else {}
            stamp = clock.get("data") if time.time() - clock.get("received_at", 0) < 2 else None
            self.audit(directory, "marker", label=payload["label"].strip(), simulation_time=stamp)
        return {"saved": True}

    def status(self, run_id, detail=False):
        with self.lock:
            directory = self.directory(run_id)
            metadata = self.metadata(run_id, directory)
            if detail:
                resolved = read_json(directory / "resolved_config.json")
                terrain = read_json(directory / "terrain.json") if (directory / "terrain.json").is_file() else self.terrain_cache.get(run_id, {})
            session = self.sessions.get(run_id)
            packet = copy.deepcopy(session["latest"]) if session else None
            active = bool(session and session["thread"].is_alive() and not session["stop"].is_set())
        archive = self.archive(run_id)
        recording = read_json(archive / "recording.json") if (archive / "recording.json").is_file() else {"state": "idle", "format": "csv", "files": []}
        if recording["state"] == "recording" and not active:
            recording = {**recording, "state": "unknown", "error": "录包写盘结果尚未确认"}
        if session:
            with session["io_lock"]:
                if session["recorder"]:
                    recording = session["recorder"].status()
        video = read_json(archive / "video.json") if (archive / "video.json").is_file() else {"state": "idle", "format": "mp4", "files": []}
        if session and session.get("video"):
            video = session["video"].status()
        elif video["state"] in ("recording", "finalizing"):
            video = {**video, "state": "unknown", "error": "视频写盘结果尚未确认"}
        result = {"run_id": run_id, "metadata": metadata, "owned": active, "telemetry": packet,
                  "controller": copy.deepcopy(session.get("controller_state")) if active else None,
                  "controller_unknown": bool(session and session.get("controller_unknown")),
                  "bridge_live": bool(active and session["bridge"] and session["bridge"].poll() is None and time.time() - (session["received_at"] or 0) < 2),
                  "render": copy.deepcopy(session.get("render")) if active else None,
                  "video": video, "recording": recording, "error": session["error"] if session else None}
        result["files"] = sorted(name for name in DOWNLOADS if (archive / name).is_file()) + [item["name"] for item in recording.get("files", [])] + [item["name"] for item in video.get("files", [])]
        if detail:
            result.update(resolved=resolved, terrain=terrain)
            topics, buffer = recording_topics(self.root, result["resolved"])
            result["recording_profile"] = {"topics": topics, "buffer_mb": buffer, "directory": str(archive), "format": "csv", "sample_hz": 10}
        return result

    def download(self, run_id, name):
        self.directory(run_id)
        directory = self.archive(run_id)
        allowed = set(DOWNLOADS)
        status = self.status(run_id)
        allowed.update(item["name"] for item in status["recording"].get("files", []))
        allowed.update(item["name"] for item in status["video"].get("files", []))
        if name not in allowed or Path(name).name != name:
            raise ConfigError("File is not available for download")
        if any(item.get("name") == name and item.get("storage") == "project" for item in status["video"].get("files", [])):
            return self.video_download(name)
        file = path_in(directory, name)
        if file.parent != directory or not file.is_file():
            raise ConfigError("Invalid download path")
        return file

    def video_download(self, name):
        # Project videos remain downloadable after their task archive is pruned.
        if Path(name).name != name or name.startswith(".") or not name.endswith(".mp4"):
            raise ConfigError("Invalid video filename")
        directory = (self.root / "videos").resolve()
        file = path_in(directory, name)
        if file.parent != directory or not file.is_file():
            raise ConfigError("Video is not available")
        return file

    def close(self):
        with self.lock:
            self.closed = True
            sessions = list(self.sessions.values())
            for session in sessions:
                session["stop"].set()
            self.frame_changed.notify_all()
        for session in sessions:
            session["thread"].join(timeout=30)
            if session["thread"].is_alive():
                raise ConfigError("Session shutdown has not completed")
        for run_id in list(self.runtime_paths):
            self.release_runtime(run_id)
        self.scratch.cleanup()
