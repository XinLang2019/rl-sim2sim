"""Bounded, explicit CSV recording of the robot telemetry already in the bridge.

One row per bridge sample (currently 10 Hz), independently of HTTP/chart polling.
Missing/stale values stay empty. No images, world models or automatic telemetry
archive; the subscriber filters model_states to the selected robot beforehand.
"""
import copy
import csv
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
import uuid

from rlsim_support.catalog import ConfigError


JOINT_FIELDS = (("q", "rad"), ("dq", "rad_s"), ("effort", "Nm"),
                ("target_q", "rad"), ("target_dq", "rad_s"), ("kp", "Nm_rad"),
                ("kd", "Nm_s_rad"), ("tau_ff", "Nm"))
CONTROLLER_FIELDS = ("ready", "fsm_state", "policy_ready", "fault", "simulation_running",
                     "policy_steps", "reset_count", "controller_running", "controller_revision", "inference_ms", "reset_robot_supported", "reset_robot_mode")


def number(value):
    if type(value) in (int, float) and math.isfinite(value):
        return format(value, ".10g")
    return ""


class CsvRecorder:
    def __init__(self, root, directory, resolved):
        from .processes import recording_topics
        self.directory, self.resolved = Path(directory), resolved
        self.directory.mkdir(parents=True, exist_ok=True)
        self.topics, self.buffer = recording_topics(root, resolved)
        self.names = list(resolved.get("policy", {}).get("joints") or resolved.get("robot_runtime", {}).get("joint_names", []))
        self.action_count = resolved.get("policy", {}).get("actions", {}).get("count", len(self.names))
        self.metadata = {"state": "idle", "format": "csv", "files": []}
        self.thread = None
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.stopping = threading.Event()
        self.error = None
        self.pending = queue.Queue(maxsize=256)
        self.pending_bytes = 0

    def _save(self):
        temporary = self.directory / "recording.json.tmp"
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(self.directory / "recording.json")

    def _log(self, text):
        with (self.directory / "recording.log").open("a", encoding="utf-8") as stream:
            stream.write(time.strftime("%Y-%m-%d %H:%M:%S") + " [INFO] " + text + "\n")

    def start(self, name=None, topics=None):
        with self.operation_lock, self.lock:
            if self.thread and self.thread.is_alive():
                raise ConfigError("Recording is already active")
            if name is not None and (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name)):
                raise ConfigError("Recording name must be 1..64 ASCII letters, digits, _ or -")
            chosen = self.topics if topics is None else topics
            if not isinstance(chosen, list) or not chosen or any(not isinstance(t, str) or t not in self.topics for t in chosen) or len(set(chosen)) != len(chosen):
                raise ConfigError("Recording topics must be a nonempty subset of configured topics")
            self.chosen = set(chosen)
            filename = (name or "robot-data") + "-" + uuid.uuid4().hex[:12] + ".csv"
            self.metadata = {"state": "recording", "format": "csv", "started_at": time.time(),
                             "current_file": filename, "topics": list(chosen), "sample_hz": 10,
                             "rows": 0, "bytes": 0, "files": list(self.metadata.get("files", []))}
            self.pending = queue.Queue(maxsize=256)
            self.pending_bytes = 0
            self.stopping.clear(); self.error = None
            self.previous_time = self.previous_reset = self.previous_received = self.previous_revision = None
            self.segment = 0
            self.columns = ["simulation_time_s", "received_time_unix_s", "message_time_s", "segment"]
            self.columns += [name + "_received_time_unix_s" for name in ("controller", "model", "joints", "command")]
            self.columns += ["controller." + name for name in CONTROLLER_FIELDS]
            for field, labels, unit in (("position", "xyz", "m"), ("quaternion", "wxyz", ""),
                                        ("linear_velocity", "xyz", "m_s"), ("angular_velocity", "xyz", "rad_s")):
                self.columns += ["body." + field + "." + axis + ("_" + unit if unit else "") for axis in labels]
            self.columns += ["command.vx_m_s", "command.vy_m_s", "command.wz_rad_s"]
            self.columns += ["joint." + name + "." + key + "_" + unit for name in self.names for key, unit in JOINT_FIELDS]
            self.columns += ["action" + str(i) for i in range(self.action_count)]
            # Keep the joint-state subscriber data as well as policy-order controller data.
            self.columns += ["joint_state." + name + "." + key + "_" + unit for name in self.names for key, unit in JOINT_FIELDS[:3]]
            self.stream = None
            try:
                self.stream = (self.directory / filename).open("x", encoding="utf-8-sig", newline="")
                self.writer = csv.writer(self.stream)
                self.writer.writerow(self.columns); self.stream.flush()
                self._save(); self._log("开始 CSV 录包：" + filename)
            except OSError as error:
                if self.stream:
                    self.stream.close()
                self.metadata.update(state="failed", error=str(error), ended_at=time.time())
                raise ConfigError("CSV recording could not start: " + str(error)) from error
            self.thread = threading.Thread(target=self._write, name="robot-csv-writer", daemon=False)
            self.thread.start()
            return self.status()

    def feed(self, packet):
        # Never let slow disk IO block the live bridge or command watchdog.
        with self.lock:
            if self.metadata["state"] != "recording" or self.stopping.is_set():
                return
            try:
                size = len(json.dumps(packet, allow_nan=False).encode("utf-8"))
                if self.pending_bytes + size > self.buffer * 1024 * 1024:
                    raise queue.Full
                self.pending.put_nowait((copy.deepcopy(packet), size))
                self.pending_bytes += size
            except queue.Full:
                self.error = "CSV 写入缓冲已满；录制失败，请检查磁盘，未静默丢弃数据。"
                self.stopping.set()

    def _row(self, packet):
        received = packet.get("service_received_at", packet.get("wall_time", time.time()))
        namespace = self.resolved["robot"]["namespace"]
        topics = {"clock": "/clock", "model": "/gazebo/model_states", "controller": namespace + "telemetry",
                  "joints": namespace + "joint_states", "command": "/cmd_vel"}
        signals = {name: entry for name, entry in packet.get("signals", {}).items()
                   if topics.get(name) in self.chosen and isinstance(entry, dict)
                   and 0 <= received - entry.get("received_at", 0) < 2}
        data = lambda name: signals.get(name, {}).get("data") or {}
        c, model, joints, command = (data(name) for name in ("controller", "model", "joints", "command"))
        stamp = signals.get("clock", {}).get("data")
        reset = c.get("reset_count")
        revision = c.get("controller_revision")
        if ((stamp is not None and self.previous_time is not None and stamp < self.previous_time)
                or (reset is not None and self.previous_reset is not None and reset != self.previous_reset)
                or (revision is not None and self.previous_revision is not None and revision != self.previous_revision)
                or (self.previous_received is not None and received - self.previous_received > 1.25)):
            self.segment += 1
        self.previous_time, self.previous_reset, self.previous_received, self.previous_revision = stamp, reset, received, revision
        row = [number(stamp), number(received), number(signals.get("joints", {}).get("message_time")), self.segment]
        row += [number(signals.get(name, {}).get("received_at")) for name in ("controller", "model", "joints", "command")]
        row += [c.get(key, "") if c.get(key) is not None else "" for key in CONTROLLER_FIELDS]
        def values(source, key, count):
            v = source.get(key, [])
            return [number(v[i]) if i < len(v) else "" for i in range(count)]
        for field, count in (("position", 3), ("quaternion", 4), ("linear_velocity", 3), ("angular_velocity", 3)):
            row += values(model, field, count)
        row += [number(command.get(key)) for key in ("vx", "vy", "wz")]
        def joint_values(source, name, field, names_key):
            names = source.get(names_key, [])
            if name not in names:
                return ""
            index, array = names.index(name), source.get(field, [])
            return number(array[index]) if index < len(array) else ""
        row += [joint_values(c, name, key, "joint_names") for name in self.names for key, _ in JOINT_FIELDS]
        row += values(c, "actions", self.action_count)
        row += [joint_values(joints, name, key, "names") for name in self.names for key, _ in JOINT_FIELDS[:3]]
        return row

    def _write(self):
        last_flush = time.monotonic()
        try:
            while not self.stopping.is_set() or not self.pending.empty():
                try:
                    packet, size = self.pending.get(timeout=.1)
                except queue.Empty:
                    continue
                with self.lock:
                    self.pending_bytes -= size
                self.writer.writerow(self._row(packet))
                with self.lock:
                    self.metadata["rows"] += 1
                if time.monotonic() - last_flush >= 1:
                    self.stream.flush(); last_flush = time.monotonic()
            self.stream.flush(); os.fsync(self.stream.fileno())
        except Exception as error:
            self.error = str(error)
        finally:
            try:
                self.stream.close()
            except OSError as error:
                self.error = str(error)
            with self.lock:
                path = self.directory / self.metadata["current_file"]
                size = path.stat().st_size if path.exists() else 0
                self.metadata.update(state="failed" if self.error else "finished", ended_at=time.time(), bytes=size)
                if self.error:
                    self.metadata["error"] = self.error
                else:
                    self.metadata["files"].append({"name": path.name, "bytes": size, "rows": self.metadata["rows"]})
                try:
                    self._save(); self._log(("录包失败：" + self.error) if self.error else "完成 CSV 写盘：{}，{} 行，{} 字节".format(path.name, self.metadata["rows"], size))
                except OSError as error:
                    self.error = "CSV 录包索引或日志写入失败：" + str(error)
                    self.metadata.update(state="failed", error=self.error)

    def status(self):
        with self.lock:
            result = copy.deepcopy(self.metadata)
            if result.get("current_file"):
                path = self.directory / result["current_file"]
                result["bytes"] = path.stat().st_size if path.is_file() else 0
            return result

    def stop(self):
        with self.operation_lock:
            self.stopping.set()
            if self.thread:
                self.thread.join(timeout=30)
                if self.thread.is_alive():
                    raise ConfigError("CSV is still flushing; recording completion is unknown")
            return self.status()
