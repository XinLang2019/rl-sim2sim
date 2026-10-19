"""Reload only a stopped policy runtime; the Gazebo session remains owned."""
import copy
import json
import threading
import time
import uuid

from rlsim_support.catalog import ConfigError, keys
from rlsim_support.experiment import resolve
from rlsim_support.yaml_format import dumps


def compatible(current, candidate):
    """Only policy and robot control settings may differ inside a live world."""
    for field in ("robot", "backend", "terrain"):
        if current[field] != candidate[field]:
            raise ConfigError("机器人、World 或物理设置已改变，需要结束仿真并生成新任务")
    old, new = copy.deepcopy(current["experiment"]), copy.deepcopy(candidate["experiment"])
    for obj in (old, new):
        for name in ("policy", "model_file", "control", "overrides"):
            obj.pop(name, None)
    if old != new:
        raise ConfigError("运行中只能修改策略和控制参数；初始位姿请使用模型位姿复位")
    for field in ("joint_names", "controller_names"):
        if current["robot_runtime"][field] != candidate["robot_runtime"][field]:
            raise ConfigError("物理关节或执行器映射已改变，需要重新启动仿真")
    physical = {current["robot"].get("xacro"), current["experiment"].get("world")}
    for path, digest in current.get("fingerprints", {}).items():
        if (path in physical or path.startswith("robots/") or "/worlds/" in path) and candidate.get("fingerprints", {}).get(path) != digest:
            raise ConfigError("机器人资产或场景文件内容已改变，需要重新启动仿真：" + path)


class ControllerLifecycle:
    def _controller_rpc(self, run_id, session, values):
        request_id = uuid.uuid4().hex
        pending = {"event": threading.Event(), "result": None}
        with session["io_lock"]:
            process = session["bridge"]
            if session["stop"].is_set() or not process or process.poll() is not None:
                raise ConfigError("控制器桥接未连接")
            with self.lock:
                session["requests"][request_id] = pending
            try:
                process.stdin.write(json.dumps({"type": "controller", "value": values, "request_id": request_id}) + "\n")
                process.stdin.flush()
            except OSError:
                with self.lock: session["requests"].pop(request_id, None)
                raise ConfigError("控制器请求未发送，桥接已断开")
        try:
            if not pending["event"].wait(60):
                session["controller_unknown"] = True
                raise ConfigError("控制器响应超时，应用状态未知；请结束当前任务后重建，不自动恢复控制")
            result = pending["result"] or {}
            if "running" not in result: session["controller_unknown"] = True
            if "running" in result:
                session["controller_state"] = {"running": result["running"], "revision": result["revision"], "received_at": time.time()}
            if result.get("success") is not True:
                raise ConfigError(result.get("message") or "控制器操作失败")
            return result
        finally:
            with self.lock: session["requests"].pop(request_id, None)

    def controller(self, run_id, payload):
        keys(payload, {"action"}, {"configuration"}, "controller lifecycle")
        action = payload["action"]
        if action not in ("stop", "start", "check", "apply") or (action in ("check", "apply")) != ("configuration" in payload):
            raise ConfigError("Expected stop/start or check/apply with configuration")
        session = self.owned(run_id)
        with self.lock:
            lock = session.setdefault("controller_lock", threading.Lock())
        if not lock.acquire(blocking=False):
            raise ConfigError("控制器操作进行中，请等待完成")
        try:
            status = self.status(run_id, detail=True)
            entry = (status.get("telemetry") or {}).get("signals", {}).get("controller", {})
            data = entry.get("data", {})
            if not status["bridge_live"] or time.time() - entry.get("received_at", 0) > 2:
                raise ConfigError("控制器遥测未就绪")
            if not data.get("controller_lifecycle_supported"):
                raise ConfigError("当前仿真节点未支持控制器重载，请构建新版本后重新启动一次任务")
            if session.get("controller_unknown"):
                raise ConfigError("控制器应用状态未知，请结束任务后重建")
            running = data.get("controller_running", True)
            ack = session.get("controller_state", {})
            if ack.get("received_at", 0) > entry.get("received_at", 0): running = ack["running"]
            if action in ("stop", "start"):
                result = self._controller_rpc(run_id, session, {"action": action})
                self.audit(self.directory(run_id), "controller_" + action)
                return result
            if running:
                raise ConfigError("请先停止控制器，再校验或应用策略与参数")
            configuration = payload["configuration"]
            checked = self.compile(configuration, write=False)
            current, candidate = status["resolved"], checked["resolved"]
            compatible(current, candidate)
            candidate["run_id"] = run_id
            if action == "check": return checked
            # Avoid changing the column meaning of a CSV already being written.
            recorder = session.get("recorder")
            if recorder and recorder.status()["state"] == "recording" and (
                    current["policy"]["joints"] != candidate["policy"]["joints"] or
                    current["policy"]["actions"]["count"] != candidate["policy"]["actions"]["count"]):
                raise ConfigError("新策略的数据列不同，请先停止 CSV 录包并写盘，再应用；仿真无需重启")
            directory = self.directory(run_id)
            staged = directory / ("controller-" + uuid.uuid4().hex)
            staged.mkdir()
            (staged / "policy.yaml").write_text(dumps(candidate["policy"]))
            (staged / "robot_runtime.yaml").write_text(dumps(candidate["robot_runtime"]))
            # Prewrite snapshots so a filesystem failure cannot occur before RPC
            # validation yet leave a claimed successful application behind.
            snapshot = json.dumps(candidate, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            runtime_pending = directory / "resolved_config.json.next"
            archive_pending = self.archive(run_id) / "resolved_config.json.next"
            runtime_pending.write_text(snapshot); archive_pending.write_text(snapshot)
            result = self._controller_rpc(run_id, session, {"action": "reload", "policy_manifest": str(staged / "policy.yaml"),
                                                          "robot_runtime_manifest": str(staged / "robot_runtime.yaml")})
            try:
                runtime_pending.replace(directory / "resolved_config.json")
                if archive_pending != runtime_pending:
                    archive_pending.replace(self.archive(run_id) / "resolved_config.json")
            except OSError as error:
                session["controller_unknown"] = True
                raise ConfigError("控制器已重载，但参数快照写入失败；请结束当前任务后重建：" + str(error)) from error
            # Retain only the loaded manifests. Pending/uncertain manifests must
            # stay available until the session's ROS children have exited.
            previous = session.get("controller_files")
            session["controller_files"] = staged
            if previous:
                import shutil
                shutil.rmtree(previous, ignore_errors=True)
            with session["io_lock"]:
                if recorder:
                    recorder.resolved = candidate
                    recorder.names = list(candidate["policy"]["joints"])
                    recorder.action_count = candidate["policy"]["actions"]["count"]
            with (self.archive(run_id) / "parameters.log").open("a", encoding="utf-8") as log:
                log.write(time.strftime("%Y-%m-%d %H:%M:%S") + " [INFO] 控制器重载参数：" + json.dumps(configuration, ensure_ascii=False, separators=(",", ":")) + "\n")
            self.audit(directory, "controller_parameters_applied", revision=result["revision"])
            return {**result, "run_id": run_id, "resolved": candidate, "terrain": checked["terrain"]}
        finally:
            if 'runtime_pending' in locals() and runtime_pending.exists(): runtime_pending.unlink()
            if 'archive_pending' in locals() and archive_pending.exists(): archive_pending.unlink()
            lock.release()
