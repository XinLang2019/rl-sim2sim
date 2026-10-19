"""Bounded, server-side MP4 capture of the existing Gazebo JPEG stream."""
import copy
import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import tempfile
import threading
import time
import uuid


class VideoRecorder:
    def __init__(self, directory, frame, keepalive, output_directory=None):
        self.directory = Path(directory)
        self.output_directory = Path(output_directory) if output_directory is not None else self.directory
        self.storage = "project" if output_directory is not None else "task"
        self.frame, self.keepalive = frame, keepalive
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.process = None
        file = self.directory / "video.json"
        self.info = json.loads(file.read_text()) if file.is_file() else {"state": "idle", "format": "mp4", "files": []}

    def save(self):
        tmp = self.directory / ".video.json.tmp"
        tmp.write_text(json.dumps(self.info, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        tmp.replace(self.directory / "video.json")

    def status(self):
        with self.lock:
            result = copy.deepcopy(self.info)
            if self.thread and self.thread.is_alive() and self.partial.is_file():
                result["bytes"] = self.partial.stat().st_size
            return result

    def start(self):
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError("仿真录屏已经启动或正在写盘")
            executable = shutil.which("ffmpeg")
            if not executable:
                raise ValueError("服务端未安装 FFmpeg，无法保存 MP4")
            first = self.frame()  # Reject unavailable/stale sources before creating files.
            name = "simulation-" + time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8] + ".mp4"
            self.output_directory.mkdir(parents=True, exist_ok=True)
            self.partial = self.output_directory / ("." + name + ".partial.mp4")
            self.output = self.output_directory / name
            self.stop_event.clear()
            self.info = {"state": "recording", "format": "mp4", "fps": 30, "started_at": time.time(),
                         "ended_at": None, "bytes": 0, "frames": 0, "name": name, "directory": str(self.output_directory.resolve()),
                         "files": self.info.get("files", []), "error": None}
            self.errors = tempfile.TemporaryFile()
            try:
                self.process = subprocess.Popen([
                    executable, "-hide_banner", "-loglevel", "error", "-nostdin", "-y",
                    "-use_wallclock_as_timestamps", "1", "-f", "mjpeg", "-i", "pipe:0",
                    "-an", "-c:v", "libx264", "-threads", "2", "-preset", "veryfast", "-crf", "23",
                    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-r", "30", "-vsync", "cfr",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.partial)],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.errors, bufsize=0)
                os.set_blocking(self.process.stdin.fileno(), False)
                self.save()
                self.thread = threading.Thread(target=self.run, args=(first,), daemon=False)
                self.thread.start()
            except Exception:
                if self.process:
                    self.process.kill(); self.process.communicate()
                self.errors.close(); self.partial.unlink(missing_ok=True)
                self.info.update(state="failed", error="无法启动视频编码器", ended_at=time.time()); self.save()
                raise
            return copy.deepcopy(self.info)

    def write(self, data):
        # A stuck encoder cannot block ROS IO or grow an unbounded frame queue.
        pending = memoryview(data)
        deadline = time.monotonic() + 2
        fd = self.process.stdin.fileno()
        while pending:
            if self.process.poll() is not None:
                raise RuntimeError("视频编码器提前退出")
            if time.monotonic() > deadline:
                raise RuntimeError("视频编码超时")
            if select.select([], [fd], [], .1)[1]:
                try:
                    pending = pending[os.write(fd, pending):]
                except BlockingIOError:
                    pass

    def run(self, first):
        error = None
        try:
            deadline, renew = time.monotonic(), 0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now >= renew:
                    self.keepalive(); renew = now + 1
                self.write(first if first is not None else self.frame()); first = None
                with self.lock:
                    self.info["frames"] += 1
                deadline = max(deadline + 1 / 30, time.monotonic())
                self.stop_event.wait(max(0, deadline - time.monotonic()))
        except Exception as exc:
            error = str(exc)
        finally:
            with self.lock:
                self.info["state"] = "finalizing"
                self.info["ended_at"] = time.time()
            try:
                self.process.stdin.close()
                code = self.process.wait(timeout=20)
                if code or not self.partial.is_file() or not self.partial.stat().st_size:
                    self.errors.seek(0)
                    raise RuntimeError("MP4 编码失败：" + self.errors.read(2000).decode("utf-8", "replace"))
                with self.partial.open("rb") as stream:
                    os.fsync(stream.fileno())
                with self.lock:
                    self.partial.replace(self.output)
                    size = self.output.stat().st_size
                    self.info["files"].append({"name": self.output.name, "bytes": size, "storage": self.storage})
                    self.info["bytes"] = size
            except Exception as exc:
                error = str(exc)
                if self.process.poll() is None:
                    self.process.kill(); self.process.wait(timeout=5)
                self.partial.unlink(missing_ok=True)
            finally:
                self.errors.close()
                with self.lock:
                    self.info.update(state="failed" if error else "finished", error=error)
                    self.save()

    def stop(self):
        with self.lock:
            thread = self.thread
            if thread and thread.is_alive():
                self.info["state"] = "finalizing"
                self.stop_event.set()
        if thread:
            thread.join(timeout=25)
            if thread.is_alive():
                if self.process and self.process.poll() is None:
                    self.process.kill()
                thread.join(timeout=5)
                if thread.is_alive():
                    raise RuntimeError("视频写盘尚未完成")
        return self.status()
