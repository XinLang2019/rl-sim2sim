"""Loopback-only HTTP API and static workbench, using Python's standard library."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import mimetypes
import os
from pathlib import Path
import secrets
import socket
from urllib.parse import unquote, urlsplit, parse_qs

from rlsim_support.catalog import ConfigError


class Server(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, manager):
        if address[0] not in ("127.0.0.1", "localhost", "::1"):
            raise ConfigError("The local workbench only binds to loopback")
        self.manager = manager
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, Handler)
        self.allowed_hosts = {"127.0.0.1:" + str(self.server_port), "localhost:" + str(self.server_port)}


class Handler(BaseHTTPRequestHandler):
    server_version = "RLsimWorkbench/1"
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        # Keep-alive avoids reconnecting per frame; disable Nagle's delayed tail.
        self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.connection.settimeout(10)
    def log_message(self, format, *args):
        pass

    def send(self, value, status=200):
        data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status); self.headers_for("application/json; charset=utf-8", len(data)); self.end_headers()
        self.wfile.write(data)

    def headers_for(self, content_type, length):
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")

    def do_GET(self):
        self.dispatch(False)

    def do_POST(self):
        self.dispatch(True)

    def dispatch(self, mutation):
        try:
            host = self.headers.get("Host", "")
            if host not in self.server.allowed_hosts:
                self.send({"error": "Invalid Host"}, 403); return
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + host:
                self.send({"error": "Cross-origin request rejected"}, 403); return
            path = unquote(urlsplit(self.path).path)
            parts = path.strip("/").split("/")
            if any(part in (".", "..") for part in parts):
                raise ConfigError("Invalid path")
            payload = None
            if mutation:
                if self.headers.get("X-RLsim-Token") != self.server.token:
                    self.send({"error": "Missing session token"}, 403); return
                if self.headers.get_content_type() != "application/json":
                    self.send({"error": "Use application/json"}, 415); return
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 256 * 1024:
                    self.send({"error": "Request size must be 1..262144 bytes"}, 413); return
                self.connection.settimeout(10)
                payload = json.loads(self.rfile.read(length), parse_constant=lambda v: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
                if not isinstance(payload, dict):
                    raise ConfigError("Expected an object")
            manager = self.server.manager
            if not mutation and path == "/api/bootstrap":
                self.send({"schema_version": 1, "token": self.server.token, **manager.catalog()}); return
            if not mutation and path == "/api/runs":
                self.send(manager.list_runs()); return
            if not mutation and path == "/api/recordings":
                self.send(manager.recordings()); return
            if mutation and path in ("/api/check", "/api/prepare", "/api/presets"):
                result = manager.save_experiment(payload) if path == "/api/presets" else manager.compile(payload, write=path == "/api/prepare")
                self.send(result); return
            if not mutation and len(parts) == 3 and parts[:2] == ["api", "videos"]:
                self.file(manager.video_download(parts[2]), attachment=True); return
            if len(parts) >= 3 and parts[:2] == ["api", "runs"]:
                run_id = parts[2]
                if not mutation and len(parts) == 3:
                    self.send(manager.status(run_id, detail=True)); return
                if not mutation and len(parts) == 4 and parts[3] == "status":
                    self.send(manager.status(run_id)); return
                if not mutation and len(parts) == 5 and parts[3:] == ["view","frame"]:
                    query=parse_qs(urlsplit(self.path).query,keep_blank_values=True)
                    after=int(query['after'][0]) if 'after' in query else None
                    wait_ms=int(query.get('wait_ms',['0'])[0])
                    frame=manager.view_frame(run_id,metadata=True,after=after,wait_ms=wait_ms)
                    if frame is None:
                        self.send_response(204);self.headers_for('image/jpeg',0);self.end_headers();return
                    file,render=frame
                    self.file(file,extra_headers={"X-Render-Sequence":str(render.get("sequence",0)),"X-Render-Received":str(render.get("frame_received_at",0)),"X-Render-Encode-Ms":str(render.get("encode_ms") or 0)}); return
                if not mutation and len(parts) == 4 and parts[3] == "logs":
                    self.send(manager.logs(run_id)); return
                if not mutation and len(parts) == 4 and parts[3] == "events":
                    self.send(manager.events(run_id)); return
                if not mutation and len(parts) == 5 and parts[3] == "files":
                    self.file(manager.download(run_id, parts[4]), attachment=True); return
                if mutation and len(parts) == 4:
                    action = parts[3]
                    if action in ("start", "stop"):
                        if payload: raise ConfigError("This action takes an empty object")
                        result = manager.start(run_id) if action == "start" else manager.stop(run_id)
                    elif action == "video": result = manager.video(run_id,payload)
                    elif action == "view": result = manager.view(run_id,payload)
                    elif action == "command": result = manager.command(run_id, payload)
                    elif action == "controller": result = manager.controller(run_id, payload)
                    elif action == "recording":
                        if "action" not in payload or set(payload) - {"action", "name", "topics"}: raise ConfigError("Expected recording action and optional name/topics")
                        if payload["action"] != "start" and set(payload) != {"action"}: raise ConfigError("Only recording start accepts options")
                        result = manager.recording(run_id, payload["action"], {k:v for k,v in payload.items() if k != "action"})
                    elif action == "marker": result = manager.marker(run_id, payload)
                    else: self.send({"error": "Not found"}, 404); return
                    self.send(result); return
            if not mutation and path in ("/", "/app.js", "/style.css"):
                self.file(manager.root / "tools/web/frontend" / ({"/": "index.html"}.get(path, path[1:]))); return
            self.send({"error": "Not found"}, 404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except FileExistsError as error:
            self.send({"error": str(error)}, 409)
        except FileNotFoundError as error:
            self.send({"error": str(error)}, 404)
        except (ConfigError, ValueError, TypeError, KeyError, OSError) as error:
            self.send({"error": str(error)}, 400)
        except Exception as error:
            self.send({"error": "Server error: " + str(error)}, 500)

    def file(self, path, attachment=False, extra_headers=None):
        # Download a fixed-length snapshot even if a log is concurrently appended.
        with Path(path).open("rb") as stream:
            length = os.fstat(stream.fileno()).st_size
            self.send_response(200)
            self.headers_for(mimetypes.guess_type(path)[0] or "application/octet-stream", length)
            for name,value in (extra_headers or {}).items():self.send_header(name,value)
            if attachment:
                self.send_header("Content-Disposition", 'attachment; filename="' + Path(path).name + '"')
            self.end_headers()
            remaining = length
            while remaining:
                data = stream.read(min(65536, remaining))
                if not data: break
                self.wfile.write(data); remaining -= len(data)
