"""Keep the latest task outputs, with cross-process protection for active runs."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

from .catalog import ConfigError, identifier, keys, path_in, simulation_config, validate_task_output_limit

TASK_OUTPUT_LIMIT = 5


def task_output_limit(root):
    path = path_in(root, "tools/runs/.output-settings.json", exists=False)
    if path.exists():
        try:
            config = json.loads(path.read_text(encoding="utf-8"))
        except ValueError as error:
            raise ConfigError("Invalid task output settings: " + str(error)) from error
        keys(config, {"task_output_limit"}, context="task output settings")
        return validate_task_output_limit(config["task_output_limit"])
    _, config = simulation_config(root)
    return config.get("task_output_limit", TASK_OUTPUT_LIMIT) if config else TASK_OUTPUT_LIMIT


def set_task_output_limit(root, value):
    """Persist settings atomically; deletion waits for successful task generation."""
    value = validate_task_output_limit(value)
    with output_guard(root):
        # Keep this service preference outside source fingerprints so changing it
        # cannot invalidate a prepared task or alter the running controller.
        path = path_in(root, "tools/runs/.output-settings.json", exists=False)
        path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps({"task_output_limit": value}) + "\n"
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=str(path.parent), prefix=".output-settings-", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(text); stream.flush(); os.fsync(stream.fileno())
            temporary.replace(path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
    return value


def _lock_path(root, name):
    directory = Path(tempfile.gettempdir()) / ("rlsim-output-locks-" + str(os.getuid()))
    directory.mkdir(mode=0o700, exist_ok=True)
    key = str(Path(root).resolve()) + "\0" + name
    return directory / (hashlib.sha256(key.encode()).hexdigest() + ".lock")


@contextmanager
def output_guard(root):
    with _lock_path(root, "generation").open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def output_lease(root, run_id, exclusive=False):
    """Caller closes the stream to release; None means another owner is active."""
    identifier(run_id)
    stream = _lock_path(root, "task:" + run_id).open("a+")
    try:
        fcntl.flock(stream, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        return None
    return stream


def _legacy_active(directory):
    # Old output directories predate leases. Verify the saved PID still points
    # at this directory, so stale "running" metadata cannot pin them forever.
    for name in ("run.json", "recording.json"):
        try:
            metadata = json.loads((directory / name).read_text())
            pid = metadata.get("pid")
            if type(pid) is int and pid > 0:
                command = Path("/proc/{}/cmdline".format(pid)).read_bytes()
                if os.fsencode(str(directory)) in command:
                    return True
        except (OSError, ValueError, AttributeError):
            continue
    return False


def prune_task_outputs(root, protected=()):
    """Run inside output_guard; return deleted IDs, never follow directory links.

    Active tasks count toward the configured slots. If more are protected,
    preserve them all and let a later successful generation retry cleanup.
    """
    limit = task_output_limit(root)
    outputs = Path(root).resolve() / "tools/runs"
    if limit == "all":
        return []
    if not outputs.is_dir():
        return []
    protected = set(protected)
    candidates, leases = [], []
    try:
        for directory in outputs.iterdir():
            if directory.name.startswith(".") or directory.is_symlink() or not directory.is_dir():
                continue
            snapshot = directory / "resolved_config.json"
            if snapshot.is_symlink() or not snapshot.is_file():
                continue
            try:
                identifier(directory.name)
            except ValueError:
                continue
            lease = output_lease(root, directory.name, exclusive=True)
            if lease:
                leases.append(lease)
            if lease is None or _legacy_active(directory):
                protected.add(directory.name)
            candidates.append((snapshot.stat().st_mtime_ns, directory.name, directory))
        candidates.sort(reverse=True)
        keep = {name for _, name, _ in candidates if name in protected}
        for _, name, _ in candidates:
            if len(keep) >= limit:
                break
            keep.add(name)
        removed = []
        for _, name, directory in candidates:
            if name not in keep:
                shutil.rmtree(directory)
                removed.append(name)
        return removed
    finally:
        for lease in leases:
            lease.close()
