"""Versioned, relocatable snapshots of legacy contracts and raw model fixtures."""
import datetime
import json
import math
import platform
import subprocess
from pathlib import Path

from .legacy import ConfigError, audit, probe_model, relative, sha256
from .relocation import relocated, relocate_contract


def fingerprints(root, contract):
    root = Path(root).resolve()
    runtime = root / "tools/archive/legacy_runtime"
    paths = set()
    for directory in (runtime / "src", runtime / "include", runtime / "library/core", root / "rlsim/control", root / "rlsim/interfaces/gazebo/rlsim_gazebo"):
        for pattern in ("*.cpp", "*.hpp", "*.h"):
            paths.update(directory.rglob(pattern))
    paths.update((root / "tools/archive/policy").glob("*/fsm.hpp"))
    paths.add(root / "tools/archive/policy/fsm.hpp")
    sources = {relative(root, p): sha256(p) for p in sorted(paths) if p.is_file()}
    assets = root / relocated("robots/" + contract["robot"]["id"] + "_description")
    resource_paths = [root / name for name in contract["sources"]] + [root / contract["model"]]
    if assets.is_dir():
        resource_paths += [p for p in assets.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    resources = {relative(root, p): sha256(p) for p in sorted(resource_paths)}
    return {"runtime_sources": sources, "resources": resources}


def capture(root, profile, runner, policy_type, probe):
    report = audit(root, profile, runner, policy_type, probe)
    if not report["ok"] or report["model_check"] != "inspected":
        problems = "; ".join(d["code"] + ": " + d["message"] for d in report["diagnostics"] if d["level"] == "error")
        raise ConfigError("Cannot capture a validated baseline: " + (problems or "model inspection is required"))
    contract = report["contract"]
    # Do not invent values for symbolic dimensions. A future deployment manifest
    # must resolve their semantics before they can become a baseline.
    for tensor in report["model"]["inputs"]:
        if any(dim <= 0 for dim in tensor["shape"]):
            raise ConfigError("Baseline requires concrete input shapes: " + tensor["name"])
    fixtures = [probe_model(probe, Path(root) / contract["model"], sample=sample)
                for sample in ("zero", "ramp")]
    try:
        commit = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    return {"schema_version": 1, "kind": "legacy_baseline",
            "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "provenance": {"git_commit": commit, "platform": platform.platform(),
                           "note": "File hashes describe working tree bytes, including uncommitted changes."},
            "validation_scope": "configuration_and_single_step_synthetic_model_io",
            "contract": contract, "diagnostics": report["diagnostics"],
            "fingerprints": fingerprints(root, contract), "fixtures": fixtures}


def compare_values(expected, actual, path="", rtol=1e-5, atol=1e-6):
    """Return the first structural/numerical mismatch, with a useful path."""
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            return path + ": object keys differ"
        for key in expected:
            error = compare_values(expected[key], actual[key], path + "/" + key, rtol, atol)
            if error:
                return error
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            return path + ": array length differs"
        for i, (a, b) in enumerate(zip(expected, actual)):
            error = compare_values(a, b, path + "/" + str(i), rtol, atol)
            if error:
                return error
    elif isinstance(expected, float) or (type(expected) is int and "/values/" in path):
        if isinstance(actual, bool) or not isinstance(actual, (float, int)) or not math.isfinite(expected) or not math.isfinite(actual):
            return path + ": non-finite or non-numeric value"
        if not math.isclose(expected, actual, rel_tol=rtol, abs_tol=atol):
            return "{}: {} != {}".format(path, expected, actual)
    elif type(expected) is not type(actual) or expected != actual:
        return "{}: {!r} != {!r}".format(path, expected, actual)
    return None


def verify(root, baseline, probe, allow_source_drift=False, rtol=1e-5, atol=1e-6):
    if not isinstance(baseline, dict) or baseline.get("schema_version") != 1 or baseline.get("kind") != "legacy_baseline":
        raise ConfigError("Unsupported baseline format")
    for tolerance in (rtol, atol):
        if not math.isfinite(tolerance) or tolerance < 0:
            raise ConfigError("Tolerances must be finite and non-negative")
    contract = relocate_contract(baseline["contract"])
    report = audit(root, contract["profile"], contract["runner"], contract["policy_type"], probe)
    messages = []

    def issue(level, code, message):
        messages.append({"level": level, "code": code, "message": message})

    if not report["ok"]:
        messages.extend(d for d in report["diagnostics"] if d["level"] == "error")
    if report["contract"] != contract:
        issue("error", "CONTRACT_CHANGED", "Resolved legacy contract differs from captured baseline")
    for category, files in baseline["fingerprints"].items():
        for name, digest in files.items():
            path = (Path(root) / relocated(name)).resolve()
            relative(Path(root), path)  # Also reject paths outside this workspace.
            if not path.is_file() or sha256(path) != digest:
                level = "warning" if allow_source_drift and category == "runtime_sources" else "error"
                issue(level, "FILE_CHANGED", name)
    if report.get("model_check") == "inspected":
        for fixture in baseline["fixtures"]:
            generator = fixture["fixture"]["generator"]
            if generator not in ("zero_v1", "ramp_v1"):
                raise ConfigError("Unknown baseline input generator: " + generator)
            actual = probe_model(probe, Path(root) / contract["model"], sample=generator[:-3])
            error = compare_values(fixture, actual, generator, rtol, atol)
            if error:
                issue("error", "MODEL_FIXTURE_CHANGED", error)
    else:
        issue("error", "MODEL_NOT_VERIFIED", "Model fixtures could not be executed")
    return {"schema_version": 1, "profile": contract["profile"],
            "ok": not any(d["level"] == "error" for d in messages),
            "diagnostics": messages, "rtol": rtol, "atol": atol,
            "validation_scope": baseline["validation_scope"]}


def write_json(path, data):
    path = Path(path)
    serialized = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    # Baselines are immutable by default. Capture to a new path for a new version.
    with path.open("x", encoding="utf-8") as stream:
        stream.write(serialized)
