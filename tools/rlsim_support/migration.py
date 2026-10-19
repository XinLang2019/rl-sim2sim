"""Verify imported resource bytes independently from the retired src layout."""
import json
from pathlib import Path
from collections import Counter

from .catalog import Catalog, path_in
from .legacy import ConfigError, sha256
from .relocation import relocated, current_path


MODEL_SUFFIXES = {".onnx", ".pt", ".jit", ".pth", ".141fonnx"}


def verify_resources(root):
    root = Path(root).resolve()
    manifest = json.loads((root / "tools/archive/resource-migration.json").read_text())
    if manifest.get("schema_version") != 1 or manifest.get("kind") != "resource_relocation":
        raise ConfigError("Unsupported relocation manifest")
    failures, checked, models = [], 0, 0
    destinations = set()
    for record in manifest["files"]:
        name = current_path(record["to"])
        if name in destinations or relocated(record["from"]) != name:
            failures.append("Invalid or duplicate relocation: " + name)
        destinations.add(name)
        # Adapter code evolves after the lossless move. The manifest retains
        # its original working-tree hash as provenance, not a runtime lockfile.
        if not name.startswith(("robots/", "policies/", "tools/archive/policy/", "tools/archive/policy_extras/", "third/")):
            continue
        path = path_in(root, name, exists=False)
        checked += 1
        if name.startswith("policies/") and path.suffix.lower() in MODEL_SUFFIXES:
            models += 1
        if "link" in record:
            actual = root / name
            import os
            if not actual.is_symlink() or os.readlink(actual) != record["link"]:
                failures.append("Symlink changed: " + name)
        elif not path.is_file() or sha256(path) != record["sha256"]:
            failures.append("Missing/changed resource: " + name)
    catalog = Catalog(root).list()
    statuses = Counter(item["deployment"]["status"] for item in catalog if "deployment" in item)
    return {"schema_version": 1, "ok": not failures, "resource_files_checked": checked,
            "models": models, "robots": sum(item["kind"] == "robot" for item in catalog),
            "policy_groups": dict(statuses), "diagnostics": failures,
            "validation_scope": "relocated_resource_bytes_and_catalog; adapter source hashes retained as pre-migration provenance"}
