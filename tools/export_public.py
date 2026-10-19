#!/usr/bin/env python3
"""Export a reviewed allowlist into a new directory, without Git history.

The local policy is deliberately excluded from the export. Nothing is uploaded.
An existing destination is never merged with or overwritten by this tool.
"""
import argparse
import fnmatch
import hashlib
import io
import json
import re
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path


ALWAYS_SKIP = {".git", ".private", ".aws", ".codex", ".agents", "__pycache__", ".pytest_cache", "node_modules"}
GENERATED = ("tools/build/", "tools/runs/", "videos/", "build/", "devel/", "install/", "logs/", "log/")
FINGERPRINT_SUFFIXES = {".onnx", ".pt", ".jit", ".stl", ".dae", ".png", ".jpg", ".jpeg", ".mp4", ".bag", ".mcap"}
MANIFEST = "PUBLIC_RELEASE.json"


class PublicationError(ValueError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def skipped(name):
    path = Path(name)
    return (any(part in ALWAYS_SKIP or part.startswith(".env") for part in path.parts)
            or name.startswith(GENERATED) or path.suffix in (".pyc", ".pyo"))


def relative_name(name):
    if not isinstance(name, str) or not name or "\\" in name:
        raise PublicationError("Policy paths must be nonempty relative POSIX paths")
    path = Path(name)
    if path.is_absolute() or ".." in path.parts or str(path) == ".":
        raise PublicationError("Policy paths must remain inside the source tree")
    return path.as_posix()


class Exporter:
    def __init__(self, root, policy):
        self.root = Path(root).resolve()
        self.policy_path = Path(policy).resolve()
        self.policy = json.loads(self.policy_path.read_text(encoding="utf-8"))
        if self.policy.get("version") != 1 or not self.policy.get("include") or not self.policy.get("forbidden_terms"):
            raise PublicationError("Policy requires version=1, include and forbidden_terms")
        for key in ("include", "exclude", "private_roots", "binary_roots"):
            self.policy[key] = [relative_name(name) for name in self.policy.get(key, [])]
        terms = self.policy["forbidden_terms"]
        if not all(isinstance(term, str) and term for term in terms):
            raise PublicationError("forbidden_terms must contain nonempty strings")
        self.forbidden = re.compile(r"(?<![a-z0-9])(?:" + "|".join(re.escape(term) for term in terms) + r")(?![a-z0-9])", re.I)
        self.private_hashes = self._fingerprints()

    def _fingerprints(self):
        result = set()
        for pattern in self.policy["private_roots"]:
            for path in self.root.glob(pattern):
                for item in path.rglob("*") if path.is_dir() else (path,):
                    if item.is_file() and not item.is_symlink() and item.suffix.lower() in FINGERPRINT_SUFFIXES:
                        result.add(digest(item.read_bytes()))
        return result

    def excluded(self, name):
        return skipped(name) or any(fnmatch.fnmatchcase(name, pattern) for pattern in self.policy["exclude"])

    def sources(self):
        selected = {}
        for name in self.policy["include"]:
            source = self.root / name
            if not source.exists():
                raise PublicationError("Allowlisted source is missing: " + name)
            for path in source.rglob("*") if source.is_dir() else (source,):
                rel = path.relative_to(self.root).as_posix()
                if self.excluded(rel):
                    continue
                if path.is_symlink():
                    # Only materialize links whose target is itself allowlisted.
                    try:
                        target = path.resolve(strict=True).relative_to(self.root).as_posix()
                    except (OSError, ValueError) as error:
                        raise PublicationError("External/broken symlink: " + rel) from error
                    if path.is_dir() or self.excluded(target) or not self.allowed(target):
                        raise PublicationError("Symlink target is outside release scope: " + rel)
                if path.is_file():
                    if path.resolve() == self.policy_path:
                        raise PublicationError("Private publication policy cannot be exported")
                    selected[rel] = path
        # The developer ignore rules can themselves contain confidential names.
        template = self.policy.get("public_gitignore")
        if template:
            selected[".gitignore"] = self.root / relative_name(template)
        if not selected:
            raise PublicationError("Empty release")
        return selected

    def allowed(self, name):
        return any(name == entry or name.startswith(entry + "/") for entry in self.policy["include"])

    def inspect(self, name, data):
        if self.forbidden.search(name):
            raise PublicationError("Confidential identifier in release path: " + name)
        fingerprint = digest(data)
        if fingerprint in self.private_hashes:
            raise PublicationError("File duplicates a protected asset (possibly renamed): " + name)
        approved = self.policy.get("reviewed_media", {}).get(name)
        if approved is not None and fingerprint != approved:
            raise PublicationError("Reviewed image changed; inspect it before updating its approval: " + name)
        try:
            text = data.decode("utf-8")
            if "\0" in text:
                raise UnicodeError()
        except UnicodeError:
            if not approved and not any(name == base or name.startswith(base + "/") for base in self.policy["binary_roots"]):
                raise PublicationError("Unreviewed binary file: " + name)
            if Path(name).suffix.lower() in (".onnx", ".pt", ".jit"):
                # Inspect readable names/paths without interpreting tensor bytes
                # as text or deserializing executable pickle payloads.
                for run in re.finditer(rb"[\x20-\x7e]{12,}", data):
                    value = run.group().decode("ascii")
                    if re.fullmatch(r"[A-Za-z0-9_./\\: -]+", value) and self.forbidden.search(value):
                        raise PublicationError("Confidential identifier in readable model metadata: " + name)
                # TorchScript can compress its source and metadata. Read only
                # metadata entries, without extracting or executing the archive.
                archive = io.BytesIO(data)
                if zipfile.is_zipfile(archive):
                    with zipfile.ZipFile(archive) as stream:
                        total = 0
                        for entry in stream.infolist():
                            if self.forbidden.search(entry.filename):
                                raise PublicationError("Confidential model archive entry: " + name)
                            if Path(entry.filename).suffix not in (".py", ".json", ".txt", ".pkl", ".debug_pkl"):
                                continue
                            total += entry.file_size
                            if total > 32 * 1024 * 1024:
                                raise PublicationError("Model metadata exceeds review limit: " + name)
                            payload = stream.read(entry)
                            # Pickle is never deserialized; its printable paths
                            # are covered by the same conservative string scan.
                            if entry.filename.endswith((".pkl", ".debug_pkl")):
                                for run in re.finditer(rb"[\x20-\x7e]{12,}", payload):
                                    value = run.group().decode("ascii")
                                    if re.fullmatch(r"[A-Za-z0-9_./\\: -]+", value) and self.forbidden.search(value):
                                        raise PublicationError("Confidential model archive metadata: " + name)
                            else:
                                self.inspect(name + "!/" + entry.filename, payload)
        else:
            if self.forbidden.search(text):
                raise PublicationError("Confidential identifier in release content: " + name)
        return fingerprint

    def check(self):
        sources = self.sources()
        for name, source in sources.items():
            self.inspect(name, source.read_bytes())
        return {"checked_files": len(sources), "uploaded": False}

    def export(self, output):
        output = Path(output).absolute()
        if output.exists() or output.is_symlink():
            raise PublicationError("Destination must not exist; choose a new directory")
        if self.root == output or output in self.root.parents:
            raise PublicationError("Destination cannot contain the source tree")
        try:
            local_name = output.relative_to(self.root).as_posix()
        except ValueError:
            local_name = None
        if local_name and self.allowed(local_name) and not self.excluded(local_name):
            raise PublicationError("Destination must be outside the allowlisted source paths")
        output.parent.mkdir(parents=True, exist_ok=True)
        sources = self.sources()
        stage = Path(tempfile.mkdtemp(prefix=".public-stage-", dir=str(output.parent)))
        try:
            records = []
            for name, source in sorted(sources.items()):
                data = source.read_bytes()
                fingerprint = self.inspect(name, data)
                target = stage / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                target.chmod(0o755 if source.stat().st_mode & 0o111 else 0o644)
                records.append({"path": name, "bytes": len(data), "sha256": fingerprint})
            (stage / MANIFEST).write_text(json.dumps({"version": 1, "git_history": False, "files": records}, indent=2) + "\n", encoding="utf-8")
            stage.rename(output)
        finally:
            if stage.exists():
                shutil.rmtree(stage)
        return {"directory": str(output), "files": len(records), "git_history": False, "uploaded": False}

    def audit(self, directory):
        directory = Path(directory).resolve()
        manifest_data = (directory / MANIFEST).read_bytes()
        self.inspect(MANIFEST, manifest_data)
        manifest = json.loads(manifest_data.decode("utf-8"))
        if manifest.get("version") != 1 or manifest.get("git_history") is not False:
            raise PublicationError("Invalid public release manifest")
        expected = {item["path"]: item["sha256"] for item in manifest["files"]}
        if len(expected) != len(manifest["files"]):
            raise PublicationError("Duplicate release manifest paths")
        actual = set()
        for path in directory.rglob("*"):
            name = path.relative_to(directory).as_posix()
            if path.is_symlink() or skipped(name):
                raise PublicationError("Unexpected local state or symlink in release: " + name)
            if not path.is_file() or name == MANIFEST:
                continue
            actual.add(name)
            if name != ".gitignore" and (not self.allowed(name) or self.excluded(name)):
                raise PublicationError("File outside release allowlist: " + name)
            if self.inspect(name, path.read_bytes()) != expected.get(name):
                raise PublicationError("Release file changed: " + name)
        if actual != set(expected):
            raise PublicationError("Release manifest and actual file set differ")
        return {"checked_files": len(actual), "git_history": False, "uploaded": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--policy", type=Path, help="Local publication policy; defaults to ROOT/.private/publication.json")
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--check", action="store_true", help="Check the allowlisted source without exporting")
    actions.add_argument("--output", type=Path, help="Create a new clean release directory")
    actions.add_argument("--audit", type=Path, help="Verify a previously exported directory before git init")
    args = parser.parse_args(argv)
    try:
        exporter = Exporter(args.root, args.policy or args.root / ".private/publication.json")
        report = exporter.check() if args.check else exporter.audit(args.audit) if args.audit else exporter.export(args.output)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, KeyError) as error:
        print("Publication blocked: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
