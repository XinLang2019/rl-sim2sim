#!/usr/bin/env python3
"""Inspect policies, compile experiments and serve the local simulation workbench."""
import argparse
import json
import sys
from pathlib import Path

try:
    from rlsim_support.baseline import capture, verify, write_json
    from rlsim_support.legacy import ConfigError, POLICY_TYPES, RUNNERS, audit, inventory
    from rlsim_support.export import export_policy
except ModuleNotFoundError as error:
    if error.name != "yaml":
        raise
    sys.exit("PyYAML is required. Install tools/requirements.txt or use a Python environment with PyYAML.")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1], help="Repository root; independent of current working directory")
    commands = result.add_subparsers(dest="command", required=True)
    migration = commands.add_parser("migration", help="Verify relocated resource bytes and catalog coverage")
    migration.add_argument("action", choices=("verify",))
    migration.add_argument("--json", action="store_true")
    serving = commands.add_parser("serve", help="Serve the local web workbench on 127.0.0.1")
    serving.add_argument("--port", type=int, default=8765)
    serving.add_argument("--policy-run", type=Path, required=True)
    serving.add_argument("--ros-python", type=Path, default=Path("/usr/bin/python3"))
    serving.add_argument("--xacro", help="Explicit xacro executable")
    listing = commands.add_parser("inventory", help="Audit archived legacy config.yaml profiles; use assets for the current catalog")
    listing.add_argument("--runner", choices=RUNNERS, default="rl_wheel")
    listing.add_argument("--probe", type=Path, help="Path to compiled rlsim_model_probe")
    listing.add_argument("--json", action="store_true")
    for name in ("check", "capture", "export"):
        help_text = {"check": "Inspect a profile", "capture": "Capture immutable contract, file hashes and model fixtures",
                     "export": "Export a validated legacy TS profile to a v1 core policy manifest"}
        command = commands.add_parser(name, help=help_text[name])
        command.add_argument("profile", help="ROBOT/PROFILE, e.g. go2w/legged_gym")
        command.add_argument("--runner", choices=RUNNERS, default="rl_wheel")
        command.add_argument("--policy-type", choices=POLICY_TYPES, help="Inspection override; never edits YAML or fixes runtime load errors")
        command.add_argument("--probe", type=Path, required=name in ("capture", "export"))
        if name in ("capture", "export"):
            command.add_argument("--output", type=Path, required=True)
        else:
            command.add_argument("--json", action="store_true")
    checking = commands.add_parser("verify", help="Verify captured files, contract and synthetic model I/O")
    checking.add_argument("baseline", type=Path)
    checking.add_argument("--probe", type=Path, required=True)
    checking.add_argument("--allow-source-drift", action="store_true", help="Allow changes to captured runtime source files; resources and configuration remain strict")
    checking.add_argument("--rtol", type=float, default=1e-5)
    checking.add_argument("--atol", type=float, default=1e-6)
    checking.add_argument("--json", action="store_true")
    assets = commands.add_parser("assets", help="List registered robot, policy, terrain and backend assets")
    assets.add_argument("--kind", choices=("robot", "policy", "terrain", "backend", "control", "experiment"))
    assets.add_argument("--json", action="store_true")
    experiments = commands.add_parser("experiment", help="Resolve, prepare and manage an experiment")
    actions = experiments.add_subparsers(dest="action", required=True)
    for name in ("check", "prepare", "run"):
        action = actions.add_parser(name)
        action.add_argument("experiment", help="Experiment id or robot profile YAML#/experiments/ID")
        action.add_argument("--terrain", help="Select a different registered terrain")
        action.add_argument("--terrain-param", action="append", default=[], metavar="KEY=VALUE")
        action.add_argument("--policy-run", type=Path, required=name != "check", help="Compiled rlsim_policy_run for actual model validation")
        if name != "check":
            action.add_argument("--output", type=Path)
            action.add_argument("--xacro", help="Explicit xacro executable")
        action.add_argument("--json", action="store_true")
    for name in ("start", "inspect", "command"):
        action = actions.add_parser(name)
        action.add_argument("directory", type=Path)
        action.add_argument("--json", action="store_true")
        if name == "command":
            action.add_argument("control", choices=("stand", "policy", "sit", "passive", "reset", "toggle"))
    return result


def show(report):
    reports = report.get("reports", [report])
    for item in reports:
        print("{} {} [{}]".format("PASS" if item["ok"] else "FAIL", item["profile"], item.get("runner", "baseline")))
        contract = item.get("contract")
        if contract:
            obs = contract["observation"]
            print("  {} | {} DOFs | obs {} -> {} | {}".format(
                contract["policy_type"], contract["robot"]["dofs"], obs["width"], obs["model_input_width"], contract["model"]))
        for diagnostic in item["diagnostics"]:
            print("  {} {}: {}".format(diagnostic["level"].upper(), diagnostic["code"], diagnostic["message"]))
    if "summary" in report:
        print("Profiles: {profiles}; passed: {passed}; failed: {failed}".format(**report["summary"]))


def experiment_command(args):
    import yaml
    import xml.etree.ElementTree as ET
    from rlsim_support.experiment import prepare, resolve
    from rlsim_support.session import launch_run, send_command
    try:
        if args.action in ("check", "prepare", "run"):
            parameters = {}
            for override in args.terrain_param:
                key, separator, value = override.partition("=")
                if not separator or not key or key in parameters:
                    raise ConfigError("Terrain overrides require unique KEY=VALUE pairs")
                parameters[key] = yaml.safe_load(value)
            if args.action == "check":
                report, _, _ = resolve(args.root, args.experiment, args.terrain, parameters, args.policy_run)
                if args.json:
                    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
                else:
                    print("PASS experiment {}: {}".format(report["id"], report["selection"]))
                    print("  Actual model checked: " + str(report["model_checked"]))
                    for diagnostic in report["diagnostics"]:
                        print("  {level} {code}: {message}".format(**diagnostic))
                return 0
            directory = prepare(args.root, args.experiment, args.output, args.terrain, parameters, args.policy_run, args.xacro)
            if args.action == "prepare":
                report = {"directory": str(directory), "state": "prepared"}
            else:
                print("Prepared run: " + str(directory), file=sys.stderr, flush=True)
                report = launch_run(directory)
        elif args.action == "start":
            report = launch_run(args.directory)
        elif args.action == "command":
            report = send_command(args.directory, args.control)
        else:
            report = json.loads((args.directory / "run.json").read_text())
            resolved = json.loads((args.directory / "resolved_config.json").read_text())
            report.update(selection=resolved["selection"], diagnostics=resolved["diagnostics"])
        print(json.dumps(report, indent=2, ensure_ascii=False) if args.json else str(report))
        return 1 if report.get("state") == "failed" else 0
    except (ET.ParseError, yaml.YAMLError) as error:
        raise ConfigError("Malformed resource document: " + str(error)) from error


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if not (args.root / "policies").is_dir() or not (args.root / "configs").is_dir():
            raise ConfigError("Repository root must contain policies/ and configs/")
        if args.command == "serve":
            import signal
            if not 1 <= args.port <= 65535:
                raise ConfigError("Port must be 1..65535")
            if not args.policy_run.is_file():
                raise ConfigError("Build rlsim_policy_run before starting the workbench")
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from tools.web.services.manager import Manager
            from tools.web.services.http_server import Server
            manager = Manager(args.root, args.policy_run.resolve(), args.ros_python, args.xacro)
            server = Server(("127.0.0.1", args.port), manager)
            def terminate(*_):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, terminate)
            print("RL sim2sim workbench: http://127.0.0.1:" + str(server.server_port), flush=True)
            try:
                server.serve_forever(poll_interval=.2)
            except KeyboardInterrupt:
                pass
            finally:
                server.server_close()
                manager.close()
            return 0
        if args.command == "migration":
            from rlsim_support.migration import verify_resources
            report = verify_resources(args.root)
            print(json.dumps(report, ensure_ascii=False, indent=2) if args.json else str(report))
            return 0 if report["ok"] else 1
        if args.command == "assets":
            from rlsim_support.catalog import Catalog
            items = Catalog(args.root).list()
            items = [item for item in items if not args.kind or item["kind"] == args.kind]
            if args.json:
                print(json.dumps(items, ensure_ascii=False, indent=2))
            else:
                for item in items:
                    print("{kind:8} {id:36} {manifest}  {status}".format(**item, status=item.get("deployment", {}).get("status", "")))
            return 0
        if args.command == "experiment":
            return experiment_command(args)
        if args.command == "inventory":
            report = inventory(args.root, args.runner, args.probe)
            success = report["summary"]["failed"] == 0
        elif args.command == "check":
            report = audit(args.root, args.profile, args.runner, args.policy_type, args.probe)
            success = report["ok"]
        elif args.command == "capture":
            report = capture(args.root, args.profile, args.runner, args.policy_type, args.probe)
            write_json(args.output, report)
            print("Captured " + str(args.output))
            return 0
        elif args.command == "export":
            from rlsim_support.yaml_format import dumps
            config = export_policy(args.root, args.profile, args.runner, args.policy_type, args.probe)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(dumps(config))
            print("Exported " + str(args.output))
            return 0
        else:
            baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
            report = verify(args.root, baseline, args.probe, args.allow_source_drift, args.rtol, args.atol)
            success = report["ok"]
        if args.json:
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        else:
            show(report)
        return 0 if success else 1
    except (ConfigError, OSError, ValueError, KeyError, TypeError) as error:
        print("rlsim: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
