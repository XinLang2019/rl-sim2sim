#!/usr/bin/env python3
"""Inspect policies, compile experiments and serve the local simulation workbench."""
import argparse
import json
import sys
from pathlib import Path

try:
    from rlsim_support.validation import ConfigError
except ModuleNotFoundError as error:
    if error.name != "yaml":
        raise
    sys.exit("PyYAML is required. Install tools/requirements.txt or use a Python environment with PyYAML.")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1], help="Repository root; independent of current working directory")
    commands = result.add_subparsers(dest="command", required=True)
    serving = commands.add_parser("serve", help="Serve the local web workbench on 127.0.0.1")
    serving.add_argument("--port", type=int, default=8765)
    serving.add_argument("--policy-run", type=Path, required=True)
    serving.add_argument("--ros-python", type=Path, default=Path("/usr/bin/python3"))
    serving.add_argument("--xacro", help="Explicit xacro executable")
    assets = commands.add_parser("assets", help="List registered robot, policy, terrain and backend assets")
    assets.add_argument("--kind", choices=("robot", "policy", "terrain", "backend", "control", "experiment"))
    assets.add_argument("--json", action="store_true")
    assets.add_argument("--check", action="store_true", help="Validate canonical robot geometry, meshes, controllers and Gazebo expansion")
    assets.add_argument("--xacro", help="Explicit xacro executable for asset validation")
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
        if args.command == "assets":
            if args.check:
                if args.kind not in (None, "robot"):
                    raise ConfigError("assets --check validates robot packages; use --kind robot or omit --kind")
                from rlsim_support.robot_assets import check_packages
                report = check_packages(args.root, xacro=args.xacro)
                if args.json:
                    print(json.dumps(report, ensure_ascii=False, indent=2))
                else:
                    for item in report["packages"]:
                        print(("PASS " if item["ok"] else "FAIL ") + item["package"] + (": " + item["error"] if not item["ok"] else ""))
                return 0 if report["ok"] else 1
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
        raise ConfigError("Unsupported command")
    except (ConfigError, OSError, ValueError, KeyError, TypeError) as error:
        print("rlsim: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
