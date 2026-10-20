"""Task output retention tests use temporary workspaces only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from rlsim_support.catalog import ConfigError, load_yaml
from rlsim_support.retention import output_guard, prune_task_outputs, set_task_output_limit, task_output_limit


class RetentionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.outputs = self.root / "tools/runs"
        self.outputs.mkdir(parents=True)

    def task(self, name, stamp):
        directory = self.outputs / name
        directory.mkdir()
        snapshot = directory / "resolved_config.json"
        snapshot.write_text("{}")
        os.utime(snapshot, ns=(stamp, stamp))
        (directory / "recording.csv").write_text("simulation_time_s\n1\n")
        (directory / "ros/log").mkdir(parents=True)
        (directory / "ros/log/legacy.log").write_text("old log")
        return directory

    def prune(self, protected=()):
        with output_guard(self.root):
            return prune_task_outputs(self.root, protected)

    def test_keeps_five_by_generation_time_not_robot_name_or_log_changes(self):
        names = ["alpha-old", "beta-old", "gamma-old", "delta-new", "alpha-new", "gamma-new", "delta-latest"]
        for i, name in enumerate(names): self.task(name, (i + 1) * 10**9)
        # Updating a recording in an older task must not change its task age.
        (self.outputs / names[0] / "recording.csv").write_text("recent append")
        removed = self.prune()
        self.assertEqual(set(removed), set(names[:2]))
        self.assertEqual({p.name for p in self.outputs.iterdir()}, set(names[-5:]))

    def test_configured_all_twenty_ten_and_five_apply_on_next_generation(self):
        for i in range(30): self.task("task-" + str(i), i + 1)
        set_task_output_limit(self.root, "all")
        self.assertEqual(self.prune(), [])
        for limit in (20, 10, 5):
            set_task_output_limit(self.root, limit)
            # Saving settings alone never deletes an archive.
            self.assertEqual(sum(p.is_dir() for p in self.outputs.iterdir()), 30 if limit == 20 else limit * 2)
            self.prune()
            self.assertEqual({p.name for p in self.outputs.iterdir() if p.is_dir()}, {"task-" + str(i) for i in range(30 - limit, 30)})

    def test_config_default_and_persistent_override_do_not_change_simulation_sources(self):
        configs = self.root / "configs"; configs.mkdir()
        original = (ROOT / "configs/simulation.yaml").read_text().replace("task_output_limit: 5", "task_output_limit: 10")
        config = configs / "simulation.yaml"; config.write_text(original)
        self.assertEqual(task_output_limit(self.root), 10)
        self.assertEqual(set_task_output_limit(self.root, 20), 20)
        self.assertEqual(task_output_limit(Path(str(self.root))), 20)
        self.assertEqual(config.read_text(), original)
        self.assertEqual(load_yaml(config)["task_output_limit"], 10)
        (self.outputs / ".output-settings.json").unlink()
        self.assertEqual(task_output_limit(self.root), 10)

    def test_invalid_values_and_corrupt_settings_cannot_trigger_cleanup(self):
        for value in (None, True, 5.0, 0, -1, 6, "5", [], {}):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                set_task_output_limit(self.root, value)
        for i in range(7): self.task("task-" + str(i), i + 1)
        (self.outputs / ".output-settings.json").write_text('{"task_output_limit":0}')
        with self.assertRaises(ConfigError): self.prune()
        self.assertEqual(sum(p.is_dir() for p in self.outputs.iterdir()), 7)
        (self.outputs / ".output-settings.json").write_text("invalid json")
        with self.assertRaises(ConfigError): self.prune()

    def test_failed_atomic_save_preserves_previous_limit_and_leaves_no_temp_file(self):
        set_task_output_limit(self.root, 20)
        with patch("rlsim_support.retention.Path.replace", side_effect=OSError("write failure")):
            with self.assertRaisesRegex(OSError, "write failure"):
                set_task_output_limit(self.root, 5)
        self.assertEqual(task_output_limit(self.root), 20)
        self.assertFalse(list(self.outputs.glob(".output-settings-*")))

    def test_preserves_hidden_entries_unknown_folders_and_symlink_targets(self):
        for i in range(7): self.task("task-" + str(i), i + 1)
        (self.outputs / ".gitignore").write_text("*\n!.gitignore\n")
        (self.outputs / ".locks").mkdir()
        (self.outputs / "notes").mkdir()
        outside = self.root / "outside"; outside.mkdir()
        (outside / "resolved_config.json").write_text("{}")
        (self.outputs / "linked-task").symlink_to(outside, target_is_directory=True)
        self.prune()
        self.assertTrue((self.outputs / ".gitignore").is_file())
        self.assertTrue((self.outputs / ".locks").is_dir())
        self.assertTrue((self.outputs / "notes").is_dir())
        self.assertTrue((outside / "resolved_config.json").is_file())
        self.assertTrue((self.outputs / "linked-task").is_symlink())

    def test_active_task_in_another_process_counts_toward_five_slots(self):
        for i in range(7): self.task("task-" + str(i), i + 1)
        code = "import sys;sys.path.insert(0,sys.argv[1]);from rlsim_support.retention import output_lease;lease=output_lease(sys.argv[2],'task-0');print('ready',flush=True);input();lease.close()"
        child = subprocess.Popen([sys.executable, "-c", code, str(ROOT / "tools"), str(self.root)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), "ready")
            self.prune()
            self.assertEqual({p.name for p in self.outputs.iterdir()}, {"task-0", "task-3", "task-4", "task-5", "task-6"})
        finally:
            child.communicate("\n", timeout=5)
        self.task("task-7", 8)
        self.prune()
        self.assertFalse((self.outputs / "task-0").exists())
        self.assertEqual(len(list(self.outputs.iterdir())), 5)

    def test_stale_legacy_running_metadata_does_not_pin_old_task(self):
        for i in range(6): self.task("task-" + str(i), i + 1)
        (self.outputs / "task-0/run.json").write_text(json.dumps({"state": "running", "pid": 2147483647}))
        self.prune()
        self.assertFalse((self.outputs / "task-0").exists())

    def test_protected_tasks_are_not_deleted_when_they_exceed_the_limit(self):
        for i in range(7): self.task("task-" + str(i), i + 1)
        self.prune({"task-" + str(i) for i in range(6)})
        self.assertEqual(len(list(self.outputs.iterdir())), 6)
        self.assertEqual(self.prune(), ["task-0"])


if __name__ == "__main__": unittest.main()
