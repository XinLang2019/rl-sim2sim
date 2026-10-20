"""Missing optional policy assets do not block unrelated deployments or bootstrap."""
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
from rlsim_support.catalog import Catalog, ConfigError, load_yaml
from rlsim_support.experiment import resolve
from rlsim_support.policy_assets import model_inventory
from tools.web.services.manager import Manager


class PolicyAvailabilityTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.config = self.root / 'configs/wheel/go2w.yaml'; self.config.parent.mkdir(parents=True)
        self.profile = load_yaml(ROOT / 'configs/wheel/go2w.yaml')
        self.policy = self.profile['policies']['go2w-ts-v001']
        self.profile['policies']['missing-checkpoint'] = {
            'schema_version': 1,
            'deployment': {'id': 'missing-checkpoint', 'robots': ['go2w'], 'status': 'unconfigured',
                           'diagnostics': ['Contract still needs configuration.'],
                           'model_directory': 'policies/wheel/go2w/removed'}}
        self.write_profile()
        for name in ('simulation.yaml', 'task.yaml'):
            shutil.copyfile(ROOT / 'configs' / name, self.root / 'configs' / name)

    def write_profile(self):
        self.config.write_text(yaml.safe_dump(self.profile, sort_keys=False))

    def model(self):
        file = self.root / self.policy['model']['file']; file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(b'ONNX inventory fixture; never executed')
        return file

    def test_missing_unconfigured_directory_is_isolated_and_source_is_unchanged(self):
        file = self.model(); original = self.config.read_bytes()
        catalog = Catalog(self.root)
        missing = next(p for p in catalog.list() if p['id'] == 'missing-checkpoint')
        self.assertFalse(missing['availability']['exists'])
        self.assertEqual(missing['deployment']['status'], 'blocked')
        self.assertIn('policies/wheel/go2w/removed', missing['availability']['error'])
        self.assertIn('Contract still needs configuration.', missing['deployment']['diagnostics'])
        self.assertEqual(catalog.deployments['go2w-ts-v001']['status'], 'ready')
        inventory = model_inventory(self.root, catalog)
        self.assertEqual([p['file'] for p in inventory], [file.relative_to(self.root).as_posix()])
        self.assertTrue(inventory[0]['selectable'])
        self.assertEqual(self.config.read_bytes(), original)
        self.assertFalse((self.root / 'policies/wheel/go2w/removed').exists())

    def test_missing_ready_directory_is_blocked_when_resolving_a_task(self):
        catalog = Catalog(self.root)
        self.assertEqual(catalog.deployments['go2w-ts-v001']['status'], 'blocked')
        with self.assertRaisesRegex(ConfigError, 'Policy is blocked:.*策略目录不存在'):
            resolve(self.root, {'robot': 'go2w', 'policy': 'go2w-ts-v001', 'terrain': 'flat'})

    def test_readding_assets_restores_declared_status_on_refresh(self):
        self.assertEqual(Catalog(self.root).deployments['go2w-ts-v001']['status'], 'blocked')
        self.model()
        (self.root / 'policies/wheel/go2w/removed').mkdir()
        refreshed = Catalog(self.root)
        self.assertEqual(refreshed.deployments['go2w-ts-v001']['status'], 'ready')
        self.assertEqual(refreshed.deployments['missing-checkpoint']['status'], 'unconfigured')
        self.assertEqual(refreshed.deployments['missing-checkpoint']['diagnostics'], ['Contract still needs configuration.'])

    def test_bootstrap_catalog_preserves_available_models_and_missing_reasons(self):
        self.model()
        manager = Manager(self.root, None); self.addCleanup(manager.close)
        data = manager.catalog()
        self.assertIn('go2w-ts-v001', data['policy_details'])
        self.assertTrue(data['models'][0]['selectable'])
        missing = next(p for p in data['resources'] if p['id'] == 'missing-checkpoint')
        self.assertFalse(missing['availability']['exists'])

    def test_outside_paths_are_rejected_even_when_the_target_is_missing(self):
        metadata = self.profile['policies']['missing-checkpoint']['deployment']
        for value in ('../outside', 'configs/missing'):
            metadata['model_directory'] = value; self.write_profile()
            with self.subTest(path=value), self.assertRaises(ConfigError):
                Catalog(self.root)
