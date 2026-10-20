"""Publication isolation tests use synthetic fixtures, never local robot assets."""
import importlib.util
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

spec = importlib.util.spec_from_file_location('public_export', Path(__file__).parents[1] / 'export_public.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PublicExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'source'
        self.root.mkdir()
        (self.root / 'code').mkdir()
        (self.root / 'code/main.py').write_text('print("public")\n')
        (self.root / '.git').mkdir()
        (self.root / '.git/config').write_text('historical secret')
        (self.root / '.private').mkdir()
        self.policy = self.root / '.private/policy.json'
        self.data = {'version': 1, 'include': ['code'], 'forbidden_terms': ['private_bot'],
                     'private_roots': ['local-private'], 'binary_roots': ['code/models']}
        self.output = self.root.parent / 'public'

    def exporter(self):
        self.policy.write_text(json.dumps(self.data))
        return module.Exporter(self.root, self.policy)

    def test_clean_export_and_audit_no_history_or_policy(self):
        exporter = self.exporter()
        exporter.export(self.output)
        self.assertEqual(exporter.audit(self.output)['checked_files'], 1)
        self.assertFalse((self.output / '.git').exists())
        self.assertFalse((self.output / '.private').exists())
        self.assertFalse(any(p.is_symlink() for p in self.output.rglob('*')))
        with self.assertRaises(module.PublicationError):
            exporter.export(self.output)

    def test_renamed_text_is_blocked_and_no_partial_release_remains(self):
        (self.root / 'code/ordinary.txt').write_text('model: private_bot-v001')
        with self.assertRaises(module.PublicationError):
            self.exporter().export(self.output)
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.output.parent.glob('.public-stage-*')))

    def test_protected_paths_blocked(self):
        (self.root / 'code/private_bot.yaml').write_text('value: 1')
        with self.assertRaises(module.PublicationError):
            self.exporter().check()

    def test_renamed_private_model_blocked_by_hash(self):
        (self.root / 'local-private').mkdir()
        data = b'\x00\xffprotected-weights'
        (self.root / 'local-private/weights.onnx').write_bytes(data)
        (self.root / 'code/models').mkdir()
        (self.root / 'code/models/ordinary.onnx').write_bytes(data)
        with self.assertRaisesRegex(module.PublicationError, 'duplicates'):
            self.exporter().export(self.output)

    def test_private_fingerprints_survive_archive_cleanup(self):
        data = b'\x00\xffprotected-archived-weights'
        self.data['protected_sha256'] = [module.digest(data)]
        # The archive no longer exists, but a renamed copy must remain private.
        (self.root / 'code/models').mkdir()
        (self.root / 'code/models/ordinary.onnx').write_bytes(data)
        with self.assertRaisesRegex(module.PublicationError, 'duplicates'):
            self.exporter().export(self.output)

    def test_unreviewed_binary_and_changed_image_blocked(self):
        image = self.root / 'code/preview.png'
        image.write_bytes(b'\xff\x00image')
        with self.assertRaisesRegex(module.PublicationError, 'Unreviewed binary'):
            self.exporter().check()
        self.data['reviewed_media'] = {'code/preview.png': module.digest(image.read_bytes())}
        self.exporter().check()
        image.write_bytes(b'\xff\x00different-image')
        with self.assertRaisesRegex(module.PublicationError, 'Reviewed image changed'):
            self.exporter().check()

    def test_symlink_cannot_import_unlisted_files(self):
        (self.root / 'local-private').mkdir()
        (self.root / 'local-private/ordinary.txt').write_text('private_bot')
        (self.root / 'code/alias.txt').symlink_to('../local-private/ordinary.txt')
        with self.assertRaisesRegex(module.PublicationError, 'outside release scope'):
            self.exporter().check()

    def test_internal_symlink_materialized_without_link(self):
        (self.root / 'code/alias.py').symlink_to('main.py')
        self.exporter().export(self.output)
        self.assertTrue((self.output / 'code/alias.py').is_file())
        self.assertFalse((self.output / 'code/alias.py').is_symlink())

    def test_audit_rejects_extra_or_changed_file(self):
        exporter = self.exporter()
        exporter.export(self.output)
        (self.output / 'unexpected.txt').write_text('not reviewed')
        with self.assertRaises(module.PublicationError):
            exporter.audit(self.output)
        (self.output / 'unexpected.txt').unlink()
        (self.output / 'code/main.py').write_text('modified')
        with self.assertRaises(module.PublicationError):
            exporter.audit(self.output)

    def test_policy_and_parent_path_cannot_be_published(self):
        self.data['include'].append('../outside')
        with self.assertRaises(module.PublicationError):
            self.exporter()
        self.data['include'] = ['code', '.private/policy.json']
        # Private policy is always filtered even if mistakenly allowlisted.
        exporter = self.exporter()
        exporter.export(self.output)
        self.assertFalse((self.output / '.private').exists())

    def test_new_unlisted_files_are_not_automatically_published(self):
        (self.root / 'new-resource.onnx').write_bytes(b'\x00\xffunclassified')
        self.exporter().export(self.output)
        self.assertFalse((self.output / 'new-resource.onnx').exists())

    def test_model_metadata_is_checked_without_deserializing_weights(self):
        (self.root / 'code/models').mkdir()
        (self.root / 'code/models/public.onnx').write_bytes(b'\x00\xff/home/training/private_bot/actor\x00')
        with self.assertRaisesRegex(module.PublicationError, 'model metadata'):
            self.exporter().check()

    def test_audit_checks_manifest_content_too(self):
        exporter = self.exporter()
        exporter.export(self.output)
        path = self.output / module.MANIFEST
        value = json.loads(path.read_text())
        value['note'] = 'private_bot'
        path.write_text(json.dumps(value))
        with self.assertRaises(module.PublicationError):
            exporter.audit(self.output)

    def test_compressed_model_source_is_checked_without_executing_it(self):
        (self.root / 'code/models').mkdir()
        path = self.root / 'code/models/ordinary.pt'
        with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('archive/code/model.py', 'robot_name = "private_bot"')
        with self.assertRaisesRegex(module.PublicationError, 'release content'):
            self.exporter().check()


if __name__ == '__main__':
    unittest.main()
