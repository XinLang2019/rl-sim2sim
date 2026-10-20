"""Asset dependencies and Gazebo expansion, without starting a simulator."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from rlsim_support.catalog import ConfigError
from rlsim_support.robot_assets import check_packages, mesh_dependencies


class RobotAssetTests(unittest.TestCase):
    def test_mesh_dependencies_keep_collada_image_files(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "demo_description"
            meshes = package / "meshes"; meshes.mkdir(parents=True)
            mesh = meshes / "body.dae"
            mesh.write_text('<COLLADA xmlns="urn:collada"><library_images><image id="texture"><init_from>body.png</init_from></image></library_images></COLLADA>')
            texture = meshes / "body.png"; texture.write_bytes(b"fixture")
            model = ET.fromstring('<robot><link><visual><geometry><mesh filename="package://demo_description/meshes/body.dae"/></geometry></visual></link></robot>')
            self.assertEqual(mesh_dependencies(package, model), {mesh, texture})
            texture.unlink()
            with self.assertRaisesRegex(ConfigError, "Missing robot mesh or texture"):
                mesh_dependencies(package, model)

    def test_mesh_paths_cannot_escape_the_robot_package(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "demo_description"; package.mkdir()
            outside = Path(directory) / "outside.stl"; outside.write_bytes(b"fixture")
            for uri in ("package://other_description/outside.stl", "package://demo_description/../outside.stl"):
                model = ET.fromstring('<robot><mesh filename="' + uri + '"/></robot>')
                with self.subTest(uri=uri), self.assertRaises(ConfigError):
                    mesh_dependencies(package, model)
            (package / "alias.stl").symlink_to(outside)
            with self.assertRaises(ConfigError):
                mesh_dependencies(package, ET.fromstring('<robot><mesh filename="package://demo_description/alias.stl"/></robot>'))

    def test_all_packages_have_complete_geometry_controller_and_texture_dependencies(self):
        report = check_packages(ROOT, render=False)
        self.assertTrue(report["ok"], report)
        self.assertEqual(len(report["packages"]), len(list((ROOT / "robots").glob("*/*_description"))))

    @unittest.skipUnless(Path("/opt/ros/noetic/bin/xacro").is_file(), "Requires ROS Noetic xacro")
    def test_all_packages_expand_with_only_supported_plugins_and_valid_references(self):
        report = check_packages(ROOT, xacro="/opt/ros/noetic/bin/xacro")
        self.assertTrue(report["ok"], report)
        self.assertTrue(all(item["render_checked"] for item in report["packages"]))


if __name__ == "__main__":
    unittest.main()
