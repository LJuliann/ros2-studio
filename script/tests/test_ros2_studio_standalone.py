import importlib.machinery
import importlib.util
from pathlib import Path
import tempfile
import tarfile
from types import SimpleNamespace
import unittest


loader = importlib.machinery.SourceFileLoader("standalone", str(Path(__file__).resolve().parents[1] / "ros2-studio-standalone"))
specification = importlib.util.spec_from_loader(loader.name, loader)
standalone = importlib.util.module_from_spec(specification)
loader.exec_module(standalone)


class StandaloneTests(unittest.TestCase):
    def test_manifest_has_only_selected_members_and_no_editor_profiles(self):
        document = '[workspace]\nmembers = ["crates/zed"]\n[workspace.dependencies]\nzed = { path = "crates/zed" }\ngpui = { path = "crates/gpui" }\nserde = "1"\n[patch.crates-io]\nwebrtc-sys = { git = "bad" }\nasync-process = { git = "good" }\n[profile.release.package]\nzed = { codegen-units = 16 }\n[profile.release-fast]\ninherits = "release"\n'
        exported = standalone.export_manifest(document, {"crates/gpui", "crates/ros_studio_app"})
        self.assertNotIn('"crates/zed"', exported)
        self.assertNotIn("webrtc-sys =", exported)
        self.assertNotIn("[profile.release.package]", exported)
        self.assertIn("async-process =", exported)
        self.assertIn('["crates/ros_studio_app"]', exported)

    def test_closure_includes_platform_and_dev_dependencies(self):
        def package(name, dependencies):
            return {"name": name, "manifest_path": f"/repo/crates/{name}/Cargo.toml", "dependencies": [{"path": f"/repo/crates/{dependency}"} for dependency in dependencies]}
        metadata = {"packages": [package("ros_studio_app", ["gpui"]), package("ros_studio_daemon", []), package("gpui", ["gpui_windows"]), package("gpui_windows", [])]}
        self.assertEqual(len(standalone.workspace_closure(metadata)), 4)
        metadata["packages"][2]["dependencies"].append({"path": "/repo/crates/editor"})
        metadata["packages"].append(package("editor", []))
        with self.assertRaisesRegex(ValueError, "Editor dependency leaked"):
            standalone.workspace_closure(metadata)

    def test_rejects_existing_export_destination_without_touching_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "keep.txt"
            source.write_text("keep")
            class Arguments:
                destination = root
            with self.assertRaisesRegex(ValueError, "existing folder"):
                standalone.export_workspace(Arguments())
            self.assertEqual(source.read_text(), "keep")

    def test_checksum_names_the_packaged_file(self):
        with tempfile.TemporaryDirectory() as directory:
            artifact = Path(directory) / "alpha.tar.gz"
            artifact.write_bytes(b"test")
            standalone.checksum(artifact)
            self.assertTrue(artifact.with_name(artifact.name + ".sha256").read_text().endswith("  alpha.tar.gz\n"))

    def test_external_symlink_is_not_exported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "source").mkdir()
            (root / "source/link").symlink_to(root / "outside")
            with self.assertRaisesRegex(ValueError, "External symbolic link"):
                standalone.copy_tree(root / "source", root / "destination")

    def test_selected_dependency_catalog_drops_unused_editor_dependencies(self):
        source = '[workspace]\nmembers=[]\n[workspace.dependencies]\nserde = { version="1", features=[\n "derive",\n] }\nlibwebrtc="1"\n[workspace.dependencies.windows]\nversion="1"\n'
        exported = standalone.export_manifest(source, {"crates/ros_studio_app"}, {"serde"})
        self.assertIn("derive", exported)
        self.assertNotIn("libwebrtc", exported)
        self.assertNotIn("[workspace.dependencies.windows]", exported)

    def test_archive_and_deb_contain_only_the_standalone_app_and_daemon(self):
        if not standalone.shutil.which("readelf") or not standalone.shutil.which("dpkg-deb"):
            self.skipTest("Linux packaging tools unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            notices = root / "notices.md"
            notices.write_text("Packaging fixture only, not a distributable application")
            # A tiny ELF fixture exercises packaging without compiling or running ROS.
            arguments = SimpleNamespace(version="0.2.0-alpha.1", app_binary=Path("/bin/true"), daemon_binary=Path("/bin/true"), output=root / "dist", glibc_max=None, licenses=notices, keep_debug=False)
            for command in ("bundle", "deb"):
                arguments.command = command
                standalone.package(arguments)
            archive_path = next(arguments.output.glob("*.tar.gz"))
            with tarfile.open(archive_path) as archive:
                names = archive.getnames()
                self.assertIn("ros2-studio.app/bin/ros2-studio", names)
                self.assertIn("ros2-studio.app/libexec/ros-studio-daemon", names)
                self.assertFalse(any("zed-editor" in name or "remote_server" in name for name in names))
                self.assertTrue(all(not name.startswith("/") and ".." not in Path(name).parts for name in names))
            deb = next(arguments.output.glob("*.deb"))
            contents = standalone.run(["dpkg-deb", "--contents", str(deb)], capture_output=True).stdout
            self.assertIn("/opt/ros2-studio/bin/ros2-studio", contents)
            self.assertNotIn("zed-editor", contents)
            self.assertTrue(deb.with_name(deb.name + ".sha256").is_file())
            with self.assertRaisesRegex(ValueError, "already exists"):
                standalone.package(arguments)
            arguments.version = "0.2.0-alpha.2"
            arguments.glibc_max = "0.0"
            with self.assertRaisesRegex(ValueError, "newer than requested"):
                standalone.package(arguments)


if __name__ == "__main__":
    unittest.main()
