import importlib.machinery
import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import tarfile
import struct
from types import SimpleNamespace
import unittest
import zipfile


loader = importlib.machinery.SourceFileLoader("standalone", str(Path(__file__).resolve().parents[1] / "ros2-studio-standalone"))
specification = importlib.util.spec_from_loader(loader.name, loader)
standalone = importlib.util.module_from_spec(specification)
loader.exec_module(standalone)


class StandaloneTests(unittest.TestCase):
    @staticmethod
    def pe_fixture(path, machine=0x8664):
        header = bytearray(64)
        header[:2] = b"MZ"
        struct.pack_into("<I", header, 60, 64)
        path.write_bytes(header + b"PE\0\0" + struct.pack("<H", machine))

    def test_windows_package_includes_crt_and_portable_launchers_without_build_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            app = root / "ros2-studio.exe"
            self.pe_fixture(app)
            crt = root / "crt"
            crt.mkdir()
            self.pe_fixture(crt / "vcruntime140.dll")
            notices = root / "notices.md"
            notices.write_text("Packaging test fixture only")
            arguments = SimpleNamespace(version="0.2.0-alpha.3", profile="release", app_binary=app, crt_dir=crt, licenses=notices, daemon_binary=None, daemon_licenses=None, daemon_info=None, inspection_only=False, output=root / "dist")
            with self.assertRaisesRegex(ValueError, "requires --daemon-binary"):
                standalone.windows_package(arguments)
            self.assertFalse(arguments.output.exists())
            arguments.inspection_only = True
            standalone.windows_package(arguments)
            artifact = next(arguments.output.glob("*.zip"))
            with zipfile.ZipFile(artifact) as archive:
                names = archive.namelist()
                self.assertIn("ros2-studio/bin/ros2-studio.exe", names)
                self.assertIn("ros2-studio/bin/vcruntime140.dll", names)
                self.assertIn("ros2-studio/THIRD-PARTY-LICENSES-WINDOWS.md", names)
                self.assertFalse(any("libexec" in name or name.endswith((".pdb", ".hlsl")) for name in names))
                launch = archive.read("ros2-studio/Start ROS 2 Studio.cmd").decode()
                self.assertIn('"%~dp0bin\\ros2-studio.exe" %*', launch)
                self.assertIn("ROS_STUDIO_SETTINGS_PATH=%~dp0data\\preferences.json", launch)
                self.assertIn("INSPECTION", archive.read("ros2-studio/README.txt").decode())
                self.assertTrue(all(not name.startswith("/") and ".." not in Path(name).parts for name in names))
            self.assertTrue(artifact.with_name(artifact.name + ".sha256").is_file())
            with self.assertRaisesRegex(ValueError, "already exists"):
                standalone.windows_package(arguments)
            arguments.version = "0.2.0-alpha.4"
            daemon = root / "ros-studio-daemon"
            daemon.write_bytes(b"\x7fELF\x02\x01" + bytes(12) + b"\x3e\x00")
            arguments.daemon_binary = daemon
            with self.assertRaisesRegex(ValueError, "cannot be combined"):
                standalone.windows_package(arguments)
            arguments.inspection_only = False
            with self.assertRaisesRegex(ValueError, "daemon-licenses"):
                standalone.windows_package(arguments)
            arguments.daemon_licenses = notices
            with self.assertRaisesRegex(ValueError, "daemon-info"):
                standalone.windows_package(arguments)
            information = {
                "source_revision": standalone.run(["git", "rev-parse", "HEAD"], cwd=standalone.ROOT, capture_output=True).stdout.strip(),
                "target": "x86_64-unknown-linux-gnu", "ros_distro": "humble",
                "features": ["ros-runtime"], "binary_sha256": "incorrect",
                "validated_tests": ["discovery-output-stop", "health-rate-silence"],
            }
            arguments.daemon_info = root / "DAEMON-INFO.json"
            arguments.daemon_info.write_text(json.dumps(information))
            with self.assertRaisesRegex(ValueError, "checksum"):
                standalone.windows_package(arguments)
            information["binary_sha256"] = hashlib.sha256(daemon.read_bytes()).hexdigest()
            information["features"] = []
            arguments.daemon_info.write_text(json.dumps(information))
            with self.assertRaisesRegex(ValueError, "ros-runtime"):
                standalone.windows_package(arguments)
            information["features"] = ["ros-runtime"]
            information["source_revision"] = "old-source-revision"
            arguments.daemon_info.write_text(json.dumps(information))
            with self.assertRaisesRegex(ValueError, "same commit"):
                standalone.windows_package(arguments)
            information["source_revision"] = standalone.run(["git", "rev-parse", "HEAD"], cwd=standalone.ROOT, capture_output=True).stdout.strip()
            arguments.daemon_info.write_text(json.dumps(information))
            standalone.windows_package(arguments)
            with zipfile.ZipFile(next(arguments.output.glob("*docker-preview.zip"))) as archive:
                self.assertIn("ros2-studio/libexec/ros-studio-daemon", archive.namelist())
                self.assertEqual(archive.read("ros2-studio/libexec/ros-studio-daemon"), daemon.read_bytes())
                packaged_info = json.loads(archive.read("ros2-studio/BUILD-INFO.json"))
                self.assertEqual(packaged_info["runtime"], "docker-preview")
                self.assertEqual(packaged_info["daemon"], information)
                self.assertIn("ros2-studio/libexec/DAEMON-INFO.json", archive.namelist())

    def test_windows_package_rejects_linux_arm_and_debug_app_binaries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / "app.exe"
            binary.write_bytes(b"\x7fELF" + bytes(64))
            with self.assertRaisesRegex(ValueError, "Not a Windows PE"):
                standalone.windows_binary(binary)
            self.pe_fixture(binary, machine=0xAA64)
            with self.assertRaisesRegex(ValueError, "x86_64"):
                standalone.windows_binary(binary)
            arguments = SimpleNamespace(version="0.2.0-alpha.3", app_binary=root / "target/debug/app.exe", profile="release")
            with self.assertRaisesRegex(ValueError, "optimized"):
                standalone.windows_package(arguments)

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
            try:
                (root / "source/link").symlink_to(root / "outside")
            except OSError as error:
                if getattr(error, "winerror", None) == 1314:
                    self.skipTest("Windows symlink privilege unavailable")
                raise
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
