# Windows validation — 2026-10-09

Baseline: `0cf47b8` (standalone alpha.2), plus the Windows changes on this branch.
Checked on Windows 11 x64 with Rust 1.98.1, MSVC Build Tools 2022, and Windows SDK
10.0.26100. The application is native Windows. Docker is the ROS runtime;
ROS 2 Studio does not launch WSL or require an Ubuntu distribution.

## Completed

- Core Rust suite: 146 passing tests across the app, scanner, generator, model,
  protocol, and daemon. Two real ROS/Docker integration tests remain opt-in.
- `cargo clippy -p ros_studio_app --no-default-features --all-targets --no-deps
  -- -D warnings`: passed.
- GUI binary Clippy (`--bin ros2-studio --no-deps -- -D warnings`): passed
  after the startup correction.
- Native development executable: built successfully; `ros2-studio.exe --help`
  starts successfully.
- Native GUI startup with ROS2-Drone: the window opens and remains responsive
  on the NVIDIA GeForce GTX 1080, using Direct3D 11.1 hardware acceleration.
  Startup exposed duplicate hover-style registration on toolbar buttons; all
  button variants now register their hover style once. The absent optional DXGI
  debug interface is logged at debug level instead of as a startup error.
- Optimized Windows `release-fast` build: compiled successfully, including the
  embedded optimized DirectX shaders. `--help` passes and the native window
  opens with ROS2-Drone on the GTX 1080. Dragging/zooming performance still needs
  a user comparison with the development build; no FPS or latency measurement
  has been collected. Windows instructions now use this profile for testing.
- All 18 DirectX release shaders compile with the Windows SDK's `fxc`.
  The vendored GPUI shaders still emit compiler warnings.
- Static inspection of the local Dronolab/ROS2-Drone checkout: 9 packages,
  10 nodes, 3 links, 4 Rust generator targets. All discovered nodes resolve a
  build command with Linux-compatible relative paths. No node was executed.
- Docker Compose parses that project's ROS service configuration on Windows.
- Windows daemon build script: PowerShell syntax checked.
- Windows portable inspection ZIP: generated with the optimized `release-fast`
  executable, x64 Visual C++ release DLLs, Windows-target notices, portable
  launchers and SHA-256 checksum. Extracted into a folder containing spaces;
  `--help` succeeds, the native window responds with ROS2-Drone, the loaded CRT
  DLL comes from the extracted package, and preferences are stored in its local
  `data` directory. Package size is approximately 6.3 MiB.
- Packaging/exporter tests: 7 passing, 2 skipped on this host. Wrong PE platform,
  debug paths, absent daemon notices, ZIP contents and overwrite protection are
  covered. The GitHub workflow now uploads the inspection ZIP after its checks;
  that remote workflow has not been run for these packaging changes yet.

## Remaining before a Windows runtime release

- Run the two opt-in integration tests with a ROS-enabled Linux daemon and a
  working Docker Desktop Linux engine, including discovery, health rate/silence,
  streamed output, Stop, and disconnect cleanup.
- Exercise rendering, pan/zoom, native dialogs, editor opening, project restore,
  and source watching on a Windows desktop.
- Run the GitHub workflow's optimized release build and startup check.
- Test the portable inspection ZIP on a clean Windows machine and other GPUs.
  For runtime support, package the native executable with a tested Linux daemon
  and corresponding notices, then test that full package on a clean machine.

Unix host Make execution remains unavailable on Windows. The Make panel explains
this and disables execution; use node/file configurations with Docker instead.

## Daemon packaging correction - 2026-10-17

The alpha.3 inspection archive did not include a ROS daemon. The corrected
default workflow is intended to produce an alpha.4 `docker-preview` archive:
Linux compiles the `ros-runtime` daemon and runs both real ROS integration tests;
Windows downloads that exact binary, notices and build information and includes
them in the ZIP. Packaging now rejects missing runtime components, mismatched
source commits/checksums and incomplete smoke-test records. An inspection-only
archive requires an explicit `--inspection-only` option.

Local verification: 7 packaging/exporter tests passed (2 platform/privilege skips)
and all 6 runtime environment tests passed, including resolution of the daemon
in `libexec` next to a portable `bin/ros2-studio.exe`. Workflow dependencies and
packaging arguments were checked; the build script syntax is also checked.

The local Docker engine is not running, so the new daemon build and real ROS
tests have not been executed here. The changed GitHub workflow must pass after
pushing before a corrected runtime ZIP is available. Windows-to-Docker execution
and the extracted runtime package still need testing on a separate Windows PC.
