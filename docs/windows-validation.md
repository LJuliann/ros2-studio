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
- Native development executable: built successfully; `ros2-studio.exe --help`
  starts successfully.
- All 18 DirectX release shaders compile with the Windows SDK's `fxc`.
  The vendored GPUI shaders still emit compiler warnings.
- Static inspection of the local Dronolab/ROS2-Drone checkout: 9 packages,
  10 nodes, 3 links, 4 Rust generator targets. All discovered nodes resolve a
  build command with Linux-compatible relative paths. No node was executed.
- Docker Compose parses that project's ROS service configuration on Windows.
- Exporter tests: 5 passing; Linux package generation and privileged symlink
  creation skipped on this host.
- Windows daemon build script: PowerShell syntax checked.

## Remaining before a Windows runtime release

- Run the two opt-in integration tests with a ROS-enabled Linux daemon and a
  working Docker Desktop Linux engine, including discovery, health rate/silence,
  streamed output, Stop, and disconnect cleanup.
- Exercise rendering, pan/zoom, native dialogs, editor opening, project restore,
  and source watching on a Windows desktop.
- Run the GitHub workflow's optimized release build and startup check.
- Package the native executable with the Linux daemon and corresponding
  Windows/daemon third-party notices, then test that package on a clean machine.

Unix host Make execution remains unavailable on Windows. The Make panel explains
this and disables execution; use node/file configurations with Docker instead.
