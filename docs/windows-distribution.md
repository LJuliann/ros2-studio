# Windows portable distribution

The default Windows download is now a **Docker runtime preview**. The old
`0.2.0-alpha.3-...-inspection.zip` omitted the daemon and cannot provide Live,
Build, Run or Stop. Replace the entire old folder with the new archive; the
daemon belongs to the application distribution, not to a host ROS installation.

The corrected `0.2.0-alpha.4-...-docker-preview.zip` contains:

```text
ros2-studio/
  Start ROS 2 Studio.cmd
  Diagnostics.cmd
  bin/ros2-studio.exe
  bin/<Visual C++ release DLLs>
  libexec/ros-studio-daemon
  libexec/DAEMON-INFO.json
  THIRD-PARTY-LICENSES-WINDOWS.md
  THIRD-PARTY-LICENSES-DAEMON.md
  BUILD-INFO.json
```

`ros2-studio.exe` runs natively on Windows. `ros-studio-daemon` is a **Linux
x86_64 ELF binary built with `ros-runtime`**; the app finds it in `libexec`, mounts
it into the ROS project container, and runs it there. Do not replace it with a
Windows dummy daemon `.exe`. Docker Desktop using Linux containers and a
compatible ROS Humble project image remain external requirements. The app does
not invoke WSL or require an Ubuntu distribution on the user's PC.

## Build and download on GitHub

1. Push the changes on `codex/windows-native` with **Push origin** in GitHub Desktop.
2. Wait for all three **Windows native** jobs: `linux-core`, `linux-daemon`, `windows`.
3. Download **ros2-studio-windows-x64-docker-preview-<commit>** from the run's
   artifacts. Select this Windows artifact, not the intermediate Linux daemon.
4. Extract the outer artifact download. It contains the portable ZIP and `.sha256`.
5. Extract the entire portable ZIP into a writable folder and run
   **Start ROS 2 Studio.cmd**. Preferences stay in `data/preferences.json`.
6. Open the ROS project, choose **Env: Docker**, and connect. Test first with a
   harmless process or test node; do not start flight/controller nodes merely
   to check packaging. Unix host Make targets remain unavailable on Windows.

The Linux job builds the daemon from the **same checked-out commit** as Windows
using a ROS Humble/Jammy container. Its two opt-in integration tests exercise
real ROS discovery, streamed output, Stop, topic rate and silence. It then
generates daemon notices and build information with the binary checksum.

Windows waits for that job, downloads the tested component and packages both
binaries. Packaging fails if the daemon/notices/build information is missing,
the source commit or checksum differs, `ros-runtime` is absent, or the recorded
ROS checks are incomplete. There is no automatic fallback to inspection mode.
These Linux checks do not replace testing Windows-to-Docker transport on a
separate Windows PC. The changed workflow must run successfully before the
corrected ZIP exists on GitHub.

## Publish the corrected prerelease

Artifacts expire after 30 days. For a durable public download:

1. After a successful run and extracted-package test, draft a release named
   **ROS 2 Studio 0.2.0-alpha.4 - Windows Docker preview**.
2. Use a new tag such as `ros2-studio-v0.2.0-alpha.4`, targeting the checked
   commit. Do not target an older `main` if the fixes have not been merged.
3. Attach the new `docker-preview.zip` and its `.sha256`, and mark **Pre-release**.
4. State that the daemon is included; Docker and the project's ROS image are
   still required. State the actual Windows/GPU testing completed. Publish only
   after reviewing the files and notes. Do not rename/reupload the old
   `inspection.zip` as a runtime package.

## Local packaging

In a sourced Linux ROS Humble build environment, install the repository's Rust
toolchain and cargo-about 0.8.2, then run:

```sh
bash script/build-ros2-studio-daemon-release
```

This builds/tests the real daemon and produces all three files under
`target/ros-humble`: `release/ros-studio-daemon`,
`THIRD-PARTY-LICENSES-DAEMON.md`, `DAEMON-INFO.json`. It does not run drone nodes.
Use this in a Linux container/build machine; end users do not perform this build.

After transferring those files into the **same source commit** on the Windows
build machine, build the optimized app and generate Windows-target notices as
described in the README. Package with Python 3.9+:

```powershell
python script/ros2-studio-standalone windows-portable 0.2.0-alpha.4 --profile release-fast --licenses target/windows-notices/THIRD-PARTY-LICENSES-WINDOWS.md --daemon-binary target/ros-humble/release/ros-studio-daemon --daemon-licenses target/ros-humble/THIRD-PARTY-LICENSES-DAEMON.md --daemon-info target/ros-humble/DAEMON-INFO.json
```

The packager includes Visual Studio's x64 release CRT DLLs (or a directory given
with `--crt-dir`), refuses `target/debug` and existing output files, and includes
no compiler/SDK. An intentionally reduced package requires **--inspection-only**
and is clearly named `inspection`; it is not the default GitHub deliverable.

## Fluidity and support reports

The app and DirectX shaders are optimized. That does not guarantee fluidity on
unknown GPUs/drivers/resolutions. Windows 11 x64 with a GTX 1080 is the currently
smoke-tested desktop; do not claim wider validation without testing it.

On another PC without development tools, test the extracted ZIP in a folder with
spaces, node dragging on a loaded ROS graph, pan/zoom, menus, editor opening,
source refresh and saved preferences. Then test Docker connection, Live,
Build/Run/Stop, output and disconnect cleanup with a harmless node.

For failures, close the app and use **Diagnostics.cmd**. The file
`data/diagnostics.log` includes build/GPU information. Share it with the Windows
version, resolution/scaling, approximate graph size and the action that failed.
`software rendering: true` indicates CPU rendering.

References: [GitHub workflow artifacts](https://docs.github.com/en/actions/concepts/workflows-and-actions/workflow-artifacts),
[Visual C++ redistribution](https://learn.microsoft.com/en-us/cpp/windows/redistributing-visual-cpp-files).
