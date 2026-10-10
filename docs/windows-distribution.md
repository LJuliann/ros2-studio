# Windows portable distribution

The first Windows archive is an **inspection prerelease**, intended for public
testing on Windows 11 x64. It includes the optimized native app, Visual C++
runtime DLLs, third-party notices, and portable launchers. Rust, Visual Studio,
ROS and WSL are not needed to inspect a project.

Extract the whole ZIP to a writable folder and open **Start ROS 2 Studio.cmd**.
Preferences are stored in `data/preferences.json` in that folder. The executable
is in `bin/ros2-studio.exe`; running it directly uses normal Windows preferences.
Do not move the executable away from its adjacent DLLs.

## Generate the ZIP on GitHub

1. Push `codex/windows-native` using **Push origin** in GitHub Desktop.
2. Open the repository's **Actions → Windows native** run for that commit.
3. Wait for the checks to pass. Download the `ros2-studio-windows-x64-inspection-<commit>` artifact.
4. Inside the downloaded artifact are the portable ZIP and its `.sha256` file.

Artifacts expire after 30 days. For a durable public download, create a GitHub
Release after checking the package on another Windows machine:

1. Go to **Releases → Draft a new release**.
2. Create tag `v0.2.0-alpha.3` targeting the checked commit/branch. Do not target
   an older `main` commit if the Windows changes have not been merged there.
3. Name it **ROS 2 Studio 0.2.0-alpha.3 — Windows inspection preview**, select
   **Set as a pre-release**, and attach both the ZIP and checksum.
4. State that Live/Build/Run/Stop via Docker are not included in this inspection
   archive, and that only Windows 11 x64 on a GTX 1080 has been smoke-tested.
5. Review the files and notes, then publish the prerelease. Do not commit binary
   ZIPs into the source repository.

The local packager can also produce the files to attach; a GitHub-generated
build gives users an artifact tied to a checked source commit. CI compilation
and `--help` do not validate GUI performance or the final package on a clean PC.

## Build/package locally

From the repository root, with the declared Rust toolchain, Windows C++ Build
Tools/SDK, Python 3.9+, and cargo-about 0.8.2:

```powershell
cargo build --locked --profile release-fast -j 2 -p ros_studio_app --bin ros2-studio
New-Item -ItemType Directory -Force target/windows-notices
cargo about generate --locked --fail --target x86_64-pc-windows-msvc --manifest-path crates/ros_studio_app/Cargo.toml --config script/licenses/zed-licenses.toml --output-file target/windows-notices/THIRD-PARTY-LICENSES-WINDOWS.md script/licenses/template.md.hbs
python script/ros2-studio-standalone windows-portable 0.2.0-alpha.3 --profile release-fast --licenses target/windows-notices/THIRD-PARTY-LICENSES-WINDOWS.md
```

The packager finds Visual Studio's x64 `Microsoft.VC143.CRT` redistributable
directory, or accepts it via `--crt-dir`. It refuses `target/debug`, wrong
architectures, missing notices, and existing output files. It includes no PDB,
SDK, shader source, project source, or development toolchain. The full `release`
profile is used on GitHub; `release-fast` also optimizes Rust and embeds optimized
shaders but skips link-time optimization for faster local builds.

## Validate public downloads and fluidity

An optimized executable avoids the unoptimized Rust/DirectX development build.
It cannot guarantee fluidity on unknown GPUs, drivers, display resolutions or
Windows configurations. Before announcing broader compatibility, test the ZIP
on a machine without development tools and, if possible, an integrated GPU:

- Extract in a folder with spaces, launch without administrator rights, open a
  ROS project, then close/reopen and check portable preferences.
- With a real graph loaded, drag a node, pan, zoom, resize the window and use
  menus. Compare at the same resolution/scaling; a blank window is insufficient.
- Verify editor opening and source refresh. Test node creation on a disposable
  project copy rather than modifying a user's original project.
- If startup fails or interactions stutter, use **Diagnostics.cmd**. After
  closing the app, `data/diagnostics.log` contains build information and the
  selected GPU/software-rendering status. Include Windows version, display
  resolution/scaling, approximate graph size and the slow action in a report.

No FPS/latency comparison on other hardware has been collected yet. If the
optimized package still stutters, measure rendering and input handling before
claiming a fix or changing the graphics backend.

## Package Docker runtime support later

Live/Build/Run/Stop require a **Linux x86_64** daemon compiled with `ros-runtime`
and a compatible Docker ROS Humble environment. A native dummy daemon `.exe`
does not provide those features. End-to-end Windows Docker validation is still
pending; the inspection archive must not be described as including them.

After building and testing that daemon, the same packager accepts
`--daemon-binary target/ros-humble/release/ros-studio-daemon` and
`--daemon-licenses <generated-Linux-daemon-notices.md>`. It creates a separately
named `docker-preview` ZIP with `libexec/ros-studio-daemon`, discoverable next to
the packaged `bin` folder. Docker Desktop and the project's ROS image remain
external requirements; they are not embedded in the portable application.

References: [GitHub workflow artifacts](https://docs.github.com/en/actions/concepts/workflows-and-actions/workflow-artifacts),
[Visual C++ redistribution](https://learn.microsoft.com/en-us/cpp/windows/redistributing-visual-cpp-files).
