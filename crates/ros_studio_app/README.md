# ROS 2 Studio standalone

Open a ROS project, inspect its Design/Live graph, and open source files in your
preferred editor. This application uses GPUI for its window, but does not embed
the Zed editor or depend on `ros_studio_zed`, `editor`, `project`, `workspace`,
or `lsp`.

The source can run in the original Cargo workspace or be exported into an
independent workspace containing only the ROS crates and GPUI support crates.
Published editor-based alpha archives remain separate from this standalone build.

## Run

From the repository root:

```sh
CARGO_BUILD_JOBS=1 cargo run --profile release-fast -j 1 -p ros_studio_app -- --editor code examples/drone_demo_ws
```

Without a folder argument, the last successfully opened project is restored if
it is still accessible; otherwise the usual empty page offers **Open project**.
An explicit folder argument always takes precedence. Use **Editor** to choose
System default, VS Code, VSCodium, Zed, or CLion. The choice applies to the
saved preferences; `--editor` and `ROS_STUDIO_EDITOR` override the saved choice.

```sh
cargo run --profile release-fast -j 1 -p ros_studio_app -- --editor zed /path/to/ros/project
```

Editor launchers (`code`, `codium`, `zed`, or `clion`) must be available in your
`PATH`. Sources open at their one-based source line, column 1. The system-default
opener uses file associations and cannot specify a source line. No editor
settings or project files are modified by opening the graph.

The command formats follow the [VS Code CLI](https://code.visualstudio.com/docs/configure/command-line),
[Zed CLI](https://zed.dev/docs/reference/cli), and
[CLion CLI](https://www.jetbrains.com/help/clion/opening-files-from-command-line.html).
Windows launcher names are `code.cmd`, `codium.cmd`, `zed`, and `clion64.exe`;
The Windows application runs natively; ROS processes run in Linux Docker containers.

## Windows

The graph, source scanning, node creation, preferences, and external editor run
in a native Windows executable. Static inspection requires neither Docker nor ROS.
ROS 2 Studio does not invoke WSL or require an Ubuntu installation on the host.

For development, install the Rust toolchain declared in `rust-toolchain.toml`
and Visual Studio C++ Build Tools with a Windows SDK. From PowerShell:

```powershell
cargo run --locked --profile release-fast -j 2 -p ros_studio_app -- --editor code "C:\path\to\ROS2-Drone"
```

Use the optimized `release-fast` executable to assess interaction performance:
`target/release-fast/ros2-studio.exe`. The default development build in
`target/debug` uses unoptimized Rust code and DirectX shaders, which can make
graph dragging and zooming slower. Docker is not needed for this comparison.

Live, health monitoring, Build/Run, and Stop require Docker Desktop using Linux
containers and a Linux `ros-studio-daemon` built with `ros-runtime`. Choose Auto
or Docker; Host ROS is unavailable on Windows. Docker manages its own Linux
backend; its [Hyper-V backend](https://docs.docker.com/desktop/setup/install/windows-install/)
is an option on supported Windows editions when WSL is unwanted. Enable Docker
Desktop's [host networking](https://docs.docker.com/engine/network/drivers/host/)
for projects such as ROS2-Drone that use `network_mode: host`. Docker must be
running and `docker` available in the application's PATH.

Build the Linux daemon from PowerShell 7, using the project's ROS Compose image:

```powershell
./script/build-ros2-studio-daemon-windows.ps1 -ProjectPath "C:\path\to\ROS2-Drone"
```

This development step builds the project's ROS image when necessary and writes
`target/ros-humble/release/ros-studio-daemon`; it does not run drone nodes.
For another Compose service use `-ComposeService`; without Compose use `-Image`.
Windows packages with runtime support must include this **Linux** daemon next to
`ros2-studio.exe` or in `libexec/ros-studio-daemon`. A native dummy daemon `.exe`
cannot replace it. On Windows, `ROS_STUDIO_DAEMON_PATH` selects the Linux binary
to mount in Docker; on Linux it keeps its direct-launch behavior.

Make targets are discovered on Windows, but their host execution currently
requires Unix. Use node/file configurations with Docker for Windows Build/Run.
The project's Linux GUI tools and hardware forwarding need their own Docker
configuration. End-to-end Docker discovery, health monitoring, Build/Run/Stop,
and desktop interactions still require a Windows smoke test before a release.

The `Windows native` GitHub workflow tests the core and fixture, builds the native
release executable including DirectX shaders, and verifies `--help` startup.
To inspect a real project without opening a window or executing its code:

```powershell
cargo run --locked -j 2 -p ros_studio_app --no-default-features --example inspect_project -- "C:\path\to\ROS2-Drone"
```

If hover, pan, or zoom feels slow, launch with `--diagnostics` to print the
selected graphics adapter and driver in the terminal. `software rendering: true`
means the window is rendered on the CPU rather than an accelerated GPU. The
application warns about this even without diagnostics. Diagnostic logging does
not change the selected graphics adapter or presentation mode.

```sh
cargo run --profile release-fast -j 1 -p ros_studio_app -- --diagnostics --editor code examples/drone_demo_ws
```

## Features

- Reuse the existing Rust, Python, and C/C++ static scanner.
- Display nodes and publisher/subscriber links with matching topic and type.
- Pan the dotted canvas, zoom around the pointer, move nodes, and fit the graph.
- Initially place dependencies from left to right, group disconnected families
  separately, and reorder branches to reduce crossings. A simple fork with
  unequal paths into one terminal join uses a main row above its secondary
  branch and join to avoid an unnecessarily wide graph. Feedback loops share a
  column; dense graphs and long/feedback links can still have crossings. This
  placement does not run while dragging or reset manually moved nodes on refresh.
- Select a node to inspect its sources and interfaces.
- Use **Packages**, **Nodes**, and **Topics** to list project entities in the
  inspector. Interface details include confidence and their source location.
- Right-click the canvas for node creation, configurations, fit, and refresh.
- Double-click a node, or click a source in the inspector, to open the code.
- Automatically rescan after source changes, preserving positions for unchanged
  node IDs and preserving the canvas viewport.
- Show scan, file-watcher, and editor-launch errors in the window.
- Create Rust nodes in existing packages with **New node**: name validation,
  automatic/manual `rclrs` API selection, topic search, and optional publishers
  and subscribers using the existing generator.
- Preview the new source and the before/after `Cargo.toml`, then confirm creation.
  Refresh the graph and open the generated source in the selected editor.
- Connect the existing ROS daemon on Linux, using a Humble host or Docker/Compose.
- Switch between Design, Live, and Both, reconciling live nodes with source nodes.
- Build, Run, and Stop a configured node or source file; show stdout/stderr and environment
  preparation in a resizable bottom console with follow, copy, clear, close, and reopen.

The watcher ignores `.git`, `target`, `build`, `install`, and `log`. Scans run in
the background. **Refresh** is available if native file watching fails.

Static inspection requires no ROS installation or daemon. Completion,
diagnostics, and debugging remain the responsibility of the external editor.
Opening an editor does not automatically configure its ROS environment or
Docker/WSL integration.

## Create a node

1. Open a project and click **New node**.
2. Enter a name and choose a Rust package. Spaces become `_`; hyphens and other
   special characters are ignored. A name cannot start with a digit.
3. Keep **Auto** for `rclrs`, or choose a supported API when its version cannot
   be inferred. Typed interfaces currently require `rclrs` 0.7.
4. Optionally filter the project's typed topics and toggle **Subscribe** or
   **Publish**. Selected interfaces stay selected when filtered out.
5. Click **Preview changes**. Nothing is written at this point. The project is
   rescanned, so preview validation uses the latest source files.
6. Review the new `src/bin/<name>.rs` and any manifest changes, then click
   **Create and open source**. **Back** lets you change the draft; **Cancel** or
   Escape dismisses it without writes before creation begins.

The generator refuses existing sources, duplicate node names, incompatible APIs,
and manifests changed since preview. It does not replace other source files.
If creation fails, the error is shown in the dialog. If the external editor
cannot start, the node remains created and the launch error appears in the graph.
The app never automatically runs or compiles the generated code.

Generation currently supports Rust only. Python/C++ packages are still scanned,
but are not offered as generator targets. Compilation requires the appropriate
ROS environment and message dependencies; generating a file does not install ROS.

## Live, Build, Run, and Stop

Runtime connection is opt-in. Opening a folder scans source files without
starting Docker or ROS. **Connect**, **Live**, **Both**, or **Build/Run** starts
the runtime session. Docker preparation can build the selected Compose service;
progress and errors are shown in **Console**.

**Environment** cycles Auto / Host / Docker. Disconnect before changing it.
Auto prefers an explicitly sourced Humble host, then the project's Compose
ROS service, then an installed Humble host, then the configured Docker image.
Docker must already be installed and usable by your account. This preview
does not install ROS or create a ROS Docker image for an arbitrary project.

The app looks for `ros-studio-daemon` beside its executable, then the packaged
`libexec` location, then `target/ros-humble/release` or `debug` in development.
Use a daemon compiled with `ros-runtime`, not the dummy daemon. The existing
development build command is:

```sh
CARGO_BUILD_JOBS=1 ./script/build-ros2-studio-daemon-linux
```

Exporting sources does not copy build artifacts. Run this command once from
the new standalone repository before testing Live/Build/Run. It uses your
configured Docker image with ROS Humble and the Rust overlay. Auto detects the
execution environment; it does not compile this component. Published standalone
packages must include the prebuilt daemon, so users do not need this development
step.

Optional environment overrides:

| Variable | Purpose |
| --- | --- |
| `ROS_STUDIO_RUNTIME` | Initial Auto/Host/Docker preference (`auto`, `host`, `docker`). |
| `ROS_STUDIO_DAEMON_PATH` | Launch a custom daemon directly, inheriting your shell environment. |
| `ROS_STUDIO_DOCKER_IMAGE` | Fallback image; defaults to `rust-drone-ros2-humble:local`. |
| `ROS_STUDIO_COMPOSE_SERVICE` | ROS service in the project's Compose file; default selection prefers `ros2`, then a service with `ros` in its name. |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY` | Forwarded to Docker to match your ROS network. |
| `CARGO_BUILD_JOBS` | Node compilation parallelism; defaults to 1. |

Docker starts a temporary container with the project mounted read/write and
the daemon read-only. Bare Docker uses host network/IPC; Compose keeps the
service's network configuration. Only connect to a trusted image/service.
The daemon sources Humble, the optional Rust overlay, and `install/setup.bash`
before running commands. Container-relative manifest paths are kept separate
from the host paths used to open your editor.

The dark UI keeps project actions and **Build / Run / Stop** in the top bar.
Graph modes, runtime connection, Make/Console toggles, and zoom controls are
grouped in the second row. The configuration strip shows the current target
and clickable package/node/topic counts.

Opening a project uses a readable zoom (normally 90–115%), based on the actual
canvas in logical pixels so monitor scaling is not applied twice. Small graphs
are centered; large ones start at their top/left and remain accessible by panning.
**Fit graph** explicitly switches to an overview of the entire graph. Refreshing
source files does not reset the current zoom or pan. Topic labels and card spacing
scale with the cards.

Select a node in the graph or inspector, then **Build** or **Run**: the selected
node is the target, shown in the configuration label. No configuration chooser
is needed. The target is saved when launched. With no selected node, the last
chosen or restored configuration is reused; if neither exists, the chooser opens.
**Configuration…** lets you choose another ROS node, a Rust/Python/C/C++ file,
or a discovered Makefile target. Choosing a file or make target clears the node
selection so Build/Run uses that configuration. **Shift+F10** runs the displayed
target and **Alt+Shift+F10** opens
the chooser. **Use as run configuration** saves a node target without launching.

ROS node configurations:

- Rust: `cargo build` or `cargo run`, with the package manifest and binary target.
- Python/C/C++ ROS packages: `colcon build --packages-select`; Run builds first,
  sources the workspace installation, then uses `ros2 run`.
- Runtime-only nodes have no source target and cannot be built/run here.

File configurations:

- Rust uses the nearest Cargo manifest and matching binary target. A module or
  library can be checked but cannot be run as a binary. Files outside a Cargo
  package use `rustc`.
- Python uses `py_compile` for Build and `python3` for Run.
- C/C++ uses a syntax check for Build; Run compiles a temporary executable.
  A standalone file does not inherit a ROS package's colcon flags or dependencies.

Make panel (host tasks):

- Opening a project detects literal targets from the root `GNUmakefile`,
  `makefile`, or `Makefile` and their local literal includes. The **Make commands**
  sidebar opens automatically when targets are found. **Make (N)** closes or
  reopens it; the file watcher keeps the list current. Discovery only reads
  files; it never runs make, evaluates variables, or starts project commands.
- Commands are grouped into **package** sections, collapsed by default. Associations are
  inferred from literal ROS run/launch/build commands, Cargo manifest paths,
  package-owned included files, and topic/service/action owners in the static
  graph. Shared, ambiguous, or unknown commands stay in **Project / shared**.
  Search matches package names as well as commands/files and exposes matching
  commands even inside collapsed sections. Variables/macros are not evaluated.
- Search the list and click **Run** beside `ros-build`, `ros-run`, or any other
  target to execute it immediately. No run configuration is required, and a
  selected graph node cannot override the clicked target. This does not change
  your selected node or saved Build/Run configuration. The exact target runs;
  the app does not guess another target or automatically build before Run.
- The active command is highlighted; **Stop Make** interrupts it. Closing the
  panel only hides it, and the main **Stop** and **Console** buttons still work.
  Only one execution can be active, but its recipe can start multiple nodes.
- Targets remain available in **Configuration…** for the optional Build/Run and
  keyboard-shortcut workflow; both buttons execute the chosen Make target.
- Make executes on the **host**, from the project folder, using the inherited
  environment, without requiring the ROS daemon. This avoids nesting Docker when
  a recipe already uses Compose. Install make and any tools required by the recipe.
  Connect Live separately to discover nodes started by the recipe.
- Output and errors appear in the same console. **Stop** interrupts make and its
  foreground process group, with a two-second grace period before forced cleanup.
  Recipes that detach containers/processes (such as `up -d`) need their own stop
  target; the app does not automatically run `docker compose down`.
- Only run trusted targets: recipes are arbitrary project code and may control
  real hardware. Interactive recipes needing terminal input are not supported.
  Computed targets/includes, pattern rules, and custom recipe prefixes are not
  fully interpreted by this conservative discovery.

One execution is allowed at a time (a make recipe can launch several nodes).
**Stop** stops the current daemon process or host make task; it also cancels a
pending launch. Disconnect only closes the ROS session, not an independent host
make task. Changing projects or closing the view requests both sessions' cleanup,
not cleanup of unrelated ROS nodes or detached services.
Normal window closing waits for session cleanup. Forced termination or an
operating-system shutdown cannot guarantee that cleanup finishes.
Running a project executes its real code: test hardware-facing nodes only in a
safe environment. The app does not sandbox your node's behavior.

**Auto mode** defaults on: a started Run switches to Live when discovery is
available; stopping or finishing that Run returns to Design. Turn it off to
keep your selected mode. Both retains static nodes and marks nodes not found
in the live snapshot as Offline. Live may correctly be empty if nothing runs
in the current ROS domain.

**Health** opens the monitoring inspector. Choose an available topic, enter
its expected Hz and a silence timeout, then **Save and monitor**. Settings are
stored per project in application preferences, not in your source files. Saving
starts a runtime connection if needed; saved watches resume on the next connection.
Only explicitly selected topics are subscribed to (maximum 32).

- **RATE OK**: received rate is within ±20% of the configured expectation.
- **RATE WARNING**: received rate is too low or too high.
- **NO DATA**: no message arrived before the configured silence deadline.
- **WAITING / UNKNOWN / UNAVAILABLE**: warming up, disconnected/stale observer,
  unsupported daemon, or missing message introspection/type support. These are
  not reported as node crashes.

The observer uses monotonic arrival times, a five-second rolling sample window,
bounded memory, and best-effort/volatile subscriptions. It runs inside the same
ROS daemon/environment as discovery, on Host or Docker, and receives data without
publishing commands. This adds subscription/network/serialization load, especially
for images: monitor important topics only. QoS and system load can affect the
received rate; it is not necessarily the publisher's true rate or proof of correct
processing. Very slow topics require a timeout longer than their normal interval.

Live/Both cards and edges show monitoring evidence. **Node presence** also lists
planned nodes absent from discovery. A topic's aggregated rate is never attributed
to an individual node when several publishers share it, nor taken as proof that
a subscriber processes its inputs. Green observations expire if reports stop;
disconnecting or switching projects clears measurements. Closing the Health panel
only hides it; **Stop watching** removes the selected subscription.

This feature requires rebuilding **both the app and the ROS runtime daemon**.
The ROS-free unit tests exercise rates/thresholds, silence/recovery, bounded memory,
stale reports, publisher ambiguity, settings persistence and protocol compatibility.
Actual receipt of messages still needs validation in a sourced ROS environment.

Optional integration smoke test, after rebuilding the runtime daemon:

```sh
CARGO_BUILD_JOBS=1 cargo test -j 1 -p ros_studio_app --no-default-features --lib \
  real_daemon_measures_topic_rate_then_silence_and_removes_the_watch -- --ignored --nocapture
```

It creates only a dedicated `/ros_studio_health_smoke_<pid>` String publisher
at 20 Hz, checks reception and the silence alarm, then removes the watch and
stops the test process/session. It does not start drone nodes or send commands
to hardware. If using namespaces/remaps, choose the actual discovered topic;
unresolved source-only message types are not offered for monitoring.

The console is output-only, not an interactive terminal. It retains the latest
5,000 lines, renders only visible rows, distinguishes stderr, and preserves
partial output chunks. Drag or Shift-click to select entire lines, then Ctrl+C;
Ctrl+A selects all lines. **Copy all**, **Clear**, and **Follow** are available.
Scrolling up or selecting lines pauses follow; enabling Follow resumes it immediately
and keeps it on as output arrives or the console is resized. Drag the divider to resize;
**Close** only hides output (it does not stop the process); **Console** reopens it.

Runtime uses a compatible ROS Humble environment: a Linux host or Linux Docker
containers from Linux/Windows. Windows Docker integration remains a preview
pending an end-to-end smoke test; arbitrary ROS distributions are not validated.

## Preferences

Editor, environment preference, Auto mode, console height, the last opened project, and each project's
run configuration are saved in `~/.config/ros2-studio/preferences.json` on Linux
(or under `XDG_CONFIG_HOME`). `ROS_STUDIO_SETTINGS_PATH` overrides the path.
Windows uses `%APPDATA%`; macOS uses `~/Library/Application Support`.
Preferences are replaced atomically. A malformed or unsupported file is reported
and left untouched; the application uses session-only defaults until corrected.
No external editor configuration is changed. Projects are not automatically
executed or connected when restoring preferences.
The last project is remembered only after a successful scan, using its absolute
canonical path. A missing, moved, or inaccessible folder (including a failed
startup scan) falls back to the empty page. Its saved path is retained so a
temporarily disconnected drive can be reopened on a later launch. Older
preferences without this field keep the empty startup page.

## Independent source workspace

From the original fork, export to a **new, nonexistent** folder:

```sh
python3 script/ros2-studio-standalone export ../ros2-studio-standalone
cd ../ros2-studio-standalone
cargo check -j 1 -p ros_studio_app
cargo run --profile release-fast -j 1 -p ros_studio_app -- --editor code examples/drone_demo_ws
```

The export keeps the application, model, scanner, generator, protocol, daemon,
their local dependency closure, licenses, and tracked test fixture. It excludes
the Zed editor and its project/workspace/language-server crates, WebRTC, build
artifacts, Git history, and user-generated fixture nodes. The original fork is
not modified or deleted. Existing destination folders are refused.

GPUI and its supporting source are still vendored from Zed Industries, with
their licenses and attribution; independent does not mean dependency-free.
The exporter verifies that the application's runtime graph contains no editor
crates. The new workspace defaults to the standalone application and one build
worker. See Windows above for the native application and Docker runtime setup.

## Standalone Linux packages

The new packager does **not** call Zed's bundle scripts, build WebRTC, or include
`zed-editor` or the remote server. Existing legacy packaging scripts are kept
for the published editor-based alpha.

First build the standalone application and a ROS-enabled daemon:

```sh
CARGO_BUILD_JOBS=1 cargo build --release -j 1 -p ros_studio_app --bin ros2-studio
```

Build the daemon in a compatible Humble environment using the command above
or, in a sourced ROS/Rust-overlay shell:

```sh
CARGO_TARGET_DIR=target/ros-humble cargo build --release -j 1 -p ros_studio_daemon --features ros-runtime --bin ros-studio-daemon
```

Generate the third-party notices with `cargo-about` for the standalone app and
ROS-enabled daemon manifests, using `script/licenses/zed-licenses.toml` and
`script/licenses/template.md.hbs`, then combine the notices. The daemon license
scan also needs its ROS environment and `--features ros-runtime`.

Package the **prebuilt** binaries and notices (no compilation or installation):

```sh
python3 script/ros2-studio-standalone bundle 0.2.0-alpha.1 --licenses THIRD-PARTY-LICENSES.md
python3 script/ros2-studio-standalone deb 0.2.0-alpha.1 --licenses THIRD-PARTY-LICENSES.md
```

Use `--app-binary`, `--daemon-binary`, and `--output` to select paths. Packaging
requires Python 3.9+, `readelf`, `strip`, and for `.deb`, `dpkg-deb`. It checks Linux ELF
architecture and symbol-version requirements, includes build information, and
writes a matching SHA-256 file. Existing artifacts are never overwritten.
Debug sections are stripped from the packaged copies, not from the original
binaries. Use `--keep-debug` when preserving them for a debugging build.

For Ubuntu 22.04, compile **both** binaries on that baseline and pass
`--glibc-max 2.35`. A binary compiled on 24.04 does not become 22.04-compatible
by changing its filename. The GLIBC check is a compatibility guard, not a
replacement for testing graphics drivers, ROS libraries, and the full package
on the oldest supported distribution.

An archive contains `ros2-studio.app/bin/ros2-studio` and
`ros2-studio.app/libexec/ros-studio-daemon`. Extract it and launch the former,
or install the `.deb` with your package manager. ROS Humble and the project's
message packages remain environment requirements for Live/Build/Run.

## Validation still required before a release

- Exercise GUI, external-editor opening, and persistence on the user's desktop.
- Exercise actual Live/Build/Run/Stop on a compatible host and trusted container.
- Smoke-test the final binaries and installer on supported distributions.

## Check

Core tests do not compile GPUI or require a display:

```sh
cargo test --offline -j 1 -p ros_studio_app --no-default-features --lib
cargo test --offline -j 1 -p ros_studio_codegen --lib
```

Check the GUI and inspect its dependencies:

```sh
cargo check --offline -j 1 -p ros_studio_app
cargo tree --offline -p ros_studio_app --edges normal
```

`--offline` assumes dependencies are already downloaded; omit it on a fresh
checkout. For linting, use `./script/clippy -p ros_studio_app`.

For manual validation, open `examples/drone_demo_ws`, move a node, switch to the
external editor and save a source file, and verify that its position remains
unchanged. Check that double-click opens the right file and line. Choose a
missing editor launcher to verify the visible error, then select an installed
editor and try again.

Verify the Packages/Nodes/Topics lists, interface-source links, right-click menu,
zoom buttons, and both configuration shortcuts. Select a file configuration,
restart the app, and confirm it is restored but not automatically executed.
Resize the console and change the editor/Auto mode, then verify persistence.
Open a different project and verify it has its own configuration.

For wizard validation, use a disposable copy of a ROS project. Enter a name
such as `camera test!`, check that it becomes `camera_test`, search `cam img`,
select **Subscribe**, and review the preview. Cancel first and verify that no
file was created. Repeat and confirm creation: the node should appear connected
to the camera topic and its file should open in the selected editor. Preview
another node, modify `Cargo.toml` externally, then confirm: creation must fail
without overwriting your changes. Test long names, scrolling, copy/paste,
selection with Shift+arrows, Tab between fields, and Escape to cancel.

For runtime validation, start with a harmless node, not a flight/controller
node. Verify stdout/stderr, a failed build, Run → Live, Stop → Design, Auto mode
off, console scrolling/selection/resize, closing and reopening it, and
disconnect/reconnect. Confirm that changing projects clears the live overlay
and that static node positions survive mode changes.

The default tests use a fake daemon and do not need Docker or ROS. An explicit
Linux integration test uses your configured real daemon, discovers the graph,
then launches and stops only `printf` + `sleep` (no drone node):

```sh
ROS_STUDIO_RUNTIME=docker cargo test --offline -j 1 -p ros_studio_app --no-default-features --lib runtime_client::tests::real_daemon_discovers_streams_and_stops_a_harmless_process -- --ignored --nocapture
```

This opt-in test starts the runtime container with the mounts/network described
above; inspect the chosen Compose service or image before running it.
