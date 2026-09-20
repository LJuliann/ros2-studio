# Build and run

Select a Rust node in ROS Graph to enable **Build** and **Run**. Build runs `cargo check` for the selected package and binary; Run starts the same binary with `cargo run`. **Stop** terminates the complete process group. ROS 2 Studio sends argument vectors to the daemon and never constructs a shell command from project data.

**Auto mode** is enabled by default. It switches the graph to Live when Run is pressed and back to Design when Stop is pressed. Turning it off keeps the currently selected graph mode unchanged.

The process state and recent stdout/stderr appear in the global **ROS Build and Run Output** panel. It uses Zed's native bottom dock, so it remains available while editing code and uses the dock's smooth resize handle. **Close** hides it; the **Output** toolbar button or the dock button restores it without discarding output. **Build**, **Run**, and **Stop** appear at the top right for active Rust, C, C++, and Python source files.

The Build and Run menus distinguish the ROS node matching the active source from the source file itself. ROS node builds use Cargo for Rust and colcon for C++ or Python packages. Direct file builds use Cargo or rustc for Rust, the system C/C++ compiler, and Python bytecode compilation. The selected Run target is remembered per file. On Linux, **Shift+F10** runs that current configuration and **Alt+Shift+F10** opens the target menu, matching the JetBrains keymap. The output uses a read-only Zed editor, so it can be scrolled, selected, and copied. Starting another command is disabled while one is active.

The package, node, and topic counts below the graph title open their corresponding lists in the Inspector. Choosing a node from that list returns the Inspector to the node details view.

With the development Docker launcher, the open project is mounted read-write below `/workspace` under its original directory name while the Zed repository remains read-only at `/workspace/zed-fork`. This lets Cargo write project build artifacts without giving the launched process write access to the editor source tree.

For a daemon configured directly through `ROS_STUDIO_DAEMON_PATH`, commands run from the open workspace on the host and inherit the daemon environment.
