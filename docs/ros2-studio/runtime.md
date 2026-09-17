# ROS 2 runtime discovery (M10)

The default `ros-studio-daemon` remains usable without ROS 2. The `ros-runtime` feature builds the same daemon with `rclrs 0.7` and must be compiled and run in a sourced ROS 2 environment. The host Zed process does not link to ROS libraries.

`open_workspace` selects the project ID. `start_runtime_discovery` creates a private introspection node and queries node names, topic names/types, and publisher/subscription endpoints once per second. The introspection node is omitted from results. The daemon emits a `runtime_graph_changed` notification only when nodes or endpoints change. `stop_runtime_discovery` removes the runtime entities from the stream. Query failures are emitted as `diagnostic` notifications; they are not silently ignored.

The project ID is `project:<workspace directory name>`, matching the static scanner. Runtime-only nodes use stable IDs based on their fully qualified ROS name. Their package and executable are unknown until Design/Live reconciliation in M11, so they are grouped under a synthetic `Runtime` package. Duplicate ROS nodes with the same fully qualified name cannot currently be distinguished by the graph API used here.

On this development machine, ROS 2 lives in the Rust-Drone Humble Docker image, not in `/opt/ros` on the host. From the ROS 2 Studio repository root, start an interactive container with the repository mounted:

```sh
docker run --rm -it --network host --ipc host \
  --mount "type=bind,src=$PWD,dst=/workspace/zed-fork" \
  --workdir /workspace/zed-fork \
  rust-drone-ros2-humble:local bash
```

Inside the container, source ROS and its Rust message overlay, install the Zed repository's pinned Rust toolchain if necessary, then build the feature:

```sh
. /opt/ros/humble/setup.bash
. /opt/ros2-rust-overlay/install/setup.bash
rustup toolchain install 1.98.1 --profile minimal
CARGO_BUILD_JOBS=1 CARGO_TARGET_DIR=target/ros-humble cargo +1.98.1 test -j 1 -p ros_studio_daemon --features ros-runtime
CARGO_BUILD_JOBS=1 CARGO_TARGET_DIR=target/ros-humble cargo +1.98.1 run -j 1 -p ros_studio_daemon --features ros-runtime --bin ros-studio-daemon
```

The daemon accepts one JSON request per line. Enter these in the same terminal after Cargo prints `Running target/ros-humble/debug/ros-studio-daemon`:

```json
{"jsonrpc":"2.0","protocol_version":1,"id":1,"method":"open_workspace","params":{"path":"/workspace/Rust-Drone"}}
{"jsonrpc":"2.0","protocol_version":1,"id":2,"method":"start_runtime_discovery","params":{}}
```

The daemon replies to both requests and then emits `runtime_graph_changed` events as ROS nodes appear or disappear. Send `stop_runtime_discovery` to stop the worker. M10 does not yet display these events in Zed; that Design/Live integration is M11.
