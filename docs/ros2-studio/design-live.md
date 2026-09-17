# Design and Live graph (M11)

The ROS Graph starts in Design mode and scans the open Zed workspace. Choosing Live or Both starts runtime discovery on demand. Live shows observed nodes; Both overlays runtime evidence on the Design graph. The first runtime patch distinguishes an empty ROS graph from a connection that is still pending. Switching modes keeps the canvas positions, zoom, and selection.

ROS 2 Studio compares fully qualified ROS node names. A Design node named `camera` matches a runtime node named `/camera`; ambiguous names are not guessed. A Design node absent from a received runtime snapshot is Offline, while a runtime-only node remains visible in Live and Both. Equivalent Rust, Python, and ROS spellings of a fully qualified interface type do not create a conflict.

The UI labels an absent Design node `NOT SEEN` rather than implying its process was explicitly stopped. The topic count includes distinct published or subscribed topic names, even if a topic currently has no connection to another node.

On the current development setup, Zed runs on Ubuntu and ROS 2 runs in the `rust-drone-ros2-humble:local` Docker image. Build the `ros-studio-daemon` binary with `--features ros-runtime` in that image first, using `CARGO_TARGET_DIR=target/ros-humble`. The Live button launches a container with the repository mounted read-only and connects to the daemon over JSON lines. It does not start ROS nodes; start the nodes you want to observe separately. The Docker image can be changed with `ROS_STUDIO_DOCKER_IMAGE`. Alternatively, `ROS_STUDIO_DAEMON_PATH` runs an already configured daemon directly on the host. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_LOCALHOST_ONLY` are forwarded to Docker when set in Zed's environment.

If Docker, the image, or the feature-built daemon is unavailable, the graph remains usable in Design mode and shows an error in Live or Both. This development launcher is not an installed-package discovery mechanism.
