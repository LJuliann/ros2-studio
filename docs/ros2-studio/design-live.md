# Design and Live graph (M11)

The ROS Graph starts in Design mode and scans the open Zed workspace. Choosing Live or Both starts runtime discovery on demand. Live shows observed nodes; Both overlays runtime evidence on the Design graph. The first runtime patch distinguishes an empty ROS graph from a connection that is still pending. Switching modes keeps the canvas positions, zoom, and selection.

ROS 2 Studio compares fully qualified ROS node names. A Design node named `camera` matches a runtime node named `/camera`; ambiguous names are not guessed. A Design node absent from a received runtime snapshot is Offline, while a runtime-only node remains visible in Live and Both. Equivalent Rust, Python, and ROS spellings of a fully qualified interface type do not create a conflict.

The UI labels an absent Design node `NOT SEEN` rather than implying its process was explicitly stopped. The topic count includes distinct published or subscribed topic names, even if a topic currently has no connection to another node.

ROS 2 Studio automatically selects a runtime for Live discovery. A sourced ROS Humble host is used directly. Otherwise, a project Docker Compose configuration is preferred, followed by `rust-drone-ros2-humble:local` as the development fallback image. Compose services named `ros2` are detected automatically and services with a `build` section are prepared before the daemon starts. The selected backend appears beside the graph connection status.

The packaged daemon is launched on the host or mounted read-only into the container and communicates with the editor over JSON lines. The ROS 2 Studio repository is not mounted or required. The opened project is mounted read-write so Build and Run can create normal project artifacts. Compose preparation output and launched-process output are streamed into the Build and Run Output panel. The panel follows new output while it is at the bottom, pauses when the user scrolls upward, and accepts `/clear` in its command field. `ROS_STUDIO_RUNTIME`, `ROS_STUDIO_COMPOSE_SERVICE`, `ROS_STUDIO_DOCKER_IMAGE`, and `ROS_STUDIO_DAEMON_PATH` provide advanced overrides. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, and `ROS_LOCALHOST_ONLY` are forwarded to Docker when set in the editor environment.

If Docker, the image, or the feature-built daemon is unavailable, the graph remains usable in Design mode and shows an error in Live or Both. This development launcher is not an installed-package discovery mechanism.
