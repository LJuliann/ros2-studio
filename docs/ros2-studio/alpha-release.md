# ROS 2 Studio alpha release

The Linux alpha contains the editor, command-line launcher, and ROS 2 Studio daemon. The daemon runs inside a ROS Humble Docker image so the host editor does not link directly to ROS libraries.

## Requirements

- Linux on x86-64 or AArch64.
- Docker Engine with permission to run containers.
- A ROS Humble development image containing `/opt/ros/humble`, `/opt/ros2-rust-overlay`, Cargo, colcon, and the toolchains needed by the opened project.

ROS 2 Studio selects the runtime automatically. An already sourced ROS Humble host is preferred. Otherwise, a project `compose.yaml`, `compose.yml`, `docker-compose.yaml`, or `docker-compose.yml` is detected and a service named `ros2` is selected. A service whose name contains `ros`, or the only service in the file, is also accepted. Compose builds a service with a `build` section before starting the daemon. When neither a host installation nor Compose is available, the default image is `rust-drone-ros2-humble:local`.

Set `ROS_STUDIO_RUNTIME=host` or `ROS_STUDIO_RUNTIME=docker` to override automatic selection. `ROS_STUDIO_COMPOSE_SERVICE` selects a specific Compose service and `ROS_STUDIO_DOCKER_IMAGE` selects a different fallback image.

## Build the release

First build the daemon from the repository root inside the ROS Humble image:

```bash
CARGO_BUILD_JOBS=1 ./script/build-ros2-studio-daemon-linux
```

Then build the editor bundle on the Linux host:

```bash
./script/bundle-ros2-studio-linux
```

The script produces these release assets:

```text
target/release/ros2-studio-linux-<architecture>.tar.gz
target/release/ros2-studio-linux-<architecture>.tar.gz.sha256
```

Set `ROS_STUDIO_DAEMON_BINARY` when the daemon was built at a different path.

## Run the archive

Verify and extract the archive:

```bash
sha256sum --check ros2-studio-linux-x86_64.tar.gz.sha256
tar -xzf ros2-studio-linux-x86_64.tar.gz
./ros2-studio.app/bin/ros2-studio
```

The bundled daemon is either launched in the detected ROS Humble host environment or mounted read-only into the selected runtime container. Only the opened project is mounted read-write, so Design, Live, Build, Run, and Stop no longer require a ROS 2 Studio source checkout on the user's machine.

This is an alpha build. The application still inherits some Zed development-channel identity, settings paths, and update behavior while ROS 2 Studio-specific branding and release infrastructure are being completed.
