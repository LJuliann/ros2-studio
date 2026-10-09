#![forbid(unsafe_code)]

#[cfg(not(feature = "ros-runtime"))]
fn main() -> std::io::Result<()> {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();

    ros_studio_daemon::serve(stdin.lock(), stdout.lock())
}

#[cfg(feature = "ros-runtime")]
fn main() -> std::io::Result<()> {
    ros_studio_daemon::serve_ros_stdio()
}
