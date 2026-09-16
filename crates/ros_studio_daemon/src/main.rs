#![forbid(unsafe_code)]

fn main() -> std::io::Result<()> {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();

    ros_studio_daemon::serve(stdin.lock(), stdout.lock())
}
