use std::{path::PathBuf, process::Stdio};

use anyhow::{Context as _, Result, bail};
use async_channel::Sender;
use async_process::Command;
use futures::{
    StreamExt as _,
    io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader},
};
use ros_studio_protocol::{
    Event, EventMessage, GraphPatch, Request, RequestMessage, ResponseMessage,
};

pub enum RuntimeMessage {
    GraphPatch(GraphPatch),
    Diagnostic(String),
}

pub async fn discover(workspace_root: PathBuf, sender: Sender<RuntimeMessage>) -> Result<()> {
    let mut command = daemon_command()?;
    let mut child = command.spawn().context("failed to start ROS 2 daemon")?;
    let mut stdin = child.stdin.take().context("ROS 2 daemon has no stdin")?;
    let stdout = child.stdout.take().context("ROS 2 daemon has no stdout")?;

    for request in [
        RequestMessage::new(
            1,
            Request::OpenWorkspace {
                path: workspace_root.to_string_lossy().into_owned(),
            },
        ),
        RequestMessage::new(2, Request::StartRuntimeDiscovery {}),
    ] {
        let request = serde_json::to_vec(&request)?;
        stdin.write_all(&request).await?;
        stdin.write_all(b"\n").await?;
    }
    stdin.flush().await?;

    let mut lines = BufReader::new(stdout).lines();
    while let Some(line) = lines.next().await {
        let line = line.context("failed to read ROS 2 daemon output")?;
        let value: serde_json::Value =
            serde_json::from_str(&line).context("ROS 2 daemon sent invalid JSON")?;

        if value.get("method").is_some() {
            let message: EventMessage = serde_json::from_value(value)?;
            message.validate()?;
            match message.event {
                Event::RuntimeGraphChanged { patch } => {
                    sender.send(RuntimeMessage::GraphPatch(patch)).await?;
                }
                Event::Diagnostic { message, .. } => {
                    sender.send(RuntimeMessage::Diagnostic(message)).await?;
                }
                Event::StaticGraphChanged { .. } | Event::ProcessOutput { .. } => {}
            }
        } else {
            let response: ResponseMessage = serde_json::from_value(value)?;
            response.validate()?;
            if let Some(error) = response.error {
                bail!("ROS 2 daemon rejected request: {}", error.message);
            }
        }
    }

    let status = child.status().await?;
    bail!("ROS 2 daemon disconnected ({status})")
}

fn daemon_command() -> Result<Command> {
    if let Some(path) = std::env::var_os("ROS_STUDIO_DAEMON_PATH") {
        let mut command = Command::new(path);
        command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true);
        return Ok(command);
    }

    let repository_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(std::path::Path::parent)
        .context("could not locate ROS 2 Studio repository")?
        .to_path_buf();
    let daemon_binary = repository_root.join("target/ros-humble/debug/ros-studio-daemon");
    if !daemon_binary.is_file() {
        bail!(
            "ROS 2 daemon is missing at {}; build it in the ROS Humble container first",
            daemon_binary.display()
        );
    }

    let image = std::env::var("ROS_STUDIO_DOCKER_IMAGE")
        .unwrap_or_else(|_| "rust-drone-ros2-humble:local".to_owned());
    let mut command = Command::new("docker");
    command
        .args(["run", "--rm", "-i", "--network", "host", "--ipc", "host"])
        .arg("--mount")
        .arg(format!(
            "type=bind,src={},dst=/workspace/zed-fork,readonly",
            repository_root.display()
        ))
        .args(["--workdir", "/workspace/zed-fork"]);
    for variable in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"] {
        if let Some(value) = std::env::var_os(variable) {
            command
                .arg("--env")
                .arg(format!("{variable}={}", value.to_string_lossy()));
        }
    }
    command
        .arg(image)
        .args([
            "bash",
            "-lc",
            ". /opt/ros/humble/setup.bash && . /opt/ros2-rust-overlay/install/setup.bash && exec /workspace/zed-fork/target/ros-humble/debug/ros-studio-daemon",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .kill_on_drop(true);

    Ok(command)
}
