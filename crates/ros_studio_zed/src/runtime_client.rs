use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    process::Stdio,
};

use anyhow::{Context as _, Result, bail};
use async_channel::{Receiver, Sender};
use async_process::Command;
use futures::{
    AsyncWrite, StreamExt as _,
    future::{self, Either},
    io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader},
    pin_mut,
};
use ros_studio_protocol::{
    Event, EventMessage, GraphPatch, Request, RequestMessage, ResponseMessage, ResponseResult,
};

pub enum RuntimeCommand {
    Launch {
        command: Vec<String>,
        env: BTreeMap<String, String>,
    },
    StopProcess {
        id: String,
    },
}

pub enum RuntimeMessage {
    GraphPatch(GraphPatch),
    Diagnostic(String),
    ProcessStarted(String),
    ProcessOutput {
        id: String,
        text: String,
    },
    ProcessExited {
        id: String,
        success: bool,
        code: Option<i32>,
    },
    RequestFailed(String),
}

pub async fn connect(
    workspace_root: PathBuf,
    sender: Sender<RuntimeMessage>,
    commands: Receiver<RuntimeCommand>,
) -> Result<()> {
    let (mut command, daemon_workspace_root) = daemon_command(&workspace_root)?;
    let mut child = command.spawn().context("failed to start ROS 2 daemon")?;
    let mut stdin = child.stdin.take().context("ROS 2 daemon has no stdin")?;
    let stdout = child.stdout.take().context("ROS 2 daemon has no stdout")?;

    let writer = async move {
        let mut request_id = 1;
        for request in [
            Request::OpenWorkspace {
                path: daemon_workspace_root,
            },
            Request::StartRuntimeDiscovery {},
        ] {
            write_request(&mut stdin, RequestMessage::new(request_id, request)).await?;
            request_id += 1;
        }

        while let Ok(command) = commands.recv().await {
            let request = match command {
                RuntimeCommand::Launch { command, env } => Request::Launch { command, env },
                RuntimeCommand::StopProcess { id } => Request::StopProcess { id },
            };
            write_request(&mut stdin, RequestMessage::new(request_id, request)).await?;
            request_id += 1;
        }

        Result::<()>::Ok(())
    };

    let reader = async {
        let mut lines = BufReader::new(stdout).lines();
        while let Some(line) = lines.next().await {
            let line = line.context("failed to read ROS 2 daemon output")?;
            handle_daemon_line(&line, &sender).await?;
        }
        Result::<()>::Ok(())
    };

    pin_mut!(writer, reader);
    match future::select(reader, writer).await {
        Either::Left((result, _)) | Either::Right((result, _)) => result?,
    }

    let status = child.status().await?;
    bail!("ROS 2 daemon disconnected ({status})")
}

async fn write_request(
    writer: &mut (impl AsyncWrite + Unpin),
    request: RequestMessage,
) -> Result<()> {
    let request = serde_json::to_vec(&request)?;
    writer.write_all(&request).await?;
    writer.write_all(b"\n").await?;
    writer.flush().await?;
    Ok(())
}

async fn handle_daemon_line(line: &str, sender: &Sender<RuntimeMessage>) -> Result<()> {
    let value: serde_json::Value =
        serde_json::from_str(line).context("ROS 2 daemon sent invalid JSON")?;

    if value.get("method").is_some() {
        let message: EventMessage = serde_json::from_value(value)?;
        message.validate()?;
        let message = match message.event {
            Event::RuntimeGraphChanged { patch } => RuntimeMessage::GraphPatch(patch),
            Event::Diagnostic { message, .. } => RuntimeMessage::Diagnostic(message),
            Event::ProcessOutput {
                id,
                stream: _,
                text,
            } => RuntimeMessage::ProcessOutput { id, text },
            Event::ProcessExited { id, success, code } => {
                RuntimeMessage::ProcessExited { id, success, code }
            }
            Event::StaticGraphChanged { .. } => return Ok(()),
        };
        sender.send(message).await?;
    } else {
        let response: ResponseMessage = serde_json::from_value(value)?;
        response.validate()?;
        if let Some(error) = response.error {
            sender
                .send(RuntimeMessage::RequestFailed(error.message))
                .await?;
        } else if let Some(ResponseResult::ProcessStarted { id }) = response.result {
            sender.send(RuntimeMessage::ProcessStarted(id)).await?;
        }
    }

    Ok(())
}

fn daemon_command(workspace_root: &Path) -> Result<(Command, String)> {
    if let Some(path) = std::env::var_os("ROS_STUDIO_DAEMON_PATH") {
        let mut command = Command::new(path);
        command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true);
        return Ok((command, workspace_root.to_string_lossy().into_owned()));
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
    let workspace_name = workspace_root
        .file_name()
        .and_then(|name| name.to_str())
        .context("open workspace has no valid UTF-8 directory name")?;
    let daemon_workspace_root = format!("/workspace/{workspace_name}");
    let mut command = Command::new("docker");
    command
        .args(["run", "--rm", "-i", "--network", "host", "--ipc", "host"])
        .arg("--mount")
        .arg(format!(
            "type=bind,src={},dst=/workspace/zed-fork,readonly",
            repository_root.display()
        ))
        .arg("--mount")
        .arg(format!(
            "type=bind,src={},dst={daemon_workspace_root}",
            workspace_root.display(),
        ))
        .arg("--workdir")
        .arg(&daemon_workspace_root);
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

    Ok((command, daemon_workspace_root))
}
