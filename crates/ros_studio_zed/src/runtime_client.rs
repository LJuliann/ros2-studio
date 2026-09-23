use std::{collections::BTreeMap, path::PathBuf};

use anyhow::{Context as _, Result, bail};
use async_channel::{Receiver, Sender};
use futures::{
    AsyncWrite, StreamExt as _,
    future::{self, Either},
    io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader},
    pin_mut,
};
use ros_studio_protocol::{
    Event, EventMessage, GraphPatch, Request, RequestMessage, ResponseMessage, ResponseResult,
};

use crate::runtime_environment::detect_runtime;

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
    EnvironmentDetected(String),
    EnvironmentOutput(String),
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
    let launch = detect_runtime(&workspace_root)?;
    let daemon_workspace_root = launch.workspace_root().to_owned();
    sender
        .send(RuntimeMessage::EnvironmentDetected(
            launch.environment_label().to_owned(),
        ))
        .await?;
    let preparation_sender = sender.clone();
    let mut child = launch
        .spawn(move |text| {
            preparation_sender
                .try_send(RuntimeMessage::EnvironmentOutput(text))
                .context("ROS environment output receiver disconnected")
        })
        .await?;
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
