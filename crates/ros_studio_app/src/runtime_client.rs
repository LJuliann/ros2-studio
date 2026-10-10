use std::{
    collections::BTreeMap,
    path::PathBuf,
    time::{Duration, Instant},
};

use anyhow::{Context as _, Result, bail, ensure};
use async_channel::{Receiver, Sender};
use futures::{
    AsyncReadExt as _, AsyncWrite, FutureExt as _, StreamExt as _,
    io::{AsyncBufReadExt as _, AsyncWriteExt as _, BufReader},
    pin_mut, select,
};
use ros_studio_protocol::{
    Event, EventMessage, GraphPatch, OutputStream, Request, RequestMessage, ResponseMessage,
    ResponseResult,
};

use crate::{
    runtime::{Execution, Operation},
    runtime_environment::{LaunchPlan, RuntimePreference, detect_runtime},
};
use ros_studio_protocol::health::{HealthReport, TopicWatch};

pub enum ClientCommand {
    ConfigureHealth(Vec<TopicWatch>),
    Launch(Execution),
    Stop(String),
    Shutdown,
}

pub enum RuntimeMessage {
    Health(HealthReport),
    HealthUnavailable(String),
    Environment(String),
    Output {
        stream: OutputStream,
        text: String,
    },
    Ready {
        discovery: bool,
    },
    Patch(GraphPatch),
    Started {
        id: String,
        operation: Operation,
    },
    Exited {
        id: String,
        success: bool,
        code: Option<i32>,
    },
    Failed {
        operation: Option<Operation>,
        message: String,
    },
    Disconnected(Option<String>),
}

pub struct SessionHandle {
    sender: Sender<ClientCommand>,
}

impl SessionHandle {
    pub fn new(sender: Sender<ClientCommand>) -> Self {
        Self { sender }
    }
    pub fn send(&self, command: ClientCommand) -> Result<()> {
        self.sender
            .try_send(command)
            .context("ROS connection is closed")
    }
}

impl Drop for SessionHandle {
    fn drop(&mut self) {
        if let Err(error) = self.sender.try_send(ClientCommand::Shutdown) {
            if !error.is_closed() {
                eprintln!("Cannot close ROS session: {error}");
            }
        }
        self.sender.close();
    }
}

pub async fn connect(
    root: PathBuf,
    preference: RuntimePreference,
    commands: Receiver<ClientCommand>,
    sender: Sender<RuntimeMessage>,
) {
    let result = async {
        let plan = detect_runtime(&root, preference)?;
        run_session(plan, commands, &sender).await
    }
    .await;
    if let Err(error) = sender
        .send(RuntimeMessage::Disconnected(
            result.err().map(|error| format!("{error:#}")),
        ))
        .await
    {
        eprintln!("ROS session output receiver closed: {error}");
    }
}

#[derive(Clone, Copy)]
enum Pending {
    Health,
    Open,
    Discovery,
    Launch(Operation),
    Stop,
}

async fn request(
    writer: &mut (impl AsyncWrite + Unpin),
    next_id: &mut u64,
    pending: &mut BTreeMap<u64, (Pending, Instant)>,
    kind: Pending,
    request: Request,
) -> Result<()> {
    let message = RequestMessage::new(*next_id, request);
    let mut bytes = serde_json::to_vec(&message)?;
    bytes.push(b'\n');
    writer.write_all(&bytes).await?;
    writer.flush().await?;
    pending.insert(*next_id, (kind, Instant::now()));
    *next_id = next_id.checked_add(1).context("Request IDs exhausted")?;
    Ok(())
}

pub enum DaemonMessage {
    Event(Event),
    Response(ResponseMessage),
}

pub fn decode_line(line: &str) -> Result<DaemonMessage> {
    let value: serde_json::Value =
        serde_json::from_str(line).context("Daemon sent invalid JSON")?;
    if value.get("method").is_some() {
        let event: EventMessage = serde_json::from_value(value)?;
        event.validate()?;
        Ok(DaemonMessage::Event(event.event))
    } else {
        let response: ResponseMessage = serde_json::from_value(value)?;
        response.validate()?;
        Ok(DaemonMessage::Response(response))
    }
}

async fn run_session(
    plan: LaunchPlan,
    commands: Receiver<ClientCommand>,
    sender: &Sender<RuntimeMessage>,
) -> Result<()> {
    sender
        .send(RuntimeMessage::Environment(plan.label.clone()))
        .await?;
    if let Some(preparation) = &plan.preparation {
        let mut child = preparation
            .command()
            .stdin(std::process::Stdio::null())
            .spawn()
            .context("Cannot prepare Docker image")?;
        let stdout = child.stdout.take().context("Preparation has no stdout")?;
        let stderr = child.stderr.take().context("Preparation has no stderr")?;
        let preparation = async {
            let status = futures::future::try_join3(
                forward_output(stdout, OutputStream::Stdout, sender),
                forward_output(stderr, OutputStream::Stderr, sender),
                async { Ok::<_, anyhow::Error>(child.status().await?) },
            )
            .await?
            .2;
            ensure!(
                status.success(),
                "Docker image preparation failed ({status})"
            );
            Ok::<_, anyhow::Error>(())
        }
        .fuse();
        let cancelled = commands.recv().fuse();
        pin_mut!(preparation, cancelled);
        select! {
            result = preparation => result?,
            command = cancelled => {
                match command {
                    Ok(ClientCommand::Shutdown) | Err(_) => return Ok(()),
                    _ => bail!("Wait for Docker preparation before launching a node"),
                }
            }
        }
    }
    let mut child = plan
        .daemon
        .command()
        .spawn()
        .context("Cannot launch ROS daemon")?;
    let mut stdin = child.stdin.take().context("Daemon has no stdin")?;
    let stdout = child.stdout.take().context("Daemon has no stdout")?;
    let stderr = child.stderr.take().context("Daemon has no stderr")?;
    let mut stdout = BufReader::new(stdout).lines();
    let mut stderr = BufReader::new(stderr).lines();
    let mut stderr_open = true;
    let mut pending = BTreeMap::new();
    let mut next_id = 1;
    let conversation = async {
        request(&mut stdin, &mut next_id, &mut pending, Pending::Open, Request::OpenWorkspace { path: plan.workspace }).await?;
        loop {
            let command = commands.recv().fuse();
            let output = stdout.next().fuse();
            let diagnostic = async {
                if stderr_open { stderr.next().await } else { futures::future::pending().await }
            }.fuse();
            let deadline = pending.values().map(|(_, sent)| *sent + Duration::from_secs(30)).min();
            let timeout = async {
                match deadline {
                    Some(deadline) => { async_io::Timer::at(deadline).await; },
                    None => futures::future::pending().await,
                }
            }.fuse();
            pin_mut!(command, output, diagnostic, timeout);
            select! {
                _ = timeout => bail!("ROS daemon did not reply to a request within 30 seconds"),
                command = command => match command {
                    Ok(ClientCommand::Shutdown) | Err(_) => return Ok(()),
                    Ok(ClientCommand::Launch(execution)) => {
                        request(&mut stdin, &mut next_id, &mut pending, Pending::Launch(execution.operation), Request::Launch {
                            command: execution.command,
                            env: BTreeMap::from([("CARGO_BUILD_JOBS".into(), std::env::var("CARGO_BUILD_JOBS").unwrap_or_else(|_| "1".into()))]),
                        }).await?;
                    }
                    Ok(ClientCommand::Stop(id)) => request(&mut stdin, &mut next_id, &mut pending, Pending::Stop, Request::StopProcess { id }).await?,
                    Ok(ClientCommand::ConfigureHealth(topics)) => request(&mut stdin, &mut next_id, &mut pending, Pending::Health, Request::ConfigureHealth { topics }).await?,
                },
                line = diagnostic => match line {
                    Some(line) => sender.send(RuntimeMessage::Output { stream: OutputStream::Stderr, text: format!("{}\n", line?) }).await?,
                    None => stderr_open = false,
                },
                line = output => {
                    let line = line.context("ROS daemon closed its output")??;
                    let decoded = match decode_line(&line) {
                        Ok(message) => message,
                        Err(error) => {
                            sender.send(RuntimeMessage::Output { stream: OutputStream::Stderr, text: format!("{line}\n") }).await?;
                            return Err(error);
                        }
                    };
                    match decoded {
                        DaemonMessage::Event(event) => {
                            let message = match event {
                                Event::HealthChanged { report } => RuntimeMessage::Health(report),
                                Event::RuntimeGraphChanged { patch } => RuntimeMessage::Patch(patch),
                                Event::ProcessOutput { stream, text, .. } => RuntimeMessage::Output { stream, text },
                                Event::ProcessExited { id, success, code } => RuntimeMessage::Exited { id, success, code },
                                Event::Diagnostic { message, .. } => RuntimeMessage::Output { stream: OutputStream::Stderr, text: format!("{message}\n") },
                                Event::StaticGraphChanged { .. } => continue,
                            };
                            sender.send(message).await?;
                        }
                        DaemonMessage::Response(response) => {
                            let id = response.id.context("Daemon response has no request ID")?;
                            let (kind, _) = pending.remove(&id).context("Daemon response has an unknown request ID")?;
                            if let Some(error) = response.error {
                                if matches!(kind, Pending::Health) {
                                    sender.send(RuntimeMessage::HealthUnavailable(error.message)).await?;
                                    continue;
                                }
                                if matches!(kind, Pending::Open) { bail!("Cannot open workspace: {}", error.message); }
                                if matches!(kind, Pending::Discovery) { sender.send(RuntimeMessage::Ready { discovery: false }).await?; }
                                sender.send(RuntimeMessage::Failed { operation: if let Pending::Launch(operation) = kind { Some(operation) } else { None }, message: error.message }).await?;
                            } else {
                                match (kind, response.result) {
                                    (Pending::Open, Some(ResponseResult::Accepted {})) => request(&mut stdin, &mut next_id, &mut pending, Pending::Discovery, Request::StartRuntimeDiscovery {}).await?,
                                    (Pending::Discovery, Some(ResponseResult::Accepted {})) => sender.send(RuntimeMessage::Ready { discovery: true }).await?,
                                    (Pending::Launch(operation), Some(ResponseResult::ProcessStarted { id })) => sender.send(RuntimeMessage::Started { id, operation }).await?,
                                    (Pending::Stop, Some(ResponseResult::Accepted {})) => {},
                                    (Pending::Health, Some(ResponseResult::Accepted {})) => {},
                                    _ => bail!("Unexpected daemon response result"),
                                }
                            }
                        }
                    }
                }
            }
        }
    }.await;
    // EOF lets the daemon stop its discovery thread and all launched process groups.
    drop(stdin);
    let shutdown = async {
        let (_, _, status) = futures::future::try_join3(
            async {
                while let Some(line) = stdout.next().await {
                    line?;
                }
                Ok::<_, anyhow::Error>(())
            },
            async {
                while let Some(line) = stderr.next().await {
                    let line = line?;
                    if !sender.is_closed() {
                        if let Err(error) = sender
                            .send(RuntimeMessage::Output {
                                stream: OutputStream::Stderr,
                                text: format!("{line}\n"),
                            })
                            .await
                        {
                            eprintln!("ROS stderr receiver closed during shutdown: {error}");
                        }
                    }
                }
                Ok::<_, anyhow::Error>(())
            },
            async { Ok::<_, anyhow::Error>(child.status().await?) },
        )
        .await?;
        ensure!(status.success(), "Daemon exited with {status}");
        Ok::<_, anyhow::Error>(())
    }
    .fuse();
    let timeout = futures::FutureExt::fuse(async_io::Timer::after(Duration::from_secs(10)));
    pin_mut!(shutdown, timeout);
    select! {
        status = shutdown => { status.context("Cannot wait for daemon shutdown")?; },
        _ = timeout => bail!("Daemon did not shut down within 10 seconds"),
    }
    conversation
}

pub(crate) async fn forward_output(
    mut reader: impl futures::AsyncRead + Unpin,
    stream: OutputStream,
    sender: &Sender<RuntimeMessage>,
) -> Result<()> {
    let mut decoder = ros_studio_protocol::output::OutputDecoder::default();
    let mut buffer = [0; 4096];
    loop {
        let count = reader.read(&mut buffer).await?;
        let text = decoder.decode(&buffer[..count], count == 0);
        if !text.is_empty() {
            sender.send(RuntimeMessage::Output { stream, text }).await?;
        }
        if count == 0 {
            break;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime_environment::CommandSpec;

    #[test]
    fn rejects_malformed_and_incompatible_daemon_messages() {
        assert!(decode_line("not json").is_err());
        assert!(
            decode_line(
                r#"{"jsonrpc":"2.0","protocol_version":99,"id":1,"result":{"kind":"accepted"}}"#
            )
            .is_err()
        );
        assert!(
            decode_line(
                r#"{"jsonrpc":"2.0","protocol_version":1,"id":1,"result":{"kind":"accepted"}}"#
            )
            .is_ok()
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn health_events_and_old_daemon_rejections_do_not_interrupt_process_launches() -> Result<()> {
        for supported in [true, false] {
            futures::executor::block_on(async {
                let root = tempfile::tempdir()?;
                let watch = TopicWatch {
                    topic: "/imu".into(),
                    type_name: "sensor_msgs/msg/Imu".into(),
                    expected_hz: 200.0,
                    tolerance: 0.2,
                    timeout_ms: 3000,
                };
                let observation = EventMessage::new(Event::HealthChanged {
                    report: HealthReport {
                        project_id: ros_studio_model::EntityId::new("project:test"),
                        topics: vec![ros_studio_protocol::health::TopicHealth {
                            topic: watch.topic.clone(),
                            settings: watch.clone(),
                            observed_hz: Some(200.0),
                            age_ms: Some(0),
                            state: ros_studio_protocol::health::TopicHealthState::Healthy,
                            detail: "received".into(),
                        }],
                    },
                });
                let response = if supported {
                    serde_json::to_string(&ResponseMessage::success(
                        3,
                        ResponseResult::Accepted {},
                    ))?
                } else {
                    serde_json::to_string(&ResponseMessage::error(
                        Some(3),
                        -32601,
                        "unknown configure_health method",
                    ))?
                };
                let notification = if supported {
                    format!("printf '%s\\n' '{}'", serde_json::to_string(&observation)?)
                } else {
                    String::new()
                };
                let script = format!(
                    r#"
read -r request
printf '%s\n' '{{"jsonrpc":"2.0","protocol_version":1,"id":1,"result":{{"kind":"accepted"}}}}'
read -r request
printf '%s\n' '{{"jsonrpc":"2.0","protocol_version":1,"id":2,"result":{{"kind":"accepted"}}}}'
read -r request
printf '%s\n' '{response}'
{notification}
read -r request
printf '%s\n' '{{"jsonrpc":"2.0","protocol_version":1,"id":4,"result":{{"kind":"process_started","id":"test-process"}}}}'
printf '%s\n' '{{"jsonrpc":"2.0","protocol_version":1,"method":"process_exited","params":{{"id":"test-process","success":true,"code":0}}}}'
while read -r request; do :; done
"#
                );
                let plan = LaunchPlan {
                    daemon: CommandSpec {
                        program: "sh".into(),
                        arguments: vec!["-c".into(), script.into()],
                        directory: root.path().into(),
                    },
                    preparation: None,
                    workspace: root.path().to_string_lossy().into_owned(),
                    label: "Health client test".into(),
                };
                let (commands, receiver) = async_channel::unbounded();
                let handle = SessionHandle::new(commands);
                let (sender, events) = async_channel::bounded(32);
                let client = run_session(plan, receiver, &sender);
                let driver = async {
                    let mut observed = false;
                    let mut launched = false;
                    while let Ok(event) = events.recv().await {
                        match event {
                            RuntimeMessage::Ready { discovery: true } => {
                                handle.send(ClientCommand::ConfigureHealth(vec![watch.clone()]))?
                            }
                            RuntimeMessage::Health(report) => {
                                ensure!(
                                    supported && report.topics[0].observed_hz == Some(200.0),
                                    "Incorrect health event"
                                );
                                observed = true;
                            }
                            RuntimeMessage::HealthUnavailable(message) => {
                                ensure!(
                                    !supported && message.contains("configure_health"),
                                    "Incorrect capability error"
                                );
                                observed = true;
                            }
                            RuntimeMessage::Started { .. } => launched = true,
                            RuntimeMessage::Exited { success: true, .. } => {
                                ensure!(
                                    observed && launched,
                                    "Health must not interrupt process launching"
                                );
                                handle.send(ClientCommand::Shutdown)?;
                                return Ok(());
                            }
                            RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                            _ => {}
                        }
                        if observed && !launched {
                            handle.send(ClientCommand::Launch(Execution {
                                operation: Operation::Build,
                                label: "test".into(),
                                command: vec!["test".into()],
                            }))?;
                            // Prevent duplicate launch while the started response is pending.
                            launched = true;
                        }
                    }
                    bail!("Health test session closed prematurely")
                };
                let completed = futures::future::try_join(client, driver).fuse();
                let timeout =
                    futures::FutureExt::fuse(async_io::Timer::after(Duration::from_secs(5)));
                pin_mut!(completed, timeout);
                select! { result = completed => { result?; }, _ = timeout => bail!("Health client test timed out") }
                Ok(())
            })?;
        }
        Ok(())
    }

    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "requires a rebuilt ROS runtime daemon and sourced ROS Humble or Docker; creates only a dedicated test topic"]
    fn real_daemon_measures_topic_rate_then_silence_and_removes_the_watch() -> Result<()> {
        futures::executor::block_on(async {
            use ros_studio_protocol::health::TopicHealthState;
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../examples/drone_demo_ws")
                .canonicalize()?;
            let plan = detect_runtime(&root, RuntimePreference::from_environment()?)?;
            let topic = format!("/ros_studio_health_smoke_{}", std::process::id());
            let (commands, receiver) = async_channel::unbounded();
            let handle = SessionHandle::new(commands);
            let (sender, events) = async_channel::bounded(256);
            let client = run_session(plan, receiver, &sender);
            let driver = async {
                let mut process = None;
                let mut stopped = false;
                let mut exited = false;
                let mut removing = false;
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Ready { discovery: true } => {
                            handle.send(ClientCommand::ConfigureHealth(vec![TopicWatch {
                                topic: topic.clone(),
                                type_name: "std_msgs/msg/String".into(),
                                expected_hz: 20.0,
                                tolerance: 0.2,
                                timeout_ms: 2000,
                            }]))?;
                            handle.send(ClientCommand::Launch(Execution {
                                operation: Operation::Run,
                                label: "Health smoke publisher".into(),
                                command: vec![
                                    "ros2".into(),
                                    "topic".into(),
                                    "pub".into(),
                                    "--rate".into(),
                                    "20".into(),
                                    topic.clone(),
                                    "std_msgs/msg/String".into(),
                                    "{data: hello}".into(),
                                ],
                            }))?;
                        }
                        RuntimeMessage::Started { id, .. } => process = Some(id),
                        RuntimeMessage::Exited { .. } => exited = true,
                        RuntimeMessage::Health(report) => {
                            if removing && report.topics.is_empty() {
                                ensure!(
                                    exited && stopped,
                                    "Watch cleared before publisher was stopped"
                                );
                                handle.send(ClientCommand::Shutdown)?;
                                return Ok(());
                            }
                            if let Some(observed) = report
                                .topics
                                .iter()
                                .find(|observed| observed.topic == topic)
                            {
                                if observed.state == TopicHealthState::Healthy
                                    && !stopped
                                    && let Some(id) = &process
                                {
                                    ensure!(
                                        observed
                                            .observed_hz
                                            .is_some_and(|hz| (16.0..=24.0).contains(&hz)),
                                        "Incorrect rate"
                                    );
                                    handle.send(ClientCommand::Stop(id.clone()))?;
                                    stopped = true;
                                }
                                if stopped
                                    && exited
                                    && !removing
                                    && observed.state == TopicHealthState::Missing
                                {
                                    handle.send(ClientCommand::ConfigureHealth(vec![]))?;
                                    removing = true;
                                }
                                if observed.state == TopicHealthState::Unavailable {
                                    bail!("{}", observed.detail);
                                }
                            }
                        }
                        RuntimeMessage::HealthUnavailable(message)
                        | RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                        _ => {}
                    }
                }
                bail!("Health smoke session closed before completion")
            };
            let completed = futures::future::try_join(client, driver).fuse();
            let timeout = futures::FutureExt::fuse(async_io::Timer::after(Duration::from_secs(45)));
            pin_mut!(completed, timeout);
            select! { result = completed => { result?; }, _ = timeout => bail!("ROS health smoke timed out") }
            Ok(())
        })
    }

    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "requires a built ros-runtime daemon and a ROS Humble host or Docker image"]
    fn real_daemon_discovers_streams_and_stops_a_harmless_process() -> Result<()> {
        futures::executor::block_on(async {
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../examples/drone_demo_ws")
                .canonicalize()?;
            let plan = detect_runtime(&root, RuntimePreference::from_environment()?)?;
            let (commands, receiver) = async_channel::unbounded();
            let handle = SessionHandle::new(commands);
            let (sender, events) = async_channel::bounded(256);
            let client = run_session(plan, receiver, &sender);
            let driver = async {
                let mut ready = false;
                let mut patch = false;
                let mut launched = false;
                let mut process = None;
                let mut stdout = false;
                let mut stderr = false;
                let mut stopped = false;
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Ready { discovery } => {
                            ensure!(discovery, "Discovery unavailable");
                            ready = true;
                        }
                        RuntimeMessage::Patch(graph) => {
                            ensure!(
                                graph.project_id.as_str() == "project:drone_demo_ws",
                                "Incorrect runtime project ID"
                            );
                            patch = true;
                        }
                        RuntimeMessage::Started { id, .. } => process = Some(id),
                        RuntimeMessage::Output { stream, text } => {
                            if text.contains("standalone smoke stdout") {
                                stdout = stream == OutputStream::Stdout;
                            }
                            if text.contains("standalone smoke stderr") {
                                stderr = stream == OutputStream::Stderr;
                            }
                        }
                        RuntimeMessage::Failed { message, .. } => bail!("{message}"),
                        RuntimeMessage::Exited { id, .. } => {
                            ensure!(
                                stopped && process.as_ref() == Some(&id),
                                "Unexpected process exit"
                            );
                            handle.send(ClientCommand::Shutdown)?;
                            return Ok(());
                        }
                        _ => {}
                    }
                    if ready && patch && !launched {
                        handle.send(ClientCommand::Launch(Execution {
                            operation: Operation::Build, label: "Harmless integration smoke".into(),
                            command: vec!["sh".into(), "-c".into(), "printf 'standalone smoke stdout\\n'; printf 'standalone smoke stderr\\n' >&2; sleep 30".into()],
                        }))?;
                        launched = true;
                    }
                    if stdout
                        && stderr
                        && !stopped
                        && let Some(id) = &process
                    {
                        handle.send(ClientCommand::Stop(id.clone()))?;
                        stopped = true;
                    }
                }
                bail!("Runtime channel closed before smoke test completed")
            };
            let completed = futures::future::try_join(client, driver).fuse();
            let timeout = futures::FutureExt::fuse(async_io::Timer::after(Duration::from_secs(45)));
            pin_mut!(completed, timeout);
            select! { result = completed => { result?; }, _ = timeout => bail!("Real daemon smoke test timed out") }
            Ok(())
        })
    }

    #[cfg(unix)]
    #[test]
    fn launches_streams_stops_and_closes_a_fake_daemon() -> Result<()> {
        futures::executor::block_on(async {
            let root = tempfile::tempdir()?;
            let script = r#"
read -r request
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"id":1,"result":{"kind":"accepted"}}'
read -r request
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"id":2,"result":{"kind":"accepted"}}'
read -r request
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"id":3,"result":{"kind":"process_started","id":"test-process"}}'
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"method":"process_output","params":{"id":"test-process","stream":"stderr","text":"build error\n"}}'
read -r request
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"method":"process_exited","params":{"id":"test-process","success":false,"code":137}}'
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"id":4,"result":{"kind":"accepted"}}'
while read -r request; do :; done
"#;
            let plan = LaunchPlan {
                daemon: CommandSpec {
                    program: "sh".into(),
                    arguments: vec!["-c".into(), script.into()],
                    directory: root.path().into(),
                },
                preparation: None,
                workspace: root.path().to_string_lossy().into_owned(),
                label: "Test".into(),
            };
            let (command_sender, commands) = async_channel::unbounded();
            let (sender, events) = async_channel::bounded(32);
            let handle = SessionHandle::new(command_sender);
            let client = run_session(plan, commands, &sender);
            let driver = async {
                let mut output = false;
                while let Ok(event) = events.recv().await {
                    match event {
                        RuntimeMessage::Ready { discovery: true } => {
                            handle.send(ClientCommand::Launch(Execution {
                                operation: Operation::Build,
                                label: "test".into(),
                                command: vec!["test".into()],
                            }))?
                        }
                        RuntimeMessage::Started {
                            id,
                            operation: Operation::Build,
                        } => handle.send(ClientCommand::Stop(id))?,
                        RuntimeMessage::Output {
                            stream: OutputStream::Stderr,
                            text,
                        } => {
                            assert_eq!(text, "build error\n");
                            output = true;
                        }
                        RuntimeMessage::Exited {
                            code: Some(137),
                            success: false,
                            ..
                        } => {
                            assert!(output);
                            handle.send(ClientCommand::Shutdown)?;
                            return Ok(());
                        }
                        _ => {}
                    }
                }
                bail!("Fake daemon disconnected unexpectedly")
            };
            let completed = futures::future::try_join(client, driver).fuse();
            let timeout = futures::FutureExt::fuse(async_io::Timer::after(Duration::from_secs(5)));
            pin_mut!(completed, timeout);
            select! { result = completed => { result?; }, _ = timeout => bail!("Fake daemon test timed out") }
            Ok(())
        })
    }
}
