use std::{
    io::{self, BufRead, Write},
    path::PathBuf,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Sender, SyncSender},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use crate::runtime_health::RuntimeHealth;
use anyhow::Context as _;
use rclrs::{Context, CreateBasicExecutor, Node as RclrsNode, SpinOptions};
use ros_studio_model::{EndpointKind, EntityId};
use ros_studio_protocol::health::{TopicWatch, validate_watches};
use ros_studio_protocol::{
    CAPABILITY_UNAVAILABLE, DiagnosticSeverity, Event, EventMessage, INVALID_PARAMS, Request,
    RequestMessage, ResponseMessage, ResponseResult, UNSUPPORTED_PROTOCOL_VERSION,
};
use serde::Serialize;

use crate::process::ProcessWorker;
use crate::runtime_graph::{
    RuntimeEndpointInfo, RuntimeGraph, RuntimeNodeInfo, project_id_for_workspace,
};

type SharedOutput = Arc<Mutex<Box<dyn Write + Send>>>;

struct RuntimeWorker {
    running: Arc<AtomicBool>,
    handle: JoinHandle<anyhow::Result<()>>,
    health_requests: Sender<Vec<TopicWatch>>,
}

impl RuntimeWorker {
    fn start(project_id: EntityId, output: SharedOutput) -> anyhow::Result<Self> {
        let context = Context::default_from_env().context("failed to initialize ROS 2 context")?;
        let executor = context.create_basic_executor();
        let observer_name = format!("ros_studio_introspection_{}", std::process::id());
        let observer = executor
            .create_node(observer_name.as_str())
            .context("failed to create ROS 2 introspection node")?;
        let observer_namespace = observer.namespace();
        let running = Arc::new(AtomicBool::new(true));
        let worker_running = running.clone();
        let (health_requests, health_receiver) = mpsc::channel();

        let handle = thread::spawn(move || {
            // Graph queries use the node while its context and executor stay alive.
            let _context = context;
            let mut executor = executor;
            let mut health = RuntimeHealth::new(project_id.clone());
            let mut graph = RuntimeGraph::new(project_id);
            let mut last_error = None;
            let mut last_spin_error = None;
            let mut next_query = Instant::now();

            while worker_running.load(Ordering::Acquire) {
                while let Ok(watches) = health_receiver.try_recv() {
                    health.configure(&observer, watches)?;
                    write_message(
                        &output,
                        &EventMessage::new(Event::HealthChanged {
                            report: health.report()?,
                        }),
                    )?;
                }
                // Arrival callbacks must spin; graph discovery alone does not
                // receive data. Short bounded spins keep shutdown responsive.
                let spin_started = Instant::now();
                let spin_error = executor
                    .spin(SpinOptions::default().timeout(Duration::from_millis(100)))
                    .first()
                    .map(ToString::to_string);
                if let Some(error) = &spin_error
                    && last_spin_error.as_ref() != Some(error)
                {
                    write_message(
                        &output,
                        &EventMessage::new(Event::Diagnostic {
                            severity: DiagnosticSeverity::Error,
                            message: format!("ROS observer: {error}"),
                            source_location: None,
                        }),
                    )?;
                }
                last_spin_error = spin_error;
                // An empty executor or an error may return immediately. Avoid
                // a busy loop when no topics are configured or receiving data.
                let rest = Duration::from_millis(100).saturating_sub(spin_started.elapsed());
                if !rest.is_zero() {
                    thread::sleep(rest);
                }
                if Instant::now() < next_query {
                    continue;
                }
                next_query = Instant::now() + Duration::from_secs(1);
                match query_graph(&observer, &observer_name, &observer_namespace) {
                    Ok((nodes, endpoints)) => {
                        last_error = None;

                        if let Some(patch) = graph.update(nodes, endpoints) {
                            write_message(
                                &output,
                                &EventMessage::new(Event::RuntimeGraphChanged { patch }),
                            )?;
                        }
                    }
                    Err(error) => {
                        let message = error.to_string();

                        if last_error.as_deref() != Some(message.as_str()) {
                            write_message(
                                &output,
                                &EventMessage::new(Event::Diagnostic {
                                    severity: DiagnosticSeverity::Error,
                                    message: message.clone(),
                                    source_location: None,
                                }),
                            )?;
                            last_error = Some(message);
                        }
                    }
                }

                if health.configured {
                    write_message(
                        &output,
                        &EventMessage::new(Event::HealthChanged {
                            report: health.report()?,
                        }),
                    )?;
                }
            }

            if health.configured {
                health.configure(&observer, Vec::new())?;
                write_message(
                    &output,
                    &EventMessage::new(Event::HealthChanged {
                        report: health.report()?,
                    }),
                )?;
            }

            if let Some(patch) = graph.clear() {
                write_message(
                    &output,
                    &EventMessage::new(Event::RuntimeGraphChanged { patch }),
                )?;
            }

            Ok(())
        });

        Ok(Self {
            running,
            handle,
            health_requests,
        })
    }

    fn stop(self) -> anyhow::Result<()> {
        self.running.store(false, Ordering::Release);
        self.handle
            .join()
            .map_err(|_| anyhow::anyhow!("ROS discovery thread panicked"))?
    }
}

pub fn serve_ros_stdio() -> io::Result<()> {
    let output: SharedOutput = Arc::new(Mutex::new(Box::new(io::stdout())));
    serve_ros(io::stdin().lock(), output)
}

fn serve_ros(reader: impl BufRead, output: SharedOutput) -> io::Result<()> {
    let (process_event_sender, process_event_receiver) = mpsc::sync_channel(256);
    let process_event_writer = thread::spawn({
        let output = output.clone();
        move || {
            for event in process_event_receiver {
                write_message(&output, &EventMessage::new(event))?;
            }
            io::Result::Ok(())
        }
    });
    let mut project_id = None;
    let mut workspace_root = None;
    let mut worker: Option<RuntimeWorker> = None;
    let mut processes = Vec::new();

    let serving = (|| -> io::Result<()> {
        for line in reader.lines() {
            let line = line?;
            reap_finished_processes(&mut processes).map_err(io::Error::other)?;
            let (response, process_to_activate) =
                match serde_json::from_str::<RequestMessage>(&line) {
                    Ok(message) => handle_request(
                        message,
                        &mut project_id,
                        &mut workspace_root,
                        &mut worker,
                        &mut processes,
                        &process_event_sender,
                        &output,
                    ),
                    Err(error) => (crate::malformed_request(&line, &error), None),
                };

            write_message(&output, &response)?;
            if let Some(process_id) = process_to_activate {
                let activation_result = processes
                    .iter()
                    .find(|process| process.id() == process_id)
                    .context("newly launched process is missing")
                    .and_then(ProcessWorker::activate);
                if let Err(error) = activation_result {
                    process_event_sender
                        .send(Event::Diagnostic {
                            severity: DiagnosticSeverity::Error,
                            message: error.to_string(),
                            source_location: None,
                        })
                        .map_err(io::Error::other)?;
                }
            }
        }

        Ok(())
    })();

    // Every shutdown step must run even when serving or discovery has failed.
    let discovery = worker.map(RuntimeWorker::stop).transpose();
    let processes = stop_processes(&mut processes);
    drop(process_event_sender);
    let writer = process_event_writer.join();

    serving?;
    discovery.map_err(io::Error::other)?;
    processes.map_err(io::Error::other)?;
    writer.map_err(|_| io::Error::other("process event writer thread panicked"))??;

    Ok(())
}

fn handle_request(
    message: RequestMessage,
    project_id: &mut Option<EntityId>,
    workspace_root: &mut Option<PathBuf>,
    worker: &mut Option<RuntimeWorker>,
    processes: &mut Vec<ProcessWorker>,
    process_events: &SyncSender<Event>,
    output: &SharedOutput,
) -> (ResponseMessage, Option<String>) {
    if let Err(error) = message.validate() {
        return (
            ResponseMessage::error(
                Some(message.id),
                UNSUPPORTED_PROTOCOL_VERSION,
                error.to_string(),
            ),
            None,
        );
    }

    match message.request {
        Request::OpenWorkspace { path } => {
            let Some(next_project_id) = project_id_for_workspace(&path) else {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        INVALID_PARAMS,
                        "workspace path has no valid name",
                    ),
                    None,
                );
            };

            if let Some(active_worker) = worker.take()
                && let Err(error) = active_worker.stop()
            {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                );
            }
            if let Err(error) = stop_processes(processes) {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                );
            }

            *project_id = Some(next_project_id);
            *workspace_root = Some(PathBuf::from(path));
            (
                ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                None,
            )
        }
        Request::StartRuntimeDiscovery {} => {
            if worker
                .as_ref()
                .is_some_and(|active| active.handle.is_finished())
                && let Some(finished_worker) = worker.take()
                && let Err(error) = finished_worker.stop()
            {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                );
            }

            if worker.is_some() {
                return (
                    ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                    None,
                );
            }

            let Some(project_id) = project_id.clone() else {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        INVALID_PARAMS,
                        "open a workspace before starting ROS discovery",
                    ),
                    None,
                );
            };

            match RuntimeWorker::start(project_id, output.clone()) {
                Ok(started_worker) => {
                    *worker = Some(started_worker);
                    (
                        ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                        None,
                    )
                }
                Err(error) => (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                ),
            }
        }
        Request::StopRuntimeDiscovery {} => {
            if let Some(active_worker) = worker.take()
                && let Err(error) = active_worker.stop()
            {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                );
            }

            (
                ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                None,
            )
        }
        Request::ConfigureHealth { topics } => {
            let result = validate_watches(&topics)
                .map_err(anyhow::Error::msg)
                .and_then(|()| {
                    worker
                        .as_ref()
                        .context("Start runtime discovery before monitoring topics")
                })
                .and_then(|worker| {
                    worker
                        .health_requests
                        .send(topics)
                        .context("Health observer stopped")
                });
            match result {
                Ok(()) => (
                    ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                    None,
                ),
                Err(error) => (
                    ResponseMessage::error(Some(message.id), INVALID_PARAMS, error.to_string()),
                    None,
                ),
            }
        }
        Request::Launch { command, env } => {
            let Some(workspace_root) = workspace_root.as_deref() else {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        INVALID_PARAMS,
                        "open a workspace before launching a process",
                    ),
                    None,
                );
            };
            match ProcessWorker::start(command, env, workspace_root, process_events.clone()) {
                Ok(process) => {
                    let process_id = process.id().to_owned();
                    processes.push(process);
                    (
                        ResponseMessage::success(
                            message.id,
                            ResponseResult::ProcessStarted {
                                id: process_id.clone(),
                            },
                        ),
                        Some(process_id),
                    )
                }
                Err(error) => (
                    ResponseMessage::error(Some(message.id), INVALID_PARAMS, error.to_string()),
                    None,
                ),
            }
        }
        Request::StopProcess { id } => {
            let Some(index) = processes.iter().position(|process| process.id() == id) else {
                return (
                    ResponseMessage::error(
                        Some(message.id),
                        INVALID_PARAMS,
                        format!("process {id} is not running"),
                    ),
                    None,
                );
            };
            match processes.swap_remove(index).stop() {
                Ok(()) => (
                    ResponseMessage::success(message.id, ResponseResult::Accepted {}),
                    None,
                ),
                Err(error) => (
                    ResponseMessage::error(
                        Some(message.id),
                        CAPABILITY_UNAVAILABLE,
                        error.to_string(),
                    ),
                    None,
                ),
            }
        }
        Request::GetParameters { .. } | Request::SetParameter { .. } => (
            ResponseMessage::error(
                Some(message.id),
                CAPABILITY_UNAVAILABLE,
                "parameters are not available yet",
            ),
            None,
        ),
    }
}

fn reap_finished_processes(processes: &mut Vec<ProcessWorker>) -> anyhow::Result<()> {
    let mut index = 0;
    while index < processes.len() {
        if processes[index].is_finished() {
            processes.swap_remove(index).stop()?;
        } else {
            index += 1;
        }
    }
    Ok(())
}

fn stop_processes(processes: &mut Vec<ProcessWorker>) -> anyhow::Result<()> {
    let mut first_error = None;
    for process in processes.drain(..) {
        if let Err(error) = process.stop()
            && first_error.is_none()
        {
            first_error = Some(error);
        }
    }
    match first_error {
        Some(error) => Err(error),
        None => Ok(()),
    }
}

fn query_graph(
    observer: &RclrsNode,
    observer_name: &str,
    observer_namespace: &str,
) -> anyhow::Result<(Vec<RuntimeNodeInfo>, Vec<RuntimeEndpointInfo>)> {
    let nodes = observer
        .get_node_names()
        .context("failed to query ROS node names")?
        .into_iter()
        .filter(|node| {
            is_known_ros_node(&node.name, &node.namespace)
                && (node.name != observer_name || node.namespace != observer_namespace)
        })
        .map(|node| RuntimeNodeInfo {
            name: node.name,
            namespace: node.namespace,
        })
        .collect();

    let mut topics = observer
        .get_topic_names_and_types()
        .context("failed to query ROS topics")?
        .into_keys()
        .collect::<Vec<_>>();
    topics.sort();

    let mut endpoints = Vec::new();

    for topic in topics {
        for info in observer
            .get_publishers_info_by_topic(&topic)
            .with_context(|| format!("failed to query publishers for {topic}"))?
        {
            if !is_known_ros_node(&info.node_name, &info.node_namespace)
                || (info.node_name == observer_name && info.node_namespace == observer_namespace)
            {
                continue;
            }

            endpoints.push(RuntimeEndpointInfo {
                node_name: info.node_name,
                node_namespace: info.node_namespace,
                name: topic.clone(),
                type_name: info.topic_type,
                kind: EndpointKind::Publisher,
            });
        }

        for info in observer
            .get_subscriptions_info_by_topic(&topic)
            .with_context(|| format!("failed to query subscriptions for {topic}"))?
        {
            if !is_known_ros_node(&info.node_name, &info.node_namespace)
                || (info.node_name == observer_name && info.node_namespace == observer_namespace)
            {
                continue;
            }

            endpoints.push(RuntimeEndpointInfo {
                node_name: info.node_name,
                node_namespace: info.node_namespace,
                name: topic.clone(),
                type_name: info.topic_type,
                kind: EndpointKind::Subscription,
            });
        }
    }

    Ok((nodes, endpoints))
}

fn is_known_ros_node(name: &str, namespace: &str) -> bool {
    // RMW can report these placeholders while endpoint discovery is incomplete.
    !name.is_empty() && name != "*NODE_NAME_UNKNOWN*" && namespace != "*NODE_NAMESPACE_UNKNOWN*"
}

fn write_message(message_output: &SharedOutput, message: &impl Serialize) -> io::Result<()> {
    let mut output = message_output
        .lock()
        .map_err(|_| io::Error::other("ROS daemon output lock was poisoned"))?;
    serde_json::to_writer(&mut *output, message).map_err(io::Error::other)?;
    output.write_all(b"\n")?;
    output.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FailingInput {
        input: io::Cursor<Vec<u8>>,
        pid_path: PathBuf,
    }

    fn wait_for_pid(path: &std::path::Path) -> io::Result<()> {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !path.is_file() {
            if Instant::now() >= deadline {
                return Err(io::Error::other("test child did not start"));
            }
            thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }

    impl io::Read for FailingInput {
        fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
            self.input.read(bytes)
        }
    }

    impl BufRead for FailingInput {
        fn fill_buf(&mut self) -> io::Result<&[u8]> {
            self.input.fill_buf()
        }
        fn consume(&mut self, count: usize) {
            self.input.consume(count);
        }
        fn read_line(&mut self, line: &mut String) -> io::Result<usize> {
            if self.input.position() as usize == self.input.get_ref().len() {
                wait_for_pid(&self.pid_path)?;
                return Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "injected input failure",
                ));
            }
            self.input.read_line(line)
        }
    }

    struct FailingOutput {
        flushes: usize,
        pid_path: PathBuf,
    }
    impl Write for FailingOutput {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            self.flushes += 1;
            if self.flushes >= 2 {
                wait_for_pid(&self.pid_path)?;
                return Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "injected output failure",
                ));
            }
            Ok(())
        }
    }

    #[cfg(unix)]
    #[test]
    fn input_and_response_errors_still_stop_launched_processes() -> anyhow::Result<()> {
        for output_failure in [false, true] {
            let root = tempfile::tempdir()?;
            let pid_path = root.path().join("child.pid");
            let messages = [
                RequestMessage::new(
                    1,
                    Request::OpenWorkspace {
                        path: root.path().to_string_lossy().into_owned(),
                    },
                ),
                RequestMessage::new(
                    2,
                    Request::Launch {
                        command: vec![
                            "sh".into(),
                            "-c".into(),
                            "echo $$ > child.pid; exec sleep 30".into(),
                        ],
                        env: std::collections::BTreeMap::new(),
                    },
                ),
            ];
            let mut input = Vec::new();
            for message in messages {
                serde_json::to_writer(&mut input, &message)?;
                input.push(b'\n');
            }
            let output: Box<dyn Write + Send> = if output_failure {
                Box::new(FailingOutput {
                    flushes: 0,
                    pid_path: pid_path.clone(),
                })
            } else {
                Box::new(io::sink())
            };
            let result = serve_ros(
                FailingInput {
                    input: io::Cursor::new(input),
                    pid_path: pid_path.clone(),
                },
                Arc::new(Mutex::new(output)),
            );
            assert_eq!(result.unwrap_err().kind(), io::ErrorKind::BrokenPipe);
            let pid: i32 = std::fs::read_to_string(pid_path)?.trim().parse()?;
            assert_eq!(
                nix::sys::signal::kill(nix::unistd::Pid::from_raw(pid), None),
                Err(nix::errno::Errno::ESRCH)
            );
        }
        Ok(())
    }

    #[test]
    fn ignores_unresolved_ros_graph_nodes() {
        assert!(is_known_ros_node("camera", "/drone"));
        assert!(!is_known_ros_node("*NODE_NAME_UNKNOWN*", "/drone"));
        assert!(!is_known_ros_node("camera", "*NODE_NAMESPACE_UNKNOWN*"));
        assert!(!is_known_ros_node("", "/drone"));
    }
}
