use std::{
    io::{self, BufRead, Write},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use anyhow::Context as _;
use rclrs::{Context, CreateBasicExecutor, Node as RclrsNode};
use ros_studio_model::{EndpointKind, EntityId};
use ros_studio_protocol::{
    CAPABILITY_UNAVAILABLE, DiagnosticSeverity, Event, EventMessage, INVALID_PARAMS, Request,
    RequestMessage, ResponseMessage, ResponseResult, UNSUPPORTED_PROTOCOL_VERSION,
};
use serde::Serialize;

use crate::runtime_graph::{
    RuntimeEndpointInfo, RuntimeGraph, RuntimeNodeInfo, project_id_for_workspace,
};

type SharedOutput = Arc<Mutex<io::Stdout>>;

struct RuntimeWorker {
    running: Arc<AtomicBool>,
    handle: JoinHandle<anyhow::Result<()>>,
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

        let handle = thread::spawn(move || {
            // Graph queries use the node while its context and executor stay alive.
            let _context = context;
            let _executor = executor;
            let mut graph = RuntimeGraph::new(project_id);
            let mut last_error = None;

            while worker_running.load(Ordering::Acquire) {
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

                thread::sleep(Duration::from_secs(1));
            }

            if let Some(patch) = graph.clear() {
                write_message(
                    &output,
                    &EventMessage::new(Event::RuntimeGraphChanged { patch }),
                )?;
            }

            Ok(())
        });

        Ok(Self { running, handle })
    }

    fn stop(self) -> anyhow::Result<()> {
        self.running.store(false, Ordering::Release);
        self.handle
            .join()
            .map_err(|_| anyhow::anyhow!("ROS discovery thread panicked"))?
    }
}

pub fn serve_ros_stdio() -> io::Result<()> {
    let output = Arc::new(Mutex::new(io::stdout()));
    let stdin = io::stdin();
    let mut project_id = None;
    let mut worker: Option<RuntimeWorker> = None;

    for line in stdin.lock().lines() {
        let line = line?;
        let response = match serde_json::from_str::<RequestMessage>(&line) {
            Ok(message) => handle_request(message, &mut project_id, &mut worker, &output),
            Err(error) => crate::malformed_request(&line, &error),
        };

        write_message(&output, &response)?;
    }

    if let Some(worker) = worker {
        worker
            .stop()
            .map_err(|error| io::Error::other(error.to_string()))?;
    }

    Ok(())
}

fn handle_request(
    message: RequestMessage,
    project_id: &mut Option<EntityId>,
    worker: &mut Option<RuntimeWorker>,
    output: &SharedOutput,
) -> ResponseMessage {
    if let Err(error) = message.validate() {
        return ResponseMessage::error(
            Some(message.id),
            UNSUPPORTED_PROTOCOL_VERSION,
            error.to_string(),
        );
    }

    match message.request {
        Request::OpenWorkspace { path } => {
            let Some(next_project_id) = project_id_for_workspace(&path) else {
                return ResponseMessage::error(
                    Some(message.id),
                    INVALID_PARAMS,
                    "workspace path has no valid name",
                );
            };

            if let Some(active_worker) = worker.take()
                && let Err(error) = active_worker.stop()
            {
                return ResponseMessage::error(
                    Some(message.id),
                    CAPABILITY_UNAVAILABLE,
                    error.to_string(),
                );
            }

            *project_id = Some(next_project_id);
            ResponseMessage::success(message.id, ResponseResult::Accepted {})
        }
        Request::StartRuntimeDiscovery {} => {
            if worker
                .as_ref()
                .is_some_and(|active| active.handle.is_finished())
                && let Some(finished_worker) = worker.take()
                && let Err(error) = finished_worker.stop()
            {
                return ResponseMessage::error(
                    Some(message.id),
                    CAPABILITY_UNAVAILABLE,
                    error.to_string(),
                );
            }

            if worker.is_some() {
                return ResponseMessage::success(message.id, ResponseResult::Accepted {});
            }

            let Some(project_id) = project_id.clone() else {
                return ResponseMessage::error(
                    Some(message.id),
                    INVALID_PARAMS,
                    "open a workspace before starting ROS discovery",
                );
            };

            match RuntimeWorker::start(project_id, output.clone()) {
                Ok(started_worker) => {
                    *worker = Some(started_worker);
                    ResponseMessage::success(message.id, ResponseResult::Accepted {})
                }
                Err(error) => ResponseMessage::error(
                    Some(message.id),
                    CAPABILITY_UNAVAILABLE,
                    error.to_string(),
                ),
            }
        }
        Request::StopRuntimeDiscovery {} => {
            if let Some(active_worker) = worker.take()
                && let Err(error) = active_worker.stop()
            {
                return ResponseMessage::error(
                    Some(message.id),
                    CAPABILITY_UNAVAILABLE,
                    error.to_string(),
                );
            }

            ResponseMessage::success(message.id, ResponseResult::Accepted {})
        }
        Request::Launch { .. } | Request::GetParameters { .. } | Request::SetParameter { .. } => {
            ResponseMessage::error(
                Some(message.id),
                CAPABILITY_UNAVAILABLE,
                "launch and parameters are not available yet",
            )
        }
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
    use super::is_known_ros_node;

    #[test]
    fn ignores_unresolved_ros_graph_nodes() {
        assert!(is_known_ros_node("camera", "/drone"));
        assert!(!is_known_ros_node("*NODE_NAME_UNKNOWN*", "/drone"));
        assert!(!is_known_ros_node("camera", "*NODE_NAMESPACE_UNKNOWN*"));
        assert!(!is_known_ros_node("", "/drone"));
    }
}
