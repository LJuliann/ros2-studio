#![forbid(unsafe_code)]

use std::{collections::BTreeMap, error::Error, fmt};

use ros_studio_model::{EntityId, Node, Package, SourceLocation};
use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const JSON_RPC_VERSION: &str = "2.0";
pub const PROTOCOL_VERSION: u32 = 1;

pub const PARSE_ERROR: i32 = -32700;
pub const INVALID_REQUEST: i32 = -32600;
pub const METHOD_NOT_FOUND: i32 = -32601;
pub const INVALID_PARAMS: i32 = -32602;
pub const UNSUPPORTED_PROTOCOL_VERSION: i32 = -32001;
pub const CAPABILITY_UNAVAILABLE: i32 = -32002;

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct RequestMessage {
    pub jsonrpc: String,
    pub protocol_version: u32,
    pub id: u64,
    #[serde(flatten)]
    pub request: Request,
}

impl RequestMessage {
    pub fn new(id: u64, request: Request) -> Self {
        Self {
            jsonrpc: JSON_RPC_VERSION.to_owned(),
            protocol_version: PROTOCOL_VERSION,
            id,
            request,
        }
    }

    pub fn validate(&self) -> Result<(), CompatibilityError> {
        validate_versions(&self.jsonrpc, self.protocol_version)
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "method", content = "params", rename_all = "snake_case")]
pub enum Request {
    OpenWorkspace {
        path: String,
    },
    StartRuntimeDiscovery {},
    StopRuntimeDiscovery {},
    Launch {
        command: Vec<String>,
        env: BTreeMap<String, String>,
    },
    GetParameters {
        node: String,
    },
    SetParameter {
        node: String,
        name: String,
        value: Value,
    },
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ResponseMessage {
    pub jsonrpc: String,
    pub protocol_version: u32,
    pub id: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<ResponseResult>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ResponseError>,
}

impl ResponseMessage {
    pub fn success(id: u64, result: ResponseResult) -> Self {
        Self {
            jsonrpc: JSON_RPC_VERSION.to_owned(),
            protocol_version: PROTOCOL_VERSION,
            id: Some(id),
            result: Some(result),
            error: None,
        }
    }

    pub fn error(id: Option<u64>, code: i32, message: impl Into<String>) -> Self {
        Self {
            jsonrpc: JSON_RPC_VERSION.to_owned(),
            protocol_version: PROTOCOL_VERSION,
            id,
            result: None,
            error: Some(ResponseError {
                code,
                message: message.into(),
            }),
        }
    }

    pub fn validate(&self) -> Result<(), CompatibilityError> {
        validate_versions(&self.jsonrpc, self.protocol_version)?;

        if self.result.is_some() == self.error.is_some() {
            return Err(CompatibilityError::InvalidResponseOutcome);
        }

        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ResponseResult {
    Accepted {},
    Parameters { values: BTreeMap<String, Value> },
    ProcessStarted { id: String },
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ResponseError {
    pub code: i32,
    pub message: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct EventMessage {
    pub jsonrpc: String,
    pub protocol_version: u32,
    #[serde(flatten)]
    pub event: Event,
}

impl EventMessage {
    pub fn new(event: Event) -> Self {
        Self {
            jsonrpc: JSON_RPC_VERSION.to_owned(),
            protocol_version: PROTOCOL_VERSION,
            event,
        }
    }

    pub fn validate(&self) -> Result<(), CompatibilityError> {
        validate_versions(&self.jsonrpc, self.protocol_version)
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "method", content = "params", rename_all = "snake_case")]
pub enum Event {
    StaticGraphChanged {
        patch: GraphPatch,
    },
    RuntimeGraphChanged {
        patch: GraphPatch,
    },
    ProcessOutput {
        id: String,
        stream: OutputStream,
        text: String,
    },
    Diagnostic {
        severity: DiagnosticSeverity,
        message: String,
        source_location: Option<SourceLocation>,
    },
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct GraphPatch {
    pub project_id: EntityId,
    pub upsert_packages: Vec<Package>,
    pub removed_package_ids: Vec<EntityId>,
    pub upsert_nodes: Vec<Node>,
    pub removed_node_ids: Vec<EntityId>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputStream {
    Stdout,
    Stderr,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticSeverity {
    Info,
    Warning,
    Error,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CompatibilityError {
    InvalidJsonRpcVersion(String),
    UnsupportedProtocolVersion(u32),
    InvalidResponseOutcome,
}

impl fmt::Display for CompatibilityError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidJsonRpcVersion(version) => {
                write!(formatter, "unsupported JSON-RPC version: {version}")
            }
            Self::UnsupportedProtocolVersion(version) => {
                write!(
                    formatter,
                    "unsupported ROS 2 Studio protocol version: {version}"
                )
            }
            Self::InvalidResponseOutcome => {
                formatter.write_str("response must contain exactly one of result or error")
            }
        }
    }
}

impl Error for CompatibilityError {}

fn validate_versions(jsonrpc: &str, protocol_version: u32) -> Result<(), CompatibilityError> {
    if jsonrpc != JSON_RPC_VERSION {
        return Err(CompatibilityError::InvalidJsonRpcVersion(
            jsonrpc.to_owned(),
        ));
    }

    if protocol_version != PROTOCOL_VERSION {
        return Err(CompatibilityError::UnsupportedProtocolVersion(
            protocol_version,
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips_with_json_rpc_envelope() -> anyhow::Result<()> {
        let request = RequestMessage::new(
            42,
            Request::OpenWorkspace {
                path: "/workspace/drone".to_owned(),
            },
        );

        let json = serde_json::to_string(&request)?;
        let decoded: RequestMessage = serde_json::from_str(&json)?;

        assert_eq!(
            json,
            r#"{"jsonrpc":"2.0","protocol_version":1,"id":42,"method":"open_workspace","params":{"path":"/workspace/drone"}}"#
        );
        assert_eq!(decoded, request);
        decoded.validate()?;

        Ok(())
    }

    #[test]
    fn every_request_variant_round_trips() -> anyhow::Result<()> {
        let requests = [
            Request::StartRuntimeDiscovery {},
            Request::StopRuntimeDiscovery {},
            Request::Launch {
                command: vec!["ros2".to_owned(), "launch".to_owned()],
                env: BTreeMap::from([("ROS_DOMAIN_ID".to_owned(), "7".to_owned())]),
            },
            Request::GetParameters {
                node: "/camera".to_owned(),
            },
            Request::SetParameter {
                node: "/camera".to_owned(),
                name: "exposure".to_owned(),
                value: Value::from(10),
            },
        ];

        for (id, request) in requests.into_iter().enumerate() {
            let message = RequestMessage::new(u64::try_from(id)?, request);
            let decoded: RequestMessage = serde_json::from_str(&serde_json::to_string(&message)?)?;
            assert_eq!(decoded, message);
            decoded.validate()?;
        }

        Ok(())
    }

    #[test]
    fn responses_have_exactly_one_outcome() -> anyhow::Result<()> {
        let success = ResponseMessage::success(42, ResponseResult::Accepted {});
        let error = ResponseMessage::error(Some(42), CAPABILITY_UNAVAILABLE, "not available");

        assert_eq!(
            serde_json::to_string(&success)?,
            r#"{"jsonrpc":"2.0","protocol_version":1,"id":42,"result":{"kind":"accepted"}}"#
        );
        assert_eq!(
            serde_json::to_string(&error)?,
            r#"{"jsonrpc":"2.0","protocol_version":1,"id":42,"error":{"code":-32002,"message":"not available"}}"#
        );
        assert_eq!(
            serde_json::from_str::<ResponseMessage>(&serde_json::to_string(&success)?)?,
            success
        );
        assert_eq!(
            serde_json::from_str::<ResponseMessage>(&serde_json::to_string(&error)?)?,
            error
        );
        success.validate()?;
        error.validate()?;

        let invalid = ResponseMessage {
            result: Some(ResponseResult::Accepted {}),
            error: Some(ResponseError {
                code: CAPABILITY_UNAVAILABLE,
                message: "not available".to_owned(),
            }),
            ..success
        };
        assert_eq!(
            invalid.validate(),
            Err(CompatibilityError::InvalidResponseOutcome)
        );

        Ok(())
    }

    #[test]
    fn graph_event_round_trips_without_request_id() -> anyhow::Result<()> {
        let event = EventMessage::new(Event::RuntimeGraphChanged {
            patch: GraphPatch {
                project_id: EntityId::new("project:drone"),
                upsert_packages: Vec::new(),
                removed_package_ids: Vec::new(),
                upsert_nodes: Vec::new(),
                removed_node_ids: vec![EntityId::new("node:camera")],
            },
        });

        let json = serde_json::to_string(&event)?;
        let decoded: EventMessage = serde_json::from_str(&json)?;

        assert!(json.contains("\"method\":\"runtime_graph_changed\""));
        assert!(!json.contains("\"id\":"));
        assert_eq!(decoded, event);
        decoded.validate()?;

        Ok(())
    }

    #[test]
    fn rejects_incompatible_protocol_versions() {
        let mut request = RequestMessage::new(1, Request::StopRuntimeDiscovery {});
        request.protocol_version += 1;

        assert_eq!(
            request.validate(),
            Err(CompatibilityError::UnsupportedProtocolVersion(2))
        );

        request.protocol_version = PROTOCOL_VERSION;
        request.jsonrpc = "1.0".to_owned();

        assert_eq!(
            request.validate(),
            Err(CompatibilityError::InvalidJsonRpcVersion("1.0".to_owned()))
        );
    }
}
