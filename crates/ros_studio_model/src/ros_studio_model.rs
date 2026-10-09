#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};

pub mod design_live;

#[derive(
    Clone, Copy, Debug, Default, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum Confidence {
    ConfirmedSource,
    Inferred,
    RuntimeConfirmed,
    Conflict,
    #[default]
    Unknown,
}

#[derive(
    Clone, Copy, Debug, Default, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum RuntimeState {
    #[default]
    Unknown,
    Online,
    Offline,
    RuntimeOnly,
    Conflict,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EndpointKind {
    Publisher,
    Subscription,
    ServiceClient,
    ServiceServer,
    ActionClient,
    ActionServer,
}

impl EndpointKind {
    pub fn interface_kind(self) -> InterfaceKind {
        match self {
            Self::Publisher | Self::Subscription => InterfaceKind::Topic,
            Self::ServiceClient | Self::ServiceServer => InterfaceKind::Service,
            Self::ActionClient | Self::ActionServer => InterfaceKind::Action,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum InterfaceKind {
    Topic,
    Service,
    Action,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceKind {
    SourceLiteral,
    LaunchRemapping,
    Configuration,
    RuntimeDiscovery,
    Heuristic,
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct Evidence {
    pub kind: EvidenceKind,
    pub source_location: Option<SourceLocation>,
    pub detail: String,
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct Endpoint {
    pub id: EntityId,
    pub kind: EndpointKind,
    pub name: String,
    pub type_name: String,
    pub source_location: Option<SourceLocation>,
    pub confidence: Confidence,
    pub evidence: Vec<Evidence>,
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct Package {
    pub id: EntityId,
    pub name: String,
    pub path: String,
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct Node {
    pub id: EntityId,
    pub logical_name: String,
    pub package_id: EntityId,
    pub executable: String,
    pub source_locations: Vec<SourceLocation>,
    pub endpoints: Vec<Endpoint>,
    pub evidence: Vec<Evidence>,
    pub runtime_state: RuntimeState,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Project {
    pub id: EntityId,
    pub name: String,
    pub root_path: String,
    pub packages: Vec<Package>,
    pub nodes: Vec<Node>,
}

impl Project {
    pub fn sort_deterministically(&mut self) {
        self.packages.sort();

        for node in &mut self.nodes {
            node.source_locations.sort();
            node.evidence.sort();

            for endpoint in &mut node.endpoints {
                endpoint.evidence.sort();
            }

            node.endpoints.sort();
        }

        self.nodes.sort();
    }
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(transparent)]
pub struct EntityId(String);

impl EntityId {
    pub fn new(value: impl Into<String>) -> Self {
        Self(value.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct SourceLocation {
    /// Workspace-relative path using `/` separators.
    pub path: String,
    /// Zero-based line index.
    pub line: u32,
    /// Zero-based UTF-8 byte offset within the line.
    pub column: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_location_serializes_deterministically() -> Result<(), serde_json::Error> {
        let location = SourceLocation {
            path: "src/navigation.rs".to_owned(),
            line: 12,
            column: 4,
        };

        let json = serde_json::to_string(&location)?;

        assert_eq!(json, r#"{"path":"src/navigation.rs","line":12,"column":4}"#);

        let decoded: SourceLocation = serde_json::from_str(&json)?;
        assert_eq!(decoded, location);

        Ok(())
    }

    #[test]
    fn entity_id_serializes_as_a_string() -> Result<(), serde_json::Error> {
        let id = EntityId::new("node:camera");

        let json = serde_json::to_string(&id)?;
        assert_eq!(json, r#""node:camera""#);

        let decoded: EntityId = serde_json::from_str(&json)?;
        assert_eq!(decoded, id);

        Ok(())
    }

    #[test]
    fn confidence_uses_stable_json_names() -> Result<(), serde_json::Error> {
        let values = [
            Confidence::ConfirmedSource,
            Confidence::Inferred,
            Confidence::RuntimeConfirmed,
            Confidence::Conflict,
            Confidence::Unknown,
        ];

        let json = serde_json::to_string(&values)?;

        assert_eq!(
            json,
            r#"["confirmed_source","inferred","runtime_confirmed","conflict","unknown"]"#
        );
        assert_eq!(Confidence::default(), Confidence::Unknown);

        Ok(())
    }

    #[test]
    fn runtime_state_uses_stable_json_names() -> Result<(), serde_json::Error> {
        let values = [
            RuntimeState::Online,
            RuntimeState::Offline,
            RuntimeState::RuntimeOnly,
            RuntimeState::Conflict,
            RuntimeState::Unknown,
        ];

        let json = serde_json::to_string(&values)?;

        assert_eq!(
            json,
            r#"["online","offline","runtime_only","conflict","unknown"]"#
        );
        assert_eq!(RuntimeState::default(), RuntimeState::Unknown);

        Ok(())
    }

    #[test]
    fn interface_kind_uses_stable_json_names() -> Result<(), serde_json::Error> {
        let values = [
            InterfaceKind::Topic,
            InterfaceKind::Service,
            InterfaceKind::Action,
        ];

        let json = serde_json::to_string(&values)?;

        assert_eq!(json, r#"["topic","service","action"]"#);
        Ok(())
    }

    #[test]
    fn endpoint_kind_uses_stable_json_names() -> Result<(), serde_json::Error> {
        let values = [
            EndpointKind::Publisher,
            EndpointKind::Subscription,
            EndpointKind::ServiceClient,
            EndpointKind::ServiceServer,
            EndpointKind::ActionClient,
            EndpointKind::ActionServer,
        ];

        let json = serde_json::to_string(&values)?;
        assert_eq!(
            json,
            r#"["publisher","subscription","service_client","service_server","action_client","action_server"]"#
        );
        Ok(())
    }

    #[test]
    fn evidence_preserves_its_origin() -> Result<(), serde_json::Error> {
        let evidence = Evidence {
            kind: EvidenceKind::SourceLiteral,
            source_location: Some(SourceLocation {
                path: "src/navigation.rs".to_owned(),
                line: 12,
                column: 4,
            }),
            detail: "create_subscription topic literal".to_owned(),
        };

        let json = serde_json::to_string(&evidence)?;

        assert_eq!(
            json,
            r#"{"kind":"source_literal","source_location":{"path":"src/navigation.rs","line":12,"column":4},"detail":"create_subscription topic literal"}"#
        );

        let decoded: Evidence = serde_json::from_str(&json)?;
        assert_eq!(decoded, evidence);

        Ok(())
    }

    #[test]
    fn endpoint_round_trips_through_json() -> Result<(), serde_json::Error> {
        let endpoint = Endpoint {
            id: EntityId::new("endpoint:camera:image_raw:publisher"),
            kind: EndpointKind::Publisher,
            name: "/camera/image_raw".to_owned(),
            type_name: "sensor_msgs/msg/Image".to_owned(),
            source_location: Some(SourceLocation {
                path: "src/camera.rs".to_owned(),
                line: 20,
                column: 8,
            }),
            confidence: Confidence::ConfirmedSource,
            evidence: vec![Evidence {
                kind: EvidenceKind::SourceLiteral,
                source_location: None,
                detail: "publisher created from a string literal".to_owned(),
            }],
        };

        let json = serde_json::to_string(&endpoint)?;
        let decoded: Endpoint = serde_json::from_str(&json)?;

        assert_eq!(decoded, endpoint);

        Ok(())
    }

    #[test]
    fn endpoint_kind_determines_interface_kind() {
        assert_eq!(
            EndpointKind::Publisher.interface_kind(),
            InterfaceKind::Topic
        );
        assert_eq!(
            EndpointKind::Subscription.interface_kind(),
            InterfaceKind::Topic
        );
        assert_eq!(
            EndpointKind::ServiceClient.interface_kind(),
            InterfaceKind::Service
        );
        assert_eq!(
            EndpointKind::ServiceServer.interface_kind(),
            InterfaceKind::Service
        );
        assert_eq!(
            EndpointKind::ActionClient.interface_kind(),
            InterfaceKind::Action
        );
        assert_eq!(
            EndpointKind::ActionServer.interface_kind(),
            InterfaceKind::Action
        );
    }

    #[test]
    fn project_sort_is_deterministic() -> Result<(), serde_json::Error> {
        let mut project = Project {
            id: EntityId::new("project:demo"),
            name: "Demo".to_owned(),
            root_path: "/workspace/demo".to_owned(),
            packages: vec![
                Package {
                    id: EntityId::new("package:navigation"),
                    name: "navigation".to_owned(),
                    path: "src/navigation".to_owned(),
                },
                Package {
                    id: EntityId::new("package:camera"),
                    name: "camera".to_owned(),
                    path: "src/camera".to_owned(),
                },
            ],
            nodes: vec![
                Node {
                    id: EntityId::new("node:navigation"),
                    logical_name: "/navigation".to_owned(),
                    package_id: EntityId::new("package:navigation"),
                    executable: "navigation".to_owned(),
                    source_locations: Vec::new(),
                    endpoints: Vec::new(),
                    evidence: Vec::new(),
                    runtime_state: RuntimeState::Unknown,
                },
                Node {
                    id: EntityId::new("node:camera"),
                    logical_name: "/camera".to_owned(),
                    package_id: EntityId::new("package:camera"),
                    executable: "camera".to_owned(),
                    source_locations: Vec::new(),
                    endpoints: Vec::new(),
                    evidence: Vec::new(),
                    runtime_state: RuntimeState::Unknown,
                },
            ],
        };

        project.sort_deterministically();

        let package_ids = project
            .packages
            .iter()
            .map(|package| package.id.as_str())
            .collect::<Vec<_>>();

        let node_ids = project
            .nodes
            .iter()
            .map(|node| node.id.as_str())
            .collect::<Vec<_>>();

        assert_eq!(package_ids, ["package:camera", "package:navigation"]);
        assert_eq!(node_ids, ["node:camera", "node:navigation"]);

        let json = serde_json::to_string(&project)?;
        let decoded: Project = serde_json::from_str(&json)?;
        assert_eq!(decoded, project);

        Ok(())
    }
}
