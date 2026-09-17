use std::collections::BTreeMap;

use ros_studio_model::{
    Confidence, Endpoint, EndpointKind, EntityId, Evidence, EvidenceKind, Node, Package,
    RuntimeState,
};
use ros_studio_protocol::GraphPatch;

const RUNTIME_PACKAGE_ID: &str = "package:runtime";

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuntimeNodeInfo {
    pub name: String,
    pub namespace: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuntimeEndpointInfo {
    pub node_name: String,
    pub node_namespace: String,
    pub name: String,
    pub type_name: String,
    pub kind: EndpointKind,
}

pub struct RuntimeGraph {
    project_id: EntityId,
    nodes: BTreeMap<EntityId, Node>,
    package_announced: bool,
}

impl RuntimeGraph {
    pub fn new(project_id: EntityId) -> Self {
        Self {
            project_id,
            nodes: BTreeMap::new(),
            package_announced: false,
        }
    }

    pub fn update(
        &mut self,
        node_infos: Vec<RuntimeNodeInfo>,
        endpoint_infos: Vec<RuntimeEndpointInfo>,
    ) -> Option<GraphPatch> {
        let mut next_nodes = BTreeMap::new();

        for node_info in node_infos {
            let node = runtime_node(&node_info.name, &node_info.namespace);
            next_nodes.insert(node.id.clone(), node);
        }

        for endpoint_info in endpoint_infos {
            let node = runtime_node(&endpoint_info.node_name, &endpoint_info.node_namespace);
            let entry = next_nodes.entry(node.id.clone()).or_insert(node);
            entry
                .endpoints
                .push(runtime_endpoint(&entry.id, endpoint_info));
        }

        for node in next_nodes.values_mut() {
            node.endpoints.sort();
            node.endpoints.dedup_by(|left, right| left.id == right.id);
        }

        let upsert_nodes = next_nodes
            .iter()
            .filter(|(id, node)| self.nodes.get(*id) != Some(*node))
            .map(|(_, node)| node.clone())
            .collect::<Vec<_>>();
        let removed_node_ids = self
            .nodes
            .keys()
            .filter(|id| !next_nodes.contains_key(*id))
            .cloned()
            .collect::<Vec<_>>();

        let upsert_packages = if self.package_announced {
            Vec::new()
        } else {
            vec![runtime_package()]
        };

        self.nodes = next_nodes;
        self.package_announced = true;

        if upsert_packages.is_empty() && upsert_nodes.is_empty() && removed_node_ids.is_empty() {
            return None;
        }

        Some(GraphPatch {
            project_id: self.project_id.clone(),
            upsert_packages,
            removed_package_ids: Vec::new(),
            upsert_nodes,
            removed_node_ids,
        })
    }

    pub fn clear(&mut self) -> Option<GraphPatch> {
        if !self.package_announced {
            return None;
        }

        let removed_node_ids = self.nodes.keys().cloned().collect();
        self.nodes.clear();
        self.package_announced = false;

        Some(GraphPatch {
            project_id: self.project_id.clone(),
            upsert_packages: Vec::new(),
            removed_package_ids: vec![EntityId::new(RUNTIME_PACKAGE_ID)],
            upsert_nodes: Vec::new(),
            removed_node_ids,
        })
    }
}

pub fn project_id_for_workspace(path: &str) -> Option<EntityId> {
    let name = std::path::Path::new(path).file_name()?.to_str()?;

    if name.is_empty() {
        return None;
    }

    Some(EntityId::new(format!("project:{name}")))
}

fn runtime_package() -> Package {
    Package {
        id: EntityId::new(RUNTIME_PACKAGE_ID),
        name: "Runtime".to_owned(),
        path: String::new(),
    }
}

fn runtime_node(name: &str, namespace: &str) -> Node {
    let full_name = fully_qualified_name(name, namespace);
    let evidence = Evidence {
        kind: EvidenceKind::RuntimeDiscovery,
        source_location: None,
        detail: "rclrs graph discovery".to_owned(),
    };

    Node {
        id: EntityId::new(format!("node:runtime:{full_name}")),
        logical_name: full_name,
        package_id: EntityId::new(RUNTIME_PACKAGE_ID),
        executable: String::new(),
        source_locations: Vec::new(),
        endpoints: Vec::new(),
        evidence: vec![evidence],
        runtime_state: RuntimeState::Online,
    }
}

fn runtime_endpoint(node_id: &EntityId, info: RuntimeEndpointInfo) -> Endpoint {
    let kind_name = match info.kind {
        EndpointKind::Publisher => "publisher",
        EndpointKind::Subscription => "subscription",
        EndpointKind::ServiceClient => "service_client",
        EndpointKind::ServiceServer => "service_server",
        EndpointKind::ActionClient => "action_client",
        EndpointKind::ActionServer => "action_server",
    };

    Endpoint {
        id: EntityId::new(format!(
            "endpoint:{}:{kind_name}:{}:{}",
            node_id.as_str(),
            info.name,
            info.type_name
        )),
        kind: info.kind,
        name: info.name,
        type_name: info.type_name,
        source_location: None,
        confidence: Confidence::RuntimeConfirmed,
        evidence: vec![Evidence {
            kind: EvidenceKind::RuntimeDiscovery,
            source_location: None,
            detail: "rclrs graph discovery".to_owned(),
        }],
    }
}

fn fully_qualified_name(name: &str, namespace: &str) -> String {
    let namespace = namespace.trim_matches('/');
    let name = name.trim_start_matches('/');

    if namespace.is_empty() {
        format!("/{name}")
    } else {
        format!("/{namespace}/{name}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Context as _;

    fn camera_node() -> RuntimeNodeInfo {
        RuntimeNodeInfo {
            name: "camera".to_owned(),
            namespace: "/drone".to_owned(),
        }
    }

    fn camera_publisher() -> RuntimeEndpointInfo {
        RuntimeEndpointInfo {
            node_name: "camera".to_owned(),
            node_namespace: "/drone".to_owned(),
            name: "/camera/image".to_owned(),
            type_name: "sensor_msgs/msg/Image".to_owned(),
            kind: EndpointKind::Publisher,
        }
    }

    #[test]
    fn emits_only_changed_nodes_and_removals() -> anyhow::Result<()> {
        let mut graph = RuntimeGraph::new(EntityId::new("project:drone"));
        let first = graph
            .update(vec![camera_node()], vec![camera_publisher()])
            .context("first snapshot should produce a patch")?;

        assert_eq!(first.project_id.as_str(), "project:drone");
        assert_eq!(first.upsert_packages.len(), 1);
        assert_eq!(first.upsert_nodes.len(), 1);
        assert_eq!(first.upsert_nodes[0].logical_name, "/drone/camera");
        assert_eq!(first.upsert_nodes[0].runtime_state, RuntimeState::Online);
        assert_eq!(first.upsert_nodes[0].endpoints.len(), 1);
        assert_eq!(first.upsert_nodes[0].endpoints[0].name, "/camera/image");

        assert!(
            graph
                .update(vec![camera_node()], vec![camera_publisher()])
                .is_none()
        );

        let changed = graph
            .update(vec![camera_node()], Vec::new())
            .context("endpoint removal should produce a patch")?;
        assert!(changed.upsert_packages.is_empty());
        assert_eq!(changed.upsert_nodes.len(), 1);
        assert!(changed.upsert_nodes[0].endpoints.is_empty());

        let removed = graph
            .update(Vec::new(), Vec::new())
            .context("node removal should produce a patch")?;
        assert!(removed.upsert_nodes.is_empty());
        assert_eq!(removed.removed_node_ids.len(), 1);
        assert_eq!(
            removed.removed_node_ids[0].as_str(),
            "node:runtime:/drone/camera"
        );

        Ok(())
    }

    #[test]
    fn clear_removes_runtime_entities() -> anyhow::Result<()> {
        let mut graph = RuntimeGraph::new(EntityId::new("project:drone"));
        assert!(graph.clear().is_none());

        assert!(
            graph
                .update(vec![camera_node()], vec![camera_publisher()])
                .is_some()
        );
        let patch = graph
            .clear()
            .context("clear should remove runtime entities")?;

        assert_eq!(patch.removed_package_ids[0].as_str(), RUNTIME_PACKAGE_ID);
        assert_eq!(patch.removed_node_ids.len(), 1);
        assert!(graph.clear().is_none());

        Ok(())
    }

    #[test]
    fn workspace_name_matches_static_project_id() {
        assert_eq!(
            project_id_for_workspace("/workspace/drone_demo_ws")
                .as_ref()
                .map(EntityId::as_str),
            Some("project:drone_demo_ws")
        );
        assert!(project_id_for_workspace("/").is_none());
    }
}
