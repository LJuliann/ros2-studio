use std::collections::{BTreeMap, BTreeSet};

use crate::{
    Confidence, Endpoint, EndpointKind, Evidence, EvidenceKind, Node, Project, RuntimeState,
};

pub fn reconcile_project(design: &Project, runtime: Option<&Project>) -> Project {
    let mut merged = design.clone();
    let Some(runtime) = runtime.filter(|runtime| runtime.id == design.id) else {
        return merged;
    };

    let design_names = node_indices_by_name(&design.nodes);
    let runtime_names = node_indices_by_name(&runtime.nodes);
    let mut matched_runtime_indices = BTreeSet::new();

    for design_node in &mut merged.nodes {
        let name = normalized_node_name(&design_node.logical_name);
        match runtime_names.get(&name) {
            None => design_node.runtime_state = RuntimeState::Offline,
            Some(runtime_indices)
                if runtime_indices.len() == 1
                    && design_names
                        .get(&name)
                        .is_some_and(|indices| indices.len() == 1) =>
            {
                let runtime_index = runtime_indices[0];
                merge_node(design_node, &runtime.nodes[runtime_index]);
                matched_runtime_indices.insert(runtime_index);
            }
            Some(_) => design_node.runtime_state = RuntimeState::Unknown,
        }
    }

    for (runtime_index, runtime_node) in runtime.nodes.iter().enumerate() {
        if matched_runtime_indices.contains(&runtime_index) {
            continue;
        }

        if !merged
            .packages
            .iter()
            .any(|package| package.id == runtime_node.package_id)
            && let Some(package) = runtime
                .packages
                .iter()
                .find(|package| package.id == runtime_node.package_id)
        {
            merged.packages.push(package.clone());
        }

        let mut runtime_only_node = runtime_node.clone();
        runtime_only_node.runtime_state = RuntimeState::RuntimeOnly;
        merged.nodes.push(runtime_only_node);
    }

    merged.sort_deterministically();
    merged
}

fn node_indices_by_name(nodes: &[Node]) -> BTreeMap<String, Vec<usize>> {
    let mut indices = BTreeMap::<String, Vec<usize>>::new();
    for (index, node) in nodes.iter().enumerate() {
        indices
            .entry(normalized_node_name(&node.logical_name))
            .or_default()
            .push(index);
    }
    indices
}

fn normalized_node_name(name: &str) -> String {
    let name = name.trim().trim_matches('/');
    format!("/{name}")
}

fn merge_node(design_node: &mut Node, runtime_node: &Node) {
    append_unique_evidence(&mut design_node.evidence, &runtime_node.evidence);

    let design_endpoints = endpoint_indices_by_key(&design_node.endpoints);
    let runtime_endpoints = endpoint_indices_by_key(&runtime_node.endpoints);
    let mut matched_runtime_indices = BTreeSet::new();

    for design_endpoint in &mut design_node.endpoints {
        let key = (design_endpoint.kind, design_endpoint.name.clone());
        if !design_endpoints
            .get(&key)
            .is_some_and(|indices| indices.len() == 1)
        {
            continue;
        }

        let Some(runtime_indices) = runtime_endpoints
            .get(&key)
            .filter(|indices| indices.len() == 1)
        else {
            continue;
        };

        let runtime_index = runtime_indices[0];
        merge_endpoint(design_endpoint, &runtime_node.endpoints[runtime_index]);
        matched_runtime_indices.insert(runtime_index);
    }

    for (runtime_index, runtime_endpoint) in runtime_node.endpoints.iter().enumerate() {
        if !matched_runtime_indices.contains(&runtime_index) {
            design_node.endpoints.push(runtime_endpoint.clone());
        }
    }

    design_node.runtime_state = if design_node
        .endpoints
        .iter()
        .any(|endpoint| endpoint.confidence == Confidence::Conflict)
    {
        RuntimeState::Conflict
    } else {
        RuntimeState::Online
    };
}

fn endpoint_indices_by_key(endpoints: &[Endpoint]) -> BTreeMap<(EndpointKind, String), Vec<usize>> {
    let mut indices = BTreeMap::<(EndpointKind, String), Vec<usize>>::new();
    for (index, endpoint) in endpoints.iter().enumerate() {
        indices
            .entry((endpoint.kind, endpoint.name.clone()))
            .or_default()
            .push(index);
    }
    indices
}

fn merge_endpoint(design_endpoint: &mut Endpoint, runtime_endpoint: &Endpoint) {
    append_unique_evidence(&mut design_endpoint.evidence, &runtime_endpoint.evidence);

    if types_conflict(&design_endpoint.type_name, &runtime_endpoint.type_name) {
        design_endpoint.confidence = Confidence::Conflict;
        design_endpoint.evidence.push(Evidence {
            kind: EvidenceKind::RuntimeDiscovery,
            source_location: None,
            detail: format!(
                "runtime type {} differs from design type {}",
                runtime_endpoint.type_name, design_endpoint.type_name
            ),
        });
    } else if design_endpoint.confidence != Confidence::Conflict {
        design_endpoint.confidence = Confidence::RuntimeConfirmed;
    }
}

fn append_unique_evidence(destination: &mut Vec<Evidence>, additional: &[Evidence]) {
    for evidence in additional {
        if !destination.contains(evidence) {
            destination.push(evidence.clone());
        }
    }
}

fn types_conflict(design_type: &str, runtime_type: &str) -> bool {
    let design_type = canonical_ros_type_name(design_type);
    let runtime_type = canonical_ros_type_name(runtime_type);
    is_qualified_ros_type(&design_type)
        && is_qualified_ros_type(&runtime_type)
        && design_type != runtime_type
}

pub fn canonical_ros_type_name(type_name: &str) -> String {
    type_name
        .trim()
        .replace("::", "/")
        .replace(".msg.", "/msg/")
        .replace(".srv.", "/srv/")
        .replace(".action.", "/action/")
}

fn is_qualified_ros_type(type_name: &str) -> bool {
    let mut parts = type_name.split('/');
    matches!(
        (parts.next(), parts.next(), parts.next(), parts.next()),
        (Some(package), Some("msg" | "srv" | "action"), Some(name), None)
            if !package.is_empty() && !name.is_empty()
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EntityId, Package, SourceLocation};

    fn project(nodes: Vec<Node>) -> Project {
        Project {
            id: EntityId::new("project:drone"),
            name: "drone".to_owned(),
            root_path: "/workspace/drone".to_owned(),
            packages: vec![Package {
                id: EntityId::new("package:camera"),
                name: "camera".to_owned(),
                path: "src/camera".to_owned(),
            }],
            nodes,
        }
    }

    fn node(id: &str, name: &str, endpoint_list: Vec<Endpoint>) -> Node {
        Node {
            id: EntityId::new(id),
            logical_name: name.to_owned(),
            package_id: EntityId::new("package:camera"),
            executable: "camera".to_owned(),
            source_locations: vec![SourceLocation {
                path: "src/camera.rs".to_owned(),
                line: 4,
                column: 2,
            }],
            endpoints: endpoint_list,
            evidence: Vec::new(),
            runtime_state: RuntimeState::Unknown,
        }
    }

    fn endpoint(id: &str, name: &str, type_name: &str) -> Endpoint {
        Endpoint {
            id: EntityId::new(id),
            kind: EndpointKind::Publisher,
            name: name.to_owned(),
            type_name: type_name.to_owned(),
            source_location: None,
            confidence: Confidence::ConfirmedSource,
            evidence: Vec::new(),
        }
    }

    fn runtime_node(name: &str, endpoints: Vec<Endpoint>) -> Node {
        let mut runtime_node = node(
            &format!("node:runtime:{}", normalized_node_name(name)),
            name,
            endpoints,
        );
        runtime_node.package_id = EntityId::new("package:runtime");
        runtime_node.executable.clear();
        runtime_node.source_locations.clear();
        runtime_node.evidence = vec![Evidence {
            kind: EvidenceKind::RuntimeDiscovery,
            source_location: None,
            detail: "runtime node".to_owned(),
        }];
        for endpoint in &mut runtime_node.endpoints {
            endpoint.confidence = Confidence::RuntimeConfirmed;
            endpoint.evidence = vec![Evidence {
                kind: EvidenceKind::RuntimeDiscovery,
                source_location: None,
                detail: "runtime endpoint".to_owned(),
            }];
        }
        runtime_node.runtime_state = RuntimeState::Online;
        runtime_node
    }

    fn runtime_project(nodes: Vec<Node>) -> Project {
        let mut runtime = project(nodes);
        runtime.packages = vec![Package {
            id: EntityId::new("package:runtime"),
            name: "Runtime".to_owned(),
            path: String::new(),
        }];
        runtime
    }

    #[test]
    fn waits_for_a_runtime_snapshot_before_marking_nodes_offline() {
        let design = project(vec![node("node:camera", "camera", Vec::new())]);
        assert_eq!(reconcile_project(&design, None), design);

        let merged = reconcile_project(&design, Some(&runtime_project(Vec::new())));
        assert_eq!(merged.nodes[0].runtime_state, RuntimeState::Offline);
        assert_eq!(merged.nodes[0].id.as_str(), "node:camera");
    }

    #[test]
    fn matches_equivalent_node_and_endpoint_names_without_losing_source_identity() {
        let design = project(vec![node(
            "node:camera:source",
            "camera",
            vec![endpoint(
                "endpoint:design:image",
                "/camera/image",
                "sensor_msgs::msg::Image",
            )],
        )]);
        let runtime = runtime_project(vec![runtime_node(
            "/camera",
            vec![endpoint(
                "endpoint:runtime:image",
                "/camera/image",
                "sensor_msgs/msg/Image",
            )],
        )]);

        let merged = reconcile_project(&design, Some(&runtime));
        assert_eq!(merged.nodes.len(), 1);
        assert_eq!(merged.nodes[0].id, design.nodes[0].id);
        assert_eq!(
            merged.nodes[0].source_locations,
            design.nodes[0].source_locations
        );
        assert_eq!(merged.nodes[0].runtime_state, RuntimeState::Online);
        assert_eq!(merged.nodes[0].evidence.len(), 1);
        assert_eq!(merged.nodes[0].endpoints.len(), 1);
        assert_eq!(
            merged.nodes[0].endpoints[0].id.as_str(),
            "endpoint:design:image"
        );
        assert_eq!(
            merged.nodes[0].endpoints[0].confidence,
            Confidence::RuntimeConfirmed
        );
        assert_eq!(merged.nodes[0].endpoints[0].evidence.len(), 1);
        assert_eq!(merged.packages, design.packages);
    }

    #[test]
    fn retains_runtime_only_nodes_and_their_package() {
        let design = project(Vec::new());
        let runtime = runtime_project(vec![runtime_node("/camera", Vec::new())]);

        let merged = reconcile_project(&design, Some(&runtime));
        assert_eq!(merged.nodes.len(), 1);
        assert_eq!(merged.nodes[0].runtime_state, RuntimeState::RuntimeOnly);
        assert_eq!(merged.nodes[0].id.as_str(), "node:runtime:/camera");
        assert_eq!(merged.packages.len(), 2);
        assert_eq!(merged.packages[1].id.as_str(), "package:runtime");
    }

    #[test]
    fn reports_conflicts_between_different_qualified_types() {
        let design = project(vec![node(
            "node:camera",
            "camera",
            vec![endpoint(
                "endpoint:design:image",
                "/camera/image",
                "sensor_msgs::msg::Image",
            )],
        )]);
        let runtime = runtime_project(vec![runtime_node(
            "/camera",
            vec![endpoint(
                "endpoint:runtime:image",
                "/camera/image",
                "std_msgs/msg/String",
            )],
        )]);

        let merged = reconcile_project(&design, Some(&runtime));
        let merged_node = &merged.nodes[0];
        assert_eq!(merged_node.runtime_state, RuntimeState::Conflict);
        assert_eq!(merged_node.endpoints[0].confidence, Confidence::Conflict);
        assert!(merged_node.endpoints[0].evidence.iter().any(|evidence| {
            evidence.detail.contains("std_msgs/msg/String")
                && evidence.detail.contains("sensor_msgs::msg::Image")
        }));
    }

    #[test]
    fn keeps_design_only_and_runtime_only_endpoints() {
        let design = project(vec![node(
            "node:camera",
            "camera",
            vec![endpoint("endpoint:design:image", "/camera/image", "Image")],
        )]);
        let runtime = runtime_project(vec![runtime_node(
            "/camera",
            vec![endpoint(
                "endpoint:runtime:status",
                "/camera/status",
                "std_msgs/msg/String",
            )],
        )]);

        let merged = reconcile_project(&design, Some(&runtime));
        assert_eq!(merged.nodes[0].runtime_state, RuntimeState::Online);
        assert_eq!(merged.nodes[0].endpoints.len(), 2);
        assert!(merged.nodes[0].endpoints.iter().any(|endpoint| {
            endpoint.id.as_str() == "endpoint:design:image"
                && endpoint.confidence == Confidence::ConfirmedSource
        }));
        assert!(
            merged.nodes[0]
                .endpoints
                .iter()
                .any(|endpoint| endpoint.id.as_str() == "endpoint:runtime:status")
        );
    }

    #[test]
    fn does_not_guess_which_duplicate_design_node_is_live() {
        let design = project(vec![
            node("node:camera:first", "camera", Vec::new()),
            node("node:camera:second", "camera", Vec::new()),
        ]);
        let runtime = runtime_project(vec![runtime_node("/camera", Vec::new())]);

        let merged = reconcile_project(&design, Some(&runtime));
        assert_eq!(merged.nodes.len(), 3);
        assert_eq!(
            merged
                .nodes
                .iter()
                .filter(|node| node.runtime_state == RuntimeState::Unknown)
                .count(),
            2
        );
        assert_eq!(
            merged
                .nodes
                .iter()
                .filter(|node| node.runtime_state == RuntimeState::RuntimeOnly)
                .count(),
            1
        );
    }

    #[test]
    fn ignores_runtime_snapshots_from_another_project() {
        let design = project(vec![node("node:camera", "camera", Vec::new())]);
        let mut runtime = runtime_project(Vec::new());
        runtime.id = EntityId::new("project:other");

        assert_eq!(reconcile_project(&design, Some(&runtime)), design);
    }

    #[test]
    fn only_compares_fully_qualified_ros_types() {
        assert!(!types_conflict(
            "sensor_msgs::msg::Image",
            "sensor_msgs/msg/Image"
        ));
        assert!(!types_conflict(
            "sensor_msgs.msg.Image",
            "sensor_msgs/msg/Image"
        ));
        assert!(!types_conflict("Image", "sensor_msgs/msg/Image"));
        assert!(types_conflict(
            "sensor_msgs/msg/Image",
            "std_msgs/msg/String"
        ));
    }
}
