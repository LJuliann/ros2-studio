use anyhow::Context as _;
use ros_studio_model::{
    Confidence, Endpoint, EndpointKind, EntityId, Evidence, EvidenceKind, Node as RosNode, Package,
    RuntimeState, SourceLocation,
};
use tree_sitter::Node;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedRustNode {
    pub variable_name: String,
    pub logical_name: String,
    pub source_location: SourceLocation,
}

pub fn detect_rust_endpoints(
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<DetectedRustEndpoint>> {
    let mut parser = tree_sitter::Parser::new();
    let language = tree_sitter_rust::LANGUAGE.into();

    parser
        .set_language(&language)
        .context("failed to load the Rust grammar")?;

    let tree = parser
        .parse(source, None)
        .context("the Rust parser returned no syntax tree")?;

    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call_expression", &mut calls);

    let mut detected = Vec::new();

    for call in calls {
        let kind = match call_name(call, source) {
            Some("create_publisher") => EndpointKind::Publisher,
            Some("create_subscription") => EndpointKind::Subscription,
            _ => continue,
        };

        let Some(node_variable_name) = call_receiver(call, source) else {
            continue;
        };

        let Some(topic_node) = named_argument(call, 0) else {
            continue;
        };

        let Some(topic_name) = simple_string_literal(topic_node, source) else {
            continue;
        };

        let Some(type_name) = first_type_argument(call, source) else {
            continue;
        };

        detected.push(DetectedRustEndpoint {
            node_variable_name: node_variable_name.to_owned(),
            kind,
            topic_name,
            type_name: type_name.to_owned(),
            source_location: source_location(source_path, topic_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));

    Ok(detected)
}

pub fn scan_rust_source(
    package: &Package,
    executable: &str,
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<RosNode>> {
    let detected_nodes = detect_rust_nodes(source, source_path)?;
    let detected_endpoints = detect_rust_endpoints(source, source_path)?;
    let mut nodes = Vec::with_capacity(detected_nodes.len());

    for detected_node in detected_nodes {
        let mut endpoints = detected_endpoints
            .iter()
            .filter(|endpoint| endpoint.node_variable_name == detected_node.variable_name)
            .map(|endpoint| to_model_endpoint(package, &detected_node, endpoint))
            .collect::<Vec<_>>();

        endpoints.sort();

        let source_location = detected_node.source_location;
        let logical_name = detected_node.logical_name;
        let node_id = EntityId::new(format!(
            "node:{}:{source_path}:{}",
            package.name, detected_node.variable_name
        ));

        nodes.push(RosNode {
            id: node_id,
            logical_name,
            package_id: package.id.clone(),
            executable: executable.to_owned(),
            source_locations: vec![source_location.clone()],
            endpoints,
            evidence: vec![Evidence {
                kind: EvidenceKind::SourceLiteral,
                source_location: Some(source_location),
                detail: "create_node name literal".to_owned(),
            }],
            runtime_state: RuntimeState::Unknown,
        });
    }

    nodes.sort();

    Ok(nodes)
}

fn to_model_endpoint(
    package: &Package,
    node: &DetectedRustNode,
    endpoint: &DetectedRustEndpoint,
) -> Endpoint {
    let source_location = endpoint.source_location.clone();

    let kind_name = match endpoint.kind {
        EndpointKind::Publisher => "publisher",
        EndpointKind::Subscription => "subscription",
        EndpointKind::ServiceClient => "service_client",
        EndpointKind::ServiceServer => "service_server",
        EndpointKind::ActionClient => "action_client",
        EndpointKind::ActionServer => "action_server",
    };

    Endpoint {
        id: EntityId::new(format!(
            "endpoint:{}:{}:{kind_name}:{}",
            package.name, node.logical_name, endpoint.topic_name
        )),
        kind: endpoint.kind,
        name: endpoint.topic_name.clone(),
        type_name: endpoint.type_name.clone(),
        source_location: Some(source_location.clone()),
        confidence: Confidence::ConfirmedSource,
        evidence: vec![Evidence {
            kind: EvidenceKind::SourceLiteral,
            source_location: Some(source_location),
            detail: format!("{kind_name} topic literal"),
        }],
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedRustEndpoint {
    pub node_variable_name: String,
    pub kind: EndpointKind,
    pub topic_name: String,
    pub type_name: String,
    pub source_location: SourceLocation,
}

pub fn detect_rust_nodes(source: &str, source_path: &str) -> anyhow::Result<Vec<DetectedRustNode>> {
    let mut parser = tree_sitter::Parser::new();
    let language = tree_sitter_rust::LANGUAGE.into();

    parser
        .set_language(&language)
        .context("failed to load the Rust grammar")?;

    let tree = parser
        .parse(source, None)
        .context("the Rust parser returned no syntax tree")?;

    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call_expression", &mut calls);

    let mut detected = Vec::new();

    for call in calls {
        if call_name(call, source) != Some("create_node") {
            continue;
        }

        let Some(name_node) = named_argument(call, 1) else {
            continue;
        };

        let Some(logical_name) = simple_string_literal(name_node, source) else {
            continue;
        };

        let Some(variable_name) = enclosing_let_binding(call, source) else {
            continue;
        };

        detected.push(DetectedRustNode {
            variable_name,
            logical_name,
            source_location: source_location(source_path, name_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));

    Ok(detected)
}

fn collect_nodes_of_kind<'tree>(node: Node<'tree>, kind: &str, collected: &mut Vec<Node<'tree>>) {
    if node.kind() == kind {
        collected.push(node);
    }

    let mut cursor = node.walk();

    for child in node.children(&mut cursor) {
        collect_nodes_of_kind(child, kind, collected);
    }
}

fn call_name<'source>(call: Node<'_>, source: &'source str) -> Option<&'source str> {
    let function = callable_expression(call)?;

    let name = match function.kind() {
        "scoped_identifier" => function.child_by_field_name("name")?,
        "field_expression" => function.child_by_field_name("field")?,
        "identifier" => function,
        _ => return None,
    };

    node_text(name, source)
}

fn call_receiver<'source>(call: Node<'_>, source: &'source str) -> Option<&'source str> {
    let function = callable_expression(call)?;

    if function.kind() != "field_expression" {
        return None;
    }

    node_text(function.child_by_field_name("value")?, source)
}

fn first_type_argument<'source>(call: Node<'_>, source: &'source str) -> Option<&'source str> {
    let function = call.child_by_field_name("function")?;

    if function.kind() != "generic_function" {
        return None;
    }

    let type_arguments = function.child_by_field_name("type_arguments")?;

    node_text(type_arguments.named_child(0)?, source)
}

fn callable_expression(call: Node<'_>) -> Option<Node<'_>> {
    let function = call.child_by_field_name("function")?;

    if function.kind() == "generic_function" {
        function.child_by_field_name("function")
    } else {
        Some(function)
    }
}

fn named_argument(call: Node<'_>, index: u32) -> Option<Node<'_>> {
    call.child_by_field_name("arguments")?.named_child(index)
}

fn enclosing_let_binding(call: Node<'_>, source: &str) -> Option<String> {
    let mut ancestor = call.parent();

    while let Some(node) = ancestor {
        if node.kind() == "let_declaration" {
            let pattern = node.child_by_field_name("pattern")?;

            if pattern.kind() == "identifier" {
                return Some(node_text(pattern, source)?.to_owned());
            }

            return None;
        }

        ancestor = node.parent();
    }

    None
}

fn simple_string_literal(node: Node<'_>, source: &str) -> Option<String> {
    if node.kind() != "string_literal" {
        return None;
    }

    let literal = node_text(node, source)?;
    let value = literal.strip_prefix('"')?.strip_suffix('"')?;

    // Les séquences échappées seront prises en charge plus tard.
    // Pour l’instant, on refuse d’inventer une valeur.
    if value.contains('\\') {
        return None;
    }

    Some(value.to_owned())
}

fn node_text<'source>(node: Node<'_>, source: &'source str) -> Option<&'source str> {
    source.get(node.byte_range())
}

fn source_location(path: &str, node: Node<'_>) -> anyhow::Result<SourceLocation> {
    let point = node.start_position();

    Ok(SourceLocation {
        path: path.replace('\\', "/"),
        line: u32::try_from(point.row).context("source line does not fit in u32")?,
        column: u32::try_from(point.column).context("source column does not fit in u32")?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_literal_node_name_and_location() -> anyhow::Result<()> {
        let source = r#"fn main() -> anyhow::Result<()> {
    let context = rclrs::Context::default();
    let mut node = rclrs::create_node(&context, "camera")?;
    Ok(())
}
"#;

        let detected = detect_rust_nodes(source, "src/camera/src/camera.rs")?;

        assert_eq!(
            detected,
            [DetectedRustNode {
                variable_name: "node".to_owned(),
                logical_name: "camera".to_owned(),
                source_location: SourceLocation {
                    path: "src/camera/src/camera.rs".to_owned(),
                    line: 2,
                    column: 48,
                },
            }]
        );

        Ok(())
    }

    #[test]
    fn ignores_dynamic_node_names() -> anyhow::Result<()> {
        let source = r#"let node = rclrs::create_node(&context, node_name)?;"#;

        assert!(detect_rust_nodes(source, "src/main.rs")?.is_empty());

        Ok(())
    }
    #[test]
    fn detects_publishers_and_subscriptions() -> anyhow::Result<()> {
        let source = r#"fn configure(node: &mut rclrs::Node) -> anyhow::Result<()> {
        let publisher = node.create_publisher::<sensor_msgs::msg::Image>(
            "/camera/image",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;
        let subscription = node.create_subscription::<geometry_msgs::msg::Twist, _>(
            "/cmd_vel",
            rclrs::QOS_PROFILE_DEFAULT,
            move |_message| {},
        )?;
        Ok(())
    }
    "#;

        let detected = detect_rust_endpoints(source, "src/camera.rs")?;

        assert_eq!(
            detected,
            [
                DetectedRustEndpoint {
                    node_variable_name: "node".to_owned(),
                    kind: EndpointKind::Publisher,
                    topic_name: "/camera/image".to_owned(),
                    type_name: "sensor_msgs::msg::Image".to_owned(),
                    source_location: SourceLocation {
                        path: "src/camera.rs".to_owned(),
                        line: 2,
                        column: 12,
                    },
                },
                DetectedRustEndpoint {
                    node_variable_name: "node".to_owned(),
                    kind: EndpointKind::Subscription,
                    topic_name: "/cmd_vel".to_owned(),
                    type_name: "geometry_msgs::msg::Twist".to_owned(),
                    source_location: SourceLocation {
                        path: "src/camera.rs".to_owned(),
                        line: 6,
                        column: 12,
                    },
                },
            ]
        );

        Ok(())
    }

    #[test]
    fn ignores_dynamic_topic_names() -> anyhow::Result<()> {
        let source = r#"node.create_publisher::<std_msgs::msg::String>(topic_name, qos)?;"#;

        assert!(detect_rust_endpoints(source, "src/main.rs")?.is_empty());

        Ok(())
    }

    #[test]
    fn builds_model_node_with_its_endpoints() -> anyhow::Result<()> {
        let package = Package {
            id: EntityId::new("package:camera"),
            name: "camera".to_owned(),
            path: "src/camera".to_owned(),
        };

        let source = r#"fn main() -> anyhow::Result<()> {
        let mut node = rclrs::create_node(&context, "camera")?;
        let publisher = node.create_publisher::<sensor_msgs::msg::Image>(
            "/camera/image",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;
        Ok(())
    }
    "#;

        let nodes = scan_rust_source(&package, "camera", source, "src/camera/src/camera.rs")?;

        assert_eq!(nodes.len(), 1);

        let node = &nodes[0];

        assert_eq!(
            node.id.as_str(),
            "node:camera:src/camera/src/camera.rs:node"
        );
        assert_eq!(node.logical_name, "camera");
        assert_eq!(node.package_id, package.id);
        assert_eq!(node.executable, "camera");
        assert_eq!(node.runtime_state, RuntimeState::Unknown);
        assert_eq!(node.endpoints.len(), 1);

        let renamed_source = source.replacen("\"camera\"", "\"renamed_camera\"", 1);
        let renamed_nodes = scan_rust_source(
            &package,
            "camera",
            &renamed_source,
            "src/camera/src/camera.rs",
        )?;
        assert_eq!(renamed_nodes[0].id, node.id);

        let endpoint = &node.endpoints[0];

        assert_eq!(
            endpoint.id.as_str(),
            "endpoint:camera:camera:publisher:/camera/image"
        );
        assert_eq!(endpoint.kind, EndpointKind::Publisher);
        assert_eq!(endpoint.name, "/camera/image");
        assert_eq!(endpoint.type_name, "sensor_msgs::msg::Image");
        assert_eq!(endpoint.confidence, Confidence::ConfirmedSource);

        Ok(())
    }
}
