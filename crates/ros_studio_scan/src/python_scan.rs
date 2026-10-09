use anyhow::Context as _;
use ros_studio_model::{
    Confidence, Endpoint, EndpointKind, EntityId, Evidence, EvidenceKind, Node as RosNode, Package,
    RuntimeState, SourceLocation,
};
use tree_sitter::Node;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedPythonNode {
    pub variable_name: String,
    pub scope_name: Option<String>,
    pub logical_name: String,
    pub source_location: SourceLocation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedPythonEndpoint {
    pub node_variable_name: String,
    pub scope_name: Option<String>,
    pub kind: EndpointKind,
    pub topic_name: String,
    pub type_name: String,
    pub source_location: SourceLocation,
}

pub fn detect_python_nodes(
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<DetectedPythonNode>> {
    let tree = parse_python(source)?;
    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call", &mut calls);
    let mut detected = Vec::new();

    for call in calls {
        let Some(function) = call.child_by_field_name("function") else {
            continue;
        };
        let Some(function_text) = node_text(function, source) else {
            continue;
        };
        let scope_name = enclosing_class_name(call, source);
        let (variable_name, name_argument_index) = if function_text.ends_with("super().__init__")
            || function_text.ends_with("Node.__init__")
        {
            ("self".to_owned(), 0)
        } else if function_text == "Node"
            || function_text.ends_with(".Node")
            || function_text.ends_with(".create_node")
        {
            let Some(variable_name) = enclosing_assignment_name(call, source) else {
                continue;
            };
            (variable_name, 0)
        } else {
            continue;
        };
        let Some(name_node) = call_argument(call, name_argument_index) else {
            continue;
        };
        let Some(logical_name) = simple_string_literal(name_node, source) else {
            continue;
        };

        detected.push(DetectedPythonNode {
            variable_name,
            scope_name,
            logical_name,
            source_location: source_location(source_path, name_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));
    Ok(detected)
}

pub fn detect_python_endpoints(
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<DetectedPythonEndpoint>> {
    let tree = parse_python(source)?;
    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call", &mut calls);
    let mut detected = Vec::new();

    for call in calls {
        let Some(function) = call.child_by_field_name("function") else {
            continue;
        };
        if function.kind() != "attribute" {
            continue;
        }
        let Some(method) = function.child_by_field_name("attribute") else {
            continue;
        };
        let kind = match node_text(method, source) {
            Some("create_publisher") => EndpointKind::Publisher,
            Some("create_subscription") => EndpointKind::Subscription,
            _ => continue,
        };
        let Some(receiver) = function.child_by_field_name("object") else {
            continue;
        };
        let Some(type_node) = call_argument(call, 0) else {
            continue;
        };
        let Some(topic_node) = call_argument(call, 1) else {
            continue;
        };
        let Some(type_name) = node_text(type_node, source) else {
            continue;
        };
        let Some(topic_name) = simple_string_literal(topic_node, source) else {
            continue;
        };

        detected.push(DetectedPythonEndpoint {
            node_variable_name: node_text(receiver, source).unwrap_or_default().to_owned(),
            scope_name: enclosing_class_name(call, source),
            kind,
            topic_name,
            type_name: type_name.to_owned(),
            source_location: source_location(source_path, topic_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));
    Ok(detected)
}

pub fn scan_python_source(
    package: &Package,
    executable: &str,
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<RosNode>> {
    let detected_nodes = detect_python_nodes(source, source_path)?;
    let detected_endpoints = detect_python_endpoints(source, source_path)?;
    let mut nodes = Vec::with_capacity(detected_nodes.len());

    for detected_node in detected_nodes {
        let mut endpoints = detected_endpoints
            .iter()
            .filter(|endpoint| {
                endpoint.node_variable_name == detected_node.variable_name
                    && endpoint.scope_name == detected_node.scope_name
            })
            .map(|endpoint| to_model_endpoint(package, &detected_node, endpoint))
            .collect::<Vec<_>>();
        endpoints.sort();

        let source_location = detected_node.source_location;
        let logical_name = detected_node.logical_name;
        let scope_name = detected_node.scope_name.as_deref().unwrap_or("module");
        let node_id = EntityId::new(format!(
            "node:{}:{source_path}:{scope_name}:{}",
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
                detail: "rclpy node name literal".to_owned(),
            }],
            runtime_state: RuntimeState::Unknown,
        });
    }

    nodes.sort();
    Ok(nodes)
}

fn to_model_endpoint(
    package: &Package,
    node: &DetectedPythonNode,
    endpoint: &DetectedPythonEndpoint,
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
            detail: format!("rclpy {kind_name} topic literal"),
        }],
    }
}

fn parse_python(source: &str) -> anyhow::Result<tree_sitter::Tree> {
    let mut parser = tree_sitter::Parser::new();
    parser
        .set_language(&tree_sitter_python::LANGUAGE.into())
        .context("failed to load the Python grammar")?;
    parser
        .parse(source, None)
        .context("the Python parser returned no syntax tree")
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

fn call_argument(call: Node<'_>, index: u32) -> Option<Node<'_>> {
    call.child_by_field_name("arguments")?.named_child(index)
}

fn enclosing_assignment_name(call: Node<'_>, source: &str) -> Option<String> {
    let mut ancestor = call.parent();
    while let Some(node) = ancestor {
        if node.kind() == "assignment" {
            let left = node.child_by_field_name("left")?;
            if left.kind() == "identifier" {
                return Some(node_text(left, source)?.to_owned());
            }
            return None;
        }
        ancestor = node.parent();
    }
    None
}

fn enclosing_class_name(node: Node<'_>, source: &str) -> Option<String> {
    let mut ancestor = node.parent();
    while let Some(node) = ancestor {
        if node.kind() == "class_definition" {
            return node
                .child_by_field_name("name")
                .and_then(|name| node_text(name, source))
                .map(str::to_owned);
        }
        ancestor = node.parent();
    }
    None
}

fn simple_string_literal(node: Node<'_>, source: &str) -> Option<String> {
    if node.kind() != "string" {
        return None;
    }
    let literal = node_text(node, source)?;
    let quote = literal.chars().next()?;
    if !matches!(quote, '\'' | '"') || !literal.ends_with(quote) {
        return None;
    }
    let value = literal.strip_prefix(quote)?.strip_suffix(quote)?;
    (!value.contains('\\')).then(|| value.to_owned())
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
    fn scans_rclpy_class_node_and_topics() -> anyhow::Result<()> {
        let source = r#"class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher = self.create_publisher(Image, '/camera/image', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
"#;
        let package = Package {
            id: EntityId::new("package:camera_py"),
            name: "camera_py".to_owned(),
            path: "src/camera_py".to_owned(),
        };

        let nodes = scan_python_source(&package, "camera_py", source, "camera.py")?;

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].logical_name, "camera");
        assert_eq!(nodes[0].endpoints.len(), 2);
        assert_eq!(nodes[0].endpoints[0].name, "/camera/image");
        assert_eq!(nodes[0].endpoints[0].type_name, "Image");
        assert_eq!(nodes[0].endpoints[1].name, "/cmd_vel");
        assert_eq!(nodes[0].endpoints[1].type_name, "Twist");

        let renamed_source = source.replacen("'camera'", "'renamed_camera'", 1);
        let renamed_nodes =
            scan_python_source(&package, "camera_py", &renamed_source, "camera.py")?;
        assert_eq!(renamed_nodes[0].id, nodes[0].id);

        Ok(())
    }

    #[test]
    fn ignores_dynamic_python_names_and_topics() -> anyhow::Result<()> {
        let source =
            "node = rclpy.create_node(node_name)\nnode.create_publisher(String, topic_name, 10)";

        assert!(detect_python_nodes(source, "node.py")?.is_empty());
        assert!(detect_python_endpoints(source, "node.py")?.is_empty());

        Ok(())
    }
}
