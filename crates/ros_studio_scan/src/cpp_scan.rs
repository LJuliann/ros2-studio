use anyhow::Context as _;
use ros_studio_model::{
    Confidence, Endpoint, EndpointKind, EntityId, Evidence, EvidenceKind, Node as RosNode, Package,
    RuntimeState, SourceLocation,
};
use tree_sitter::Node;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedCppNode {
    pub variable_name: String,
    pub scope_name: Option<String>,
    pub logical_name: String,
    pub source_location: SourceLocation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedCppEndpoint {
    pub node_variable_name: String,
    pub scope_name: Option<String>,
    pub kind: EndpointKind,
    pub topic_name: String,
    pub type_name: String,
    pub source_location: SourceLocation,
}

pub fn detect_cpp_nodes(source: &str, source_path: &str) -> anyhow::Result<Vec<DetectedCppNode>> {
    let tree = parse_cpp(source)?;
    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call_expression", &mut calls);
    let mut detected = Vec::new();

    for call in calls {
        let Some(function) = call.child_by_field_name("function") else {
            continue;
        };
        let Some(function_text) = node_text(function, source) else {
            continue;
        };
        let scope_name = enclosing_class_name(call, source);
        let variable_name = if function_text.ends_with("Node::make_shared")
            || function_text.ends_with("rclcpp::create_node")
            || is_rclcpp_make_shared(function_text)
        {
            let Some(variable_name) = enclosing_declarator_name(call, source) else {
                continue;
            };
            variable_name
        } else if scope_name.is_some()
            && (function_text == "Node" || function_text.ends_with("::Node"))
        {
            "this".to_owned()
        } else {
            continue;
        };
        let Some(name_node) = call_argument(call, 0) else {
            continue;
        };
        let Some(logical_name) = simple_string_literal(name_node, source) else {
            continue;
        };

        detected.push(DetectedCppNode {
            variable_name,
            scope_name,
            logical_name,
            source_location: source_location(source_path, name_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));
    Ok(detected)
}

fn is_rclcpp_make_shared(function: &str) -> bool {
    function.ends_with("make_shared<rclcpp::Node>")
}

pub fn detect_cpp_endpoints(
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<DetectedCppEndpoint>> {
    let tree = parse_cpp(source)?;
    let mut calls = Vec::new();
    collect_nodes_of_kind(tree.root_node(), "call_expression", &mut calls);
    let mut detected = Vec::new();

    for call in calls {
        let Some(function) = call.child_by_field_name("function") else {
            continue;
        };
        let Some(function_text) = node_text(function, source) else {
            continue;
        };
        let Some((node_variable_name, kind, type_name)) = parse_endpoint_function(function_text)
        else {
            continue;
        };
        let Some(topic_node) = call_argument(call, 0) else {
            continue;
        };
        let Some(topic_name) = simple_string_literal(topic_node, source) else {
            continue;
        };

        detected.push(DetectedCppEndpoint {
            node_variable_name,
            scope_name: enclosing_class_name(call, source),
            kind,
            topic_name,
            type_name,
            source_location: source_location(source_path, topic_node)?,
        });
    }

    detected.sort_by(|left, right| left.source_location.cmp(&right.source_location));
    Ok(detected)
}

pub fn scan_cpp_source(
    package: &Package,
    executable: &str,
    source: &str,
    source_path: &str,
) -> anyhow::Result<Vec<RosNode>> {
    let detected_nodes = detect_cpp_nodes(source, source_path)?;
    let detected_endpoints = detect_cpp_endpoints(source, source_path)?;
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
        let scope_name = detected_node.scope_name.as_deref().unwrap_or("global");
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
                detail: "rclcpp node name literal".to_owned(),
            }],
            runtime_state: RuntimeState::Unknown,
        });
    }

    nodes.sort();
    Ok(nodes)
}

fn parse_endpoint_function(function: &str) -> Option<(String, EndpointKind, String)> {
    let (method_name, kind) = if function.contains("create_publisher") {
        ("create_publisher", EndpointKind::Publisher)
    } else if function.contains("create_subscription") {
        ("create_subscription", EndpointKind::Subscription)
    } else {
        return None;
    };
    let method_start = function.rfind(method_name)?;
    let receiver_prefix = function[..method_start]
        .trim_end_matches(['-', '>', '.'])
        .trim();
    let node_variable_name = if receiver_prefix.is_empty() {
        "this".to_owned()
    } else {
        receiver_prefix
            .rsplit("::")
            .next()
            .unwrap_or(receiver_prefix)
            .to_owned()
    };
    let type_start = function[method_start + method_name.len()..].find('<')?
        + method_start
        + method_name.len()
        + 1;
    let type_end = function.rfind('>')?;
    if type_end < type_start {
        return None;
    }
    let type_name = function[type_start..type_end].trim().to_owned();

    Some((node_variable_name, kind, type_name))
}

fn to_model_endpoint(
    package: &Package,
    node: &DetectedCppNode,
    endpoint: &DetectedCppEndpoint,
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
            detail: format!("rclcpp {kind_name} topic literal"),
        }],
    }
}

fn parse_cpp(source: &str) -> anyhow::Result<tree_sitter::Tree> {
    let mut parser = tree_sitter::Parser::new();
    parser
        .set_language(&tree_sitter_cpp::LANGUAGE.into())
        .context("failed to load the C++ grammar")?;
    parser
        .parse(source, None)
        .context("the C++ parser returned no syntax tree")
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

fn enclosing_declarator_name(call: Node<'_>, source: &str) -> Option<String> {
    let mut ancestor = call.parent();
    while let Some(node) = ancestor {
        if node.kind() == "init_declarator" {
            let declarator = node.child_by_field_name("declarator")?;
            return identifier_text(declarator, source).map(str::to_owned);
        }
        ancestor = node.parent();
    }
    None
}

fn identifier_text<'source>(node: Node<'_>, source: &'source str) -> Option<&'source str> {
    if node.kind() == "identifier" {
        return node_text(node, source);
    }
    let mut cursor = node.walk();
    node.named_children(&mut cursor)
        .find_map(|child| identifier_text(child, source))
}

fn enclosing_class_name(node: Node<'_>, source: &str) -> Option<String> {
    let mut ancestor = node.parent();
    while let Some(node) = ancestor {
        if matches!(node.kind(), "class_specifier" | "struct_specifier") {
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
    if node.kind() != "string_literal" {
        return None;
    }
    let literal = node_text(node, source)?;
    let value = literal.strip_prefix('"')?.strip_suffix('"')?;
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
    fn scans_rclcpp_node_and_topics() -> anyhow::Result<()> {
        let source = r#"int main() {
    auto node = std::make_shared<rclcpp::Node>("camera");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);
    auto subscription = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, callback);
}
"#;
        let package = Package {
            id: EntityId::new("package:camera_cpp"),
            name: "camera_cpp".to_owned(),
            path: "src/camera_cpp".to_owned(),
        };

        let nodes = scan_cpp_source(&package, "camera_cpp", source, "src/camera.cpp")?;

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].logical_name, "camera");
        assert_eq!(nodes[0].endpoints.len(), 2);
        assert_eq!(nodes[0].endpoints[0].name, "/camera/image");
        assert_eq!(nodes[0].endpoints[0].type_name, "sensor_msgs::msg::Image");
        assert_eq!(nodes[0].endpoints[1].name, "/cmd_vel");

        let renamed_source = source.replacen("\"camera\"", "\"renamed_camera\"", 1);
        let renamed_nodes =
            scan_cpp_source(&package, "camera_cpp", &renamed_source, "src/camera.cpp")?;
        assert_eq!(renamed_nodes[0].id, nodes[0].id);

        Ok(())
    }

    #[test]
    fn ignores_dynamic_cpp_names_and_topics() -> anyhow::Result<()> {
        let source = r#"auto node = std::make_shared<rclcpp::Node>(node_name);
node->create_publisher<std_msgs::msg::String>(topic_name, 10);"#;

        assert!(detect_cpp_nodes(source, "node.cpp")?.is_empty());
        assert!(detect_cpp_endpoints(source, "node.cpp")?.is_empty());

        Ok(())
    }
}
