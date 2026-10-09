#![forbid(unsafe_code)]

use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, OpenOptions},
    io::Write,
    path::{Path, PathBuf},
    str::FromStr,
};

use anyhow::{Context as _, bail, ensure};
use ros_studio_model::{EndpointKind, EntityId, Project};
use toml_edit::{DocumentMut, Item, Table, Value, value};

const ROS_ENV_VERSION_REQUIREMENT: &str = "=0.2.0";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RclrsApi {
    V06,
    V07,
}

impl RclrsApi {
    pub fn version(self) -> &'static str {
        match self {
            Self::V06 => "0.6",
            Self::V07 => "0.7",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ApiSelection {
    #[default]
    Auto,
    Manual(RclrsApi),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ApiDetection {
    Missing,
    Supported(RclrsApi),
    Unknown(String),
    Unsupported(String),
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct NodeInterface {
    pub kind: EndpointKind,
    pub name: String,
    pub type_name: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TopicOption {
    pub name: String,
    pub type_name: String,
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
}

pub fn available_topics(project: &Project) -> Vec<TopicOption> {
    let mut topics = BTreeMap::<(String, String), (BTreeSet<String>, BTreeSet<String>)>::new();
    for node in &project.nodes {
        for endpoint in &node.endpoints {
            if !matches!(
                endpoint.kind,
                EndpointKind::Publisher | EndpointKind::Subscription
            ) {
                continue;
            }
            let Some(type_name) = canonical_message_type(&endpoint.type_name) else {
                continue;
            };
            let (publishers, subscribers) = topics
                .entry((endpoint.name.clone(), type_name))
                .or_default();
            match endpoint.kind {
                EndpointKind::Publisher => {
                    publishers.insert(node.logical_name.clone());
                }
                EndpointKind::Subscription => {
                    subscribers.insert(node.logical_name.clone());
                }
                _ => continue,
            }
        }
    }
    topics
        .into_iter()
        .map(
            |((name, type_name), (publishers, subscribers))| TopicOption {
                name,
                type_name,
                publishers: publishers.into_iter().collect(),
                subscribers: subscribers.into_iter().collect(),
            },
        )
        .collect()
}

fn canonical_message_type(type_name: &str) -> Option<String> {
    let parts = if type_name.contains("::") {
        type_name.split("::").collect::<Vec<_>>()
    } else {
        type_name.split('/').collect::<Vec<_>>()
    };
    let [package, "msg", message] = parts.as_slice() else {
        return None;
    };
    if !valid_rust_identifier(package) || !valid_rust_identifier(message) {
        return None;
    }
    Some(format!("{package}::msg::{message}"))
}

fn valid_rust_identifier(identifier: &str) -> bool {
    valid_node_name(identifier)
        && !matches!(
            identifier,
            "as" | "async"
                | "await"
                | "break"
                | "const"
                | "crate"
                | "dyn"
                | "else"
                | "enum"
                | "extern"
                | "false"
                | "fn"
                | "for"
                | "if"
                | "impl"
                | "in"
                | "let"
                | "loop"
                | "match"
                | "mod"
                | "move"
                | "mut"
                | "pub"
                | "ref"
                | "return"
                | "self"
                | "Self"
                | "static"
                | "struct"
                | "super"
                | "trait"
                | "true"
                | "type"
                | "unsafe"
                | "use"
                | "where"
                | "while"
        )
}

pub fn detect_rclrs_api(project: &Project, package_id: &EntityId) -> anyhow::Result<ApiDetection> {
    let package = project
        .packages
        .iter()
        .find(|package| &package.id == package_id)
        .context("Selected package is no longer in the project")?;
    let manifest_path = Path::new(&project.root_path)
        .join(&package.path)
        .join("Cargo.toml");
    let manifest = DocumentMut::from_str(&fs::read_to_string(&manifest_path)?)?;
    detect_rclrs_api_in_manifest(project, &manifest_path, &manifest)
}

fn detect_rclrs_api_in_manifest(
    project: &Project,
    manifest_path: &Path,
    manifest: &DocumentMut,
) -> anyhow::Result<ApiDetection> {
    let Some(dependency) = manifest
        .get("dependencies")
        .and_then(|dependencies| dependencies.get("rclrs"))
    else {
        return Ok(ApiDetection::Missing);
    };
    if dependency_workspace(dependency) {
        let workspace_manifest_path = Path::new(&project.root_path).join("Cargo.toml");
        if workspace_manifest_path != manifest_path && workspace_manifest_path.is_file() {
            let workspace_manifest =
                DocumentMut::from_str(&fs::read_to_string(workspace_manifest_path)?)?;
            if let Some(workspace_dependency) = workspace_manifest
                .get("workspace")
                .and_then(|workspace| workspace.get("dependencies"))
                .and_then(|dependencies| dependencies.get("rclrs"))
            {
                return Ok(detect_dependency(workspace_dependency));
            }
        }
        return Ok(ApiDetection::Unknown(
            "rclrs is inherited from a workspace".into(),
        ));
    }
    Ok(detect_dependency(dependency))
}

fn dependency_workspace(dependency: &Item) -> bool {
    match dependency {
        Item::Value(Value::InlineTable(table)) => {
            table.get("workspace").and_then(Value::as_bool) == Some(true)
        }
        Item::Table(table) => table.get("workspace").and_then(Item::as_bool) == Some(true),
        _ => false,
    }
}

fn detect_dependency(dependency: &Item) -> ApiDetection {
    let Some(version) = dependency_version(dependency) else {
        return ApiDetection::Unknown("rclrs version is not declared explicitly".into());
    };
    let normalized = version.trim().trim_start_matches(['^', '~', '=']).trim();
    let parts = normalized.split('.').collect::<Vec<_>>();
    let api = match parts.as_slice() {
        ["0", "6"] | ["0", "6", _] => Some(RclrsApi::V06),
        ["0", "7"] | ["0", "7", _] => Some(RclrsApi::V07),
        _ => None,
    };
    // A precise numeric patch is supported, but range expressions are deliberately not guessed.
    let api = api.filter(|_| {
        normalized
            .split('.')
            .all(|part| !part.is_empty() && part.bytes().all(|byte| byte.is_ascii_digit()))
    });
    match api {
        Some(api) => ApiDetection::Supported(api),
        None => {
            ApiDetection::Unsupported(format!("unsupported or ambiguous rclrs version: {version}"))
        }
    }
}

fn dependency_version(dependency: &Item) -> Option<&str> {
    match dependency {
        Item::Value(Value::String(version)) => Some(version.value().as_str()),
        Item::Value(Value::InlineTable(table)) => table.get("version").and_then(Value::as_str),
        Item::Table(table) => table.get("version").and_then(Item::as_str),
        _ => None,
    }
}

pub struct NodePreview {
    pub source_relative_path: PathBuf,
    pub manifest_relative_path: PathBuf,
    pub source: String,
    pub manifest_before: String,
    pub manifest_after: String,
    pub api_note: String,
    source_path: PathBuf,
    manifest_path: PathBuf,
}

impl NodePreview {
    pub fn changes_manifest(&self) -> bool {
        self.manifest_before != self.manifest_after
    }
}

pub fn preview_node(
    project: &Project,
    package_id: &EntityId,
    node_name: &str,
) -> anyhow::Result<NodePreview> {
    preview_node_with_api(project, package_id, node_name, ApiSelection::Auto)
}

pub fn preview_node_with_api(
    project: &Project,
    package_id: &EntityId,
    node_name: &str,
    selection: ApiSelection,
) -> anyhow::Result<NodePreview> {
    preview_node_with_interfaces(project, package_id, node_name, selection, &[])
}

pub fn preview_node_with_interfaces(
    project: &Project,
    package_id: &EntityId,
    node_name: &str,
    selection: ApiSelection,
    interfaces: &[NodeInterface],
) -> anyhow::Result<NodePreview> {
    ensure!(
        valid_node_name(node_name),
        "Node name must start with a letter or underscore and contain only ASCII letters, digits, or underscores"
    );
    ensure!(
        !project
            .nodes
            .iter()
            .any(|node| { &node.package_id == package_id && node.logical_name == node_name }),
        "A node named {node_name} already exists in this package"
    );

    let package = project
        .packages
        .iter()
        .find(|package| &package.id == package_id)
        .context("Selected package is no longer in the project")?;
    let workspace_root = Path::new(&project.root_path)
        .canonicalize()
        .context("Cannot access the workspace directory")?;
    let package_relative_path = Path::new(&package.path);
    ensure!(
        !package_relative_path.is_absolute(),
        "Package path must be relative to the workspace"
    );
    let package_root = workspace_root
        .join(package_relative_path)
        .canonicalize()
        .with_context(|| format!("Cannot access package {}", package.name))?;
    ensure!(
        package_root.starts_with(&workspace_root),
        "Package is outside the workspace"
    );
    let source_directory = package_root.join("src");
    let source_directory = source_directory
        .canonicalize()
        .with_context(|| format!("Package {} has no src directory", package.name))?;
    ensure!(
        source_directory.starts_with(&package_root),
        "Package source directory is outside the package"
    );
    let bin_directory = source_directory.join("bin");
    if bin_directory.exists() {
        let bin_directory = bin_directory.canonicalize()?;
        ensure!(
            bin_directory.starts_with(&package_root),
            "Package bin directory is outside the package"
        );
    }

    let source_path = bin_directory.join(format!("{node_name}.rs"));
    ensure!(
        !source_path.exists(),
        "Source file already exists: {}",
        source_path.display()
    );
    let manifest_path = package_root.join("Cargo.toml");
    ensure!(
        manifest_path.is_file(),
        "Selected package is not a Rust crate"
    );
    let manifest_before = fs::read_to_string(&manifest_path)
        .with_context(|| format!("Cannot read {}", manifest_path.display()))?;
    let mut manifest = DocumentMut::from_str(&manifest_before)
        .with_context(|| format!("Cannot parse {}", manifest_path.display()))?;
    let package = manifest
        .as_table()
        .get("package")
        .and_then(Item::as_table)
        .context("Selected Cargo.toml does not declare a package")?;
    ensure!(
        package.get("autobins").and_then(Item::as_bool) != Some(false),
        "This package disables automatic binaries; add a bin target manually first"
    );
    if let Some(binaries) = manifest
        .as_table()
        .get("bin")
        .and_then(Item::as_array_of_tables)
    {
        ensure!(
            !binaries
                .iter()
                .any(|binary| binary.get("name").and_then(Item::as_str) == Some(node_name)),
            "A binary target named {node_name} already exists"
        );
    }

    let detection = detect_rclrs_api_in_manifest(project, &manifest_path, &manifest)?;
    let api = match (selection, &detection) {
        (ApiSelection::Auto, ApiDetection::Missing) => RclrsApi::V07,
        (ApiSelection::Auto, ApiDetection::Supported(api)) => *api,
        (ApiSelection::Auto, ApiDetection::Unknown(reason)) => {
            bail!("Cannot detect the rclrs API: {reason}. Choose a version in the wizard")
        }
        (_, ApiDetection::Unsupported(reason)) => bail!("{reason}"),
        (ApiSelection::Manual(api), ApiDetection::Supported(found)) if api != *found => {
            bail!(
                "Package declares rclrs {}; selected rclrs {} would not match",
                found.version(),
                api.version()
            )
        }
        (ApiSelection::Manual(api), _) => api,
    };

    if !interfaces.is_empty() {
        ensure!(
            api == RclrsApi::V07,
            "Typed pub/sub generation currently requires rclrs 0.7; create the node without interfaces or use a 0.7 package"
        );
    }
    let available = available_topics(project);
    let mut unique_interfaces = BTreeSet::new();
    for interface in interfaces {
        ensure!(
            matches!(
                interface.kind,
                EndpointKind::Publisher | EndpointKind::Subscription
            ),
            "Only publishers and subscriptions are supported"
        );
        ensure!(
            available.iter().any(|topic| {
                topic.name == interface.name && topic.type_name == interface.type_name
            }),
            "Topic {} ({}) is no longer in the project",
            interface.name,
            interface.type_name
        );
        ensure!(
            unique_interfaces.insert(interface.clone()),
            "Duplicate interface selected: {} ({})",
            interface.name,
            interface.type_name
        );
    }

    if manifest.as_table().get("dependencies").is_none() {
        manifest["dependencies"] = Item::Table(Table::new());
    }
    let dependencies = manifest["dependencies"]
        .as_table_mut()
        .context("Cannot add rclrs to the package dependencies")?;
    if dependencies.get("rclrs").is_none() {
        dependencies["rclrs"] = value(api.version());
    }
    if dependencies.get("anyhow").is_none() {
        dependencies["anyhow"] = value("1.0");
    }
    if !interfaces.is_empty() {
        if let Some(ros_env_dependency) = dependencies.get("ros-env") {
            let version = dependency_version(ros_env_dependency)
                .context("Existing ros-env dependency has no explicit version; typed pub/sub requires ros-env =0.2.0")?;
            ensure!(
                version.trim() == ROS_ENV_VERSION_REQUIREMENT,
                "Existing ros-env version {version} is incompatible with rclrs 0.7; expected {ROS_ENV_VERSION_REQUIREMENT}"
            );
        } else {
            dependencies["ros-env"] = value(ROS_ENV_VERSION_REQUIREMENT);
        }
    }

    let source_relative_path = source_path.strip_prefix(&workspace_root)?.to_path_buf();
    let manifest_relative_path = manifest_path.strip_prefix(&workspace_root)?.to_path_buf();

    Ok(NodePreview {
        source_relative_path,
        manifest_relative_path,
        source: render_node_source(node_name, &unique_interfaces),
        manifest_before,
        manifest_after: manifest.to_string(),
        api_note: match (selection, &detection) {
            (ApiSelection::Auto, ApiDetection::Missing) => {
                "Auto uses rclrs 0.7 (no dependency declared)".to_owned()
            }
            (ApiSelection::Auto, _) => format!("Detected rclrs {} API", api.version()),
            (ApiSelection::Manual(_), _) => {
                format!("Using manually selected rclrs {} API", api.version())
            }
        },
        source_path,
        manifest_path,
    })
}

fn render_node_source(node_name: &str, interfaces: &BTreeSet<NodeInterface>) -> String {
    let mut source =
        String::from("use rclrs::{CreateBasicExecutor, RclrsErrorFilter, SpinOptions};\n");
    let message_packages = interfaces
        .iter()
        .filter_map(|interface| interface.type_name.split("::").next())
        .collect::<BTreeSet<_>>();
    if !message_packages.is_empty() {
        source.push_str(&format!(
            "use ros_env::{{{}}};\n",
            message_packages.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    let node_binding = if interfaces.is_empty() {
        "_node"
    } else {
        "node"
    };
    source.push_str(&format!(
        "\nfn main() -> anyhow::Result<()> {{\n    let context = rclrs::Context::default_from_env()?;\n    let mut executor = context.create_basic_executor();\n    let {node_binding} = executor.create_node(\"{node_name}\")?;\n\n"
    ));
    source.push_str("    // ros-studio:interfaces:start\n");
    let mut used_bindings = BTreeSet::new();
    let mut interface_bindings = Vec::new();
    for interface in interfaces {
        let role = match interface.kind {
            EndpointKind::Publisher => "publisher",
            EndpointKind::Subscription => "subscriber",
            _ => continue,
        };
        let base_binding = format!("{role}_{}", topic_identifier(&interface.name));
        let mut binding = base_binding.clone();
        let mut suffix = 2;
        while !used_bindings.insert(binding.clone()) {
            binding = format!("{base_binding}_{suffix}");
            suffix += 1;
        }
        let topic = format!("{:?}", interface.name);
        match interface.kind {
            EndpointKind::Publisher => source.push_str(&format!(
                "    let {binding} = node.create_publisher::<{}>({topic})?;\n",
                interface.type_name
            )),
            EndpointKind::Subscription => source.push_str(&format!(
                "    let {binding} = node.create_subscription::<{}, _>({topic}, move |_message| {{}})?;\n",
                interface.type_name
            )),
            _ => {}
        }
        interface_bindings.push(binding);
    }
    if !interface_bindings.is_empty() {
        source.push_str("    let _ros_interfaces = (\n");
        for binding in interface_bindings {
            source.push_str(&format!("        {binding},\n"));
        }
        source.push_str("    );\n");
    }
    source.push_str("    // ros-studio:interfaces:end\n\n");
    source.push_str("    executor.spin(SpinOptions::default()).first_error()?;\n    Ok(())\n}\n");
    source
}

fn topic_identifier(topic: &str) -> String {
    let mut identifier = String::new();
    for character in topic.chars() {
        if character.is_ascii_alphanumeric() {
            identifier.push(character.to_ascii_lowercase());
        } else if !identifier.is_empty() && !identifier.ends_with('_') {
            identifier.push('_');
        }
    }
    while identifier.ends_with('_') {
        identifier.pop();
    }
    if identifier.is_empty() {
        "topic".to_owned()
    } else {
        identifier
    }
}

pub fn write_node(preview: &NodePreview) -> anyhow::Result<PathBuf> {
    ensure!(
        fs::read_to_string(&preview.manifest_path)? == preview.manifest_before,
        "Cargo.toml changed after the preview; review the changes again"
    );
    let source_directory = preview
        .source_path
        .parent()
        .context("Generated source has no parent directory")?;
    if !source_directory.exists() {
        fs::create_dir(source_directory)
            .with_context(|| format!("Cannot create {}", source_directory.display()))?;
    }
    let mut source_file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&preview.source_path)
        .with_context(|| format!("Cannot create {}", preview.source_path.display()))?;
    if let Err(error) = source_file
        .write_all(preview.source.as_bytes())
        .and_then(|()| source_file.flush())
    {
        if let Err(cleanup_error) = fs::remove_file(&preview.source_path) {
            bail!("Cannot write source: {error}; cannot remove partial file: {cleanup_error}");
        }
        return Err(error).context("Cannot write generated source");
    }

    if preview.changes_manifest() {
        if let Err(error) = update_manifest(preview) {
            if let Err(cleanup_error) = fs::remove_file(&preview.source_path) {
                bail!("{error:#}; cannot remove generated source: {cleanup_error}");
            }
            return Err(error);
        }
    }

    Ok(preview.source_path.clone())
}

fn update_manifest(preview: &NodePreview) -> anyhow::Result<()> {
    let manifest_directory = preview
        .manifest_path
        .parent()
        .context("Cargo.toml has no parent directory")?;
    let mut temporary = tempfile::NamedTempFile::new_in(manifest_directory)?;
    temporary.write_all(preview.manifest_after.as_bytes())?;
    temporary.flush()?;
    fs::set_permissions(
        temporary.path(),
        fs::metadata(&preview.manifest_path)?.permissions(),
    )?;
    ensure!(
        fs::read_to_string(&preview.manifest_path)? == preview.manifest_before,
        "Cargo.toml changed while the node was being created; review the changes again"
    );
    temporary
        .persist(&preview.manifest_path)
        .map_err(|error| error.error)
        .with_context(|| format!("Cannot update {}", preview.manifest_path.display()))?;
    Ok(())
}

fn valid_node_name(name: &str) -> bool {
    let mut characters = name.bytes();
    matches!(characters.next(), Some(b'a'..=b'z' | b'A'..=b'Z' | b'_'))
        && characters
            .all(|character| matches!(character, b'a'..=b'z' | b'A'..=b'Z' | b'0'..=b'9' | b'_'))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_studio_model::{Confidence, Endpoint, Node, Package, RuntimeState};

    fn fixture(root: &Path) -> anyhow::Result<Project> {
        let package_root = root.join("src/camera");
        fs::create_dir_all(package_root.join("src"))?;
        fs::write(
            package_root.join("Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
        )?;
        fs::write(
            package_root.join("package.xml"),
            "<package format=\"3\"><name>camera</name><version>0.1.0</version><description>test</description><maintainer email=\"test@example.com\">test</maintainer><license>Apache-2.0</license></package>",
        )?;
        Ok(Project {
            id: EntityId::new("project:test"),
            name: "test".to_owned(),
            root_path: root.to_string_lossy().into_owned(),
            packages: vec![Package {
                id: EntityId::new("package:camera"),
                name: "camera".to_owned(),
                path: "src/camera".to_owned(),
            }],
            nodes: Vec::new(),
        })
    }

    fn fixture_with_topics(root: &Path) -> anyhow::Result<Project> {
        let mut project = fixture(root)?;
        project.nodes.push(Node {
            id: EntityId::new("node:fixture:camera"),
            logical_name: "camera".to_owned(),
            package_id: project.packages[0].id.clone(),
            executable: "camera".to_owned(),
            source_locations: Vec::new(),
            endpoints: vec![
                Endpoint {
                    id: EntityId::new("endpoint:fixture:camera:image"),
                    kind: EndpointKind::Publisher,
                    name: "/camera/image".to_owned(),
                    type_name: "sensor_msgs::msg::Image".to_owned(),
                    source_location: None,
                    confidence: Confidence::ConfirmedSource,
                    evidence: Vec::new(),
                },
                Endpoint {
                    id: EntityId::new("endpoint:fixture:camera:status"),
                    kind: EndpointKind::Subscription,
                    name: "/camera/status".to_owned(),
                    type_name: "std_msgs/msg/String".to_owned(),
                    source_location: None,
                    confidence: Confidence::ConfirmedSource,
                    evidence: Vec::new(),
                },
            ],
            evidence: Vec::new(),
            runtime_state: RuntimeState::Unknown,
        });
        Ok(project)
    }

    #[test]
    fn previews_and_creates_an_additional_rust_binary() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let preview = preview_node(&project, &project.packages[0].id, "camera_backup")?;
        assert!(preview.changes_manifest());
        assert!(
            preview
                .source_relative_path
                .ends_with("src/camera/src/bin/camera_backup.rs")
        );
        assert!(
            preview
                .source
                .contains("executor.create_node(\"camera_backup\")")
        );
        assert!(
            preview
                .source
                .contains("executor.spin(SpinOptions::default())")
        );
        assert!(preview.manifest_after.contains("rclrs = \"0.7\""));
        assert!(preview.manifest_after.contains("anyhow = \"1.0\""));

        let source_path = write_node(&preview)?;
        assert_eq!(fs::read_to_string(source_path)?, preview.source);
        assert!(
            fs::read_to_string(root.path().join("src/camera/Cargo.toml"))?
                .contains("rclrs = \"0.7\"")
        );
        let scanned = ros_studio_scan::scan_project(root.path(), "test")?;
        assert_eq!(scanned.nodes.len(), 1);
        assert_eq!(scanned.nodes[0].logical_name, "camera_backup");
        assert_eq!(scanned.nodes[0].executable, "camera_backup");
        Ok(())
    }

    #[test]
    fn rejects_invalid_names_and_existing_files() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        for name in ["", "1camera", "camera-node", "../camera"] {
            assert!(preview_node(&project, &project.packages[0].id, name).is_err());
        }
        let preview = preview_node(&project, &project.packages[0].id, "camera_backup")?;
        write_node(&preview)?;
        assert!(preview_node(&project, &project.packages[0].id, "camera_backup").is_err());
        Ok(())
    }

    #[test]
    fn refuses_a_changed_manifest_without_overwriting_it() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let preview = preview_node(&project, &project.packages[0].id, "camera_backup")?;
        fs::write(root.path().join("src/camera/Cargo.toml"), "# changed\n")?;
        assert!(write_node(&preview).is_err());
        assert!(!root.path().join(&preview.source_relative_path).exists());
        Ok(())
    }

    #[test]
    fn refuses_to_overwrite_a_new_source_file() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let preview = preview_node(&project, &project.packages[0].id, "camera_backup")?;
        let source_path = root.path().join(&preview.source_relative_path);
        fs::create_dir_all(source_path.parent().context("source parent missing")?)?;
        fs::write(&source_path, "user code\n")?;
        assert!(write_node(&preview).is_err());
        assert_eq!(fs::read_to_string(source_path)?, "user code\n");
        Ok(())
    }

    #[test]
    fn detects_existing_rclrs_version_and_preserves_it() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        fs::write(
            root.path().join("src/camera/Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = { version = \"0.6.1\" }\n",
        )?;
        let package_id = &project.packages[0].id;
        assert_eq!(
            detect_rclrs_api(&project, package_id)?,
            ApiDetection::Supported(RclrsApi::V06)
        );
        let preview = preview_node(&project, package_id, "backup")?;
        assert!(preview.manifest_after.contains("version = \"0.6.1\""));
        assert!(!preview.manifest_after.contains("rclrs = \"0.7\""));
        assert!(preview.source.contains("executor.create_node(\"backup\")"));
        assert!(
            preview_node_with_api(
                &project,
                package_id,
                "backup",
                ApiSelection::Manual(RclrsApi::V07)
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn unknown_dependency_requires_manual_api_selection() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        fs::write(
            root.path().join("src/camera/Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = { git = \"https://example.invalid/rclrs\" }\n",
        )?;
        let package_id = &project.packages[0].id;
        assert!(matches!(
            detect_rclrs_api(&project, package_id)?,
            ApiDetection::Unknown(_)
        ));
        assert!(preview_node(&project, package_id, "backup").is_err());
        let preview = preview_node_with_api(
            &project,
            package_id,
            "backup",
            ApiSelection::Manual(RclrsApi::V07),
        )?;
        assert!(preview.manifest_after.contains("git ="));
        assert!(!preview.manifest_after.contains("rclrs = \"0.7\""));
        Ok(())
    }

    #[test]
    fn manual_selection_controls_new_dependency_version() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let preview = preview_node_with_api(
            &project,
            &project.packages[0].id,
            "backup",
            ApiSelection::Manual(RclrsApi::V06),
        )?;
        assert!(preview.manifest_after.contains("rclrs = \"0.6\""));
        Ok(())
    }

    #[test]
    fn detects_workspace_inherited_dependency() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        fs::write(
            root.path().join("Cargo.toml"),
            "[workspace]\n[workspace.dependencies]\nrclrs = \"0.6\"\n",
        )?;
        fs::write(
            root.path().join("src/camera/Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = { workspace = true }\n",
        )?;
        assert_eq!(
            detect_rclrs_api(&project, &project.packages[0].id)?,
            ApiDetection::Supported(RclrsApi::V06)
        );
        Ok(())
    }

    #[test]
    fn rejects_explicit_unsupported_rclrs_version_even_with_manual_choice() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        fs::write(
            root.path().join("src/camera/Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = \"0.5\"\n",
        )?;
        assert!(matches!(
            detect_rclrs_api(&project, &project.packages[0].id)?,
            ApiDetection::Unsupported(_)
        ));
        assert!(
            preview_node_with_api(
                &project,
                &project.packages[0].id,
                "backup",
                ApiSelection::Manual(RclrsApi::V07)
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn generates_existing_typed_interfaces_and_scans_them() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture_with_topics(root.path())?;
        let topics = available_topics(&project);
        assert_eq!(topics.len(), 2);
        assert_eq!(topics[0].publishers, vec!["camera"]);
        assert_eq!(topics[1].type_name, "std_msgs::msg::String");
        let interfaces = vec![
            NodeInterface {
                kind: EndpointKind::Subscription,
                name: "/camera/image".to_owned(),
                type_name: "sensor_msgs::msg::Image".to_owned(),
            },
            NodeInterface {
                kind: EndpointKind::Publisher,
                name: "/camera/status".to_owned(),
                type_name: "std_msgs::msg::String".to_owned(),
            },
        ];
        let preview = preview_node_with_interfaces(
            &project,
            &project.packages[0].id,
            "observer",
            ApiSelection::Auto,
            &interfaces,
        )?;
        assert!(preview.manifest_after.contains("ros-env = \"=0.2.0\""));
        assert!(
            preview
                .source
                .contains("use ros_env::{sensor_msgs, std_msgs};")
        );
        assert!(preview.source.contains("ros-studio:interfaces:start"));
        assert!(preview.source.contains("let subscriber_camera_image ="));
        assert!(preview.source.contains("let publisher_camera_status ="));
        assert!(preview.source.contains("let _ros_interfaces = ("));
        assert!(preview.source.contains(
            "create_subscription::<sensor_msgs::msg::Image, _>(\"/camera/image\", move |_message| {})?;"
        ));
        assert!(
            preview
                .source
                .contains("create_publisher::<std_msgs::msg::String>(\"/camera/status\")")
        );
        write_node(&preview)?;
        let scanned = ros_studio_scan::scan_project(root.path(), "test")?;
        assert_eq!(scanned.nodes.len(), 1);
        assert_eq!(scanned.nodes[0].endpoints.len(), 2);
        assert!(scanned.nodes[0].endpoints.iter().any(|endpoint| {
            endpoint.kind == EndpointKind::Subscription
                && endpoint.name == "/camera/image"
                && endpoint.type_name == "sensor_msgs::msg::Image"
        }));
        Ok(())
    }

    #[test]
    fn gives_interface_bindings_descriptive_unique_names() {
        let interfaces = BTreeSet::from([
            NodeInterface {
                kind: EndpointKind::Subscription,
                name: "/camera/image".to_owned(),
                type_name: "sensor_msgs::msg::Image".to_owned(),
            },
            NodeInterface {
                kind: EndpointKind::Subscription,
                name: "/camera-image".to_owned(),
                type_name: "sensor_msgs::msg::Image".to_owned(),
            },
        ]);
        let source = render_node_source("observer", &interfaces);
        assert!(source.contains("let subscriber_camera_image ="));
        assert!(source.contains("let subscriber_camera_image_2 ="));
        assert!(source.contains("        subscriber_camera_image,"));
        assert!(source.contains("        subscriber_camera_image_2,"));
        assert_eq!(topic_identifier("/camera/image_raw"), "camera_image_raw");
        assert_eq!(topic_identifier("///"), "topic");
    }

    #[test]
    fn refuses_unknown_topics_and_rclrs_06_typed_interfaces() -> anyhow::Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture_with_topics(root.path())?;
        let selected = [NodeInterface {
            kind: EndpointKind::Subscription,
            name: "/missing".to_owned(),
            type_name: "sensor_msgs::msg::Image".to_owned(),
        }];
        assert!(
            preview_node_with_interfaces(
                &project,
                &project.packages[0].id,
                "observer",
                ApiSelection::Auto,
                &selected,
            )
            .is_err()
        );
        let selected = [NodeInterface {
            kind: EndpointKind::Subscription,
            name: "/camera/image".to_owned(),
            type_name: "sensor_msgs::msg::Image".to_owned(),
        }];
        assert!(
            preview_node_with_interfaces(
                &project,
                &project.packages[0].id,
                "observer",
                ApiSelection::Manual(RclrsApi::V06),
                &selected,
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn refuses_incompatible_existing_ros_env_dependency() -> anyhow::Result<()> {
        for version in ["0.1", "0.2", "0.2.1"] {
            let root = tempfile::tempdir()?;
            let project = fixture_with_topics(root.path())?;
            fs::write(
                root.path().join("src/camera/Cargo.toml"),
                format!(
                    "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = \"0.7\"\nros-env = \"{version}\"\n"
                ),
            )?;
            let selected = [NodeInterface {
                kind: EndpointKind::Subscription,
                name: "/camera/image".to_owned(),
                type_name: "sensor_msgs::msg::Image".to_owned(),
            }];
            assert!(
                preview_node_with_interfaces(
                    &project,
                    &project.packages[0].id,
                    "observer",
                    ApiSelection::Auto,
                    &selected,
                )
                .is_err()
            );
        }
        Ok(())
    }
}
