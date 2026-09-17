#![forbid(unsafe_code)]

use std::{
    fs::{self, OpenOptions},
    io::Write,
    path::{Path, PathBuf},
    str::FromStr,
};

use anyhow::{Context as _, bail, ensure};
use ros_studio_model::{EntityId, Project};
use toml_edit::{DocumentMut, Item, Table, Value, value};

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
    let version = match dependency {
        Item::Value(Value::String(version)) => Some(version.value().as_str()),
        Item::Value(Value::InlineTable(table)) => table.get("version").and_then(Value::as_str),
        Item::Table(table) => table.get("version").and_then(Item::as_str),
        _ => None,
    };
    let Some(version) = version else {
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

    let source_relative_path = source_path.strip_prefix(&workspace_root)?.to_path_buf();
    let manifest_relative_path = manifest_path.strip_prefix(&workspace_root)?.to_path_buf();

    Ok(NodePreview {
        source_relative_path,
        manifest_relative_path,
        source: format!(
            "use rclrs::{{CreateBasicExecutor, RclrsErrorFilter, SpinOptions}};\n\nfn main() -> anyhow::Result<()> {{\n    let context = rclrs::Context::default_from_env()?;\n    let mut executor = context.create_basic_executor();\n    let _node = executor.create_node(\"{node_name}\")?;\n\n    executor.spin(SpinOptions::default()).first_error()?;\n    Ok(())\n}}\n"
        ),
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
    use ros_studio_model::Package;

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
}
