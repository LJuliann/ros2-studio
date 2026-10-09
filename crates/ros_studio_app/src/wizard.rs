use std::{collections::BTreeSet, path::Path};

use anyhow::{Context as _, Result};
use ros_studio_codegen::{ApiSelection, NodeInterface, NodePreview, preview_node_with_interfaces};
use ros_studio_model::{EntityId, Package, Project};

#[derive(Clone, Default)]
pub struct NodeDraft {
    pub name: String,
    pub package_id: Option<EntityId>,
    pub api: ApiSelection,
    pub interfaces: BTreeSet<NodeInterface>,
}

impl NodeDraft {
    pub fn preview(&self, project: &Project) -> Result<NodePreview> {
        let package_id = self.package_id.as_ref().context("Select a Rust package")?;
        preview_node_with_interfaces(
            project,
            package_id,
            &self.name,
            self.api,
            &self.interfaces.iter().cloned().collect::<Vec<_>>(),
        )
    }
}

pub fn rust_packages(project: &Project) -> impl Iterator<Item = &Package> {
    let root = Path::new(&project.root_path);
    project.packages.iter().filter(move |package| {
        root.join(&package.path)
            .join("Cargo.toml")
            .canonicalize()
            .is_ok_and(|manifest| manifest.starts_with(root) && manifest.is_file())
    })
}

pub fn normalize_name_input(name: &str) -> String {
    let mut normalized = String::with_capacity(name.len());
    for character in name.chars() {
        let character = if character.is_whitespace() {
            '_'
        } else {
            character
        };
        let allowed = character == '_'
            || character.is_ascii_alphabetic()
            || (!normalized.is_empty() && character.is_ascii_digit());
        if allowed {
            normalized.push(character);
        }
    }
    normalized
}

pub fn topic_matches_filter(name: &str, type_name: &str, filter: &str) -> bool {
    let name = name.to_lowercase();
    let type_name = type_name.to_lowercase();
    filter.to_lowercase().split_whitespace().all(|term| {
        [name.as_str(), type_name.as_str()]
            .into_iter()
            .any(|candidate| {
                let mut characters = candidate.chars();
                term.chars()
                    .all(|character| characters.by_ref().any(|next| next == character))
            })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GraphSnapshot;
    use ros_studio_codegen::write_node;
    use std::fs;

    fn fixture(root: &Path) -> Result<Project> {
        let package = root.join("src/camera");
        fs::create_dir_all(package.join("src"))?;
        fs::write(
            package.join("package.xml"),
            "<package format=\"3\"><name>camera</name></package>",
        )?;
        fs::write(
            package.join("Cargo.toml"),
            "[package]\nname = \"camera\"\nversion = \"0.1.0\"\n[dependencies]\nrclrs = \"0.7\"\n",
        )?;
        fs::write(
            package.join("src/main.rs"),
            "fn main() { let node = executor.create_node(\"camera\")?; let publisher = node.create_publisher::<sensor_msgs::msg::Image>(\"/camera/image\")?; }",
        )?;
        Ok(GraphSnapshot::scan(root)?.project)
    }

    #[test]
    fn normalizes_names_and_filters_topics() {
        assert_eq!(normalize_name_input("3caméra - backup!"), "camra__backup");
        assert_eq!(normalize_name_input("_camera2\tcopy"), "_camera2_copy");
        assert!(topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "cam img"
        ));
        assert!(topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "Sensor Image"
        ));
        assert!(!topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "velocity"
        ));
    }

    #[test]
    fn previews_without_writes_then_creates_a_connected_node() -> Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let package_id = rust_packages(&project)
            .next()
            .context("Missing Rust package")?
            .id
            .clone();
        let topics = ros_studio_codegen::available_topics(&project);
        let topic = topics.first().context("Missing camera topic")?;
        let draft = NodeDraft {
            name: "camera_backup".into(),
            package_id: Some(package_id),
            interfaces: BTreeSet::from([NodeInterface {
                kind: ros_studio_model::EndpointKind::Subscription,
                name: topic.name.clone(),
                type_name: topic.type_name.clone(),
            }]),
            ..Default::default()
        };
        let manifest = root.path().join("src/camera/Cargo.toml");
        let before = fs::read_to_string(&manifest)?;
        let preview = draft.preview(&project)?;
        assert_eq!(fs::read_to_string(&manifest)?, before);
        assert!(!root.path().join(&preview.source_relative_path).exists());
        let source = write_node(&preview)?;
        assert!(source.is_file());
        let graph = GraphSnapshot::scan(root.path())?;
        assert_eq!(graph.project.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 1);
        assert_eq!(
            graph.edges.first().context("Missing edge")?.topic,
            "/camera/image"
        );
        assert!(write_node(&preview).is_err());
        Ok(())
    }

    #[test]
    fn stale_preview_preserves_user_changes() -> Result<()> {
        let root = tempfile::tempdir()?;
        let project = fixture(root.path())?;
        let draft = NodeDraft {
            name: "camera_backup".into(),
            package_id: project.packages.first().map(|package| package.id.clone()),
            ..Default::default()
        };
        let preview = draft.preview(&project)?;
        let manifest = root.path().join(&preview.manifest_relative_path);
        let changed = format!("{}\n# User edited this\n", fs::read_to_string(&manifest)?);
        fs::write(&manifest, &changed)?;
        assert!(write_node(&preview).is_err());
        assert_eq!(fs::read_to_string(manifest)?, changed);
        assert!(!root.path().join(&preview.source_relative_path).exists());
        Ok(())
    }

    #[test]
    fn does_not_offer_non_rust_or_outside_packages() -> Result<()> {
        let root = tempfile::tempdir()?;
        let outside = tempfile::tempdir()?;
        fs::write(
            outside.path().join("Cargo.toml"),
            "[package]\nname = \"outside\"\nversion = \"0.1.0\"\n",
        )?;
        let mut project = fixture(root.path())?;
        project.packages.push(Package {
            id: EntityId::new("package:python"),
            name: "python".into(),
            path: "src/python".into(),
        });
        project.packages.push(Package {
            id: EntityId::new("package:outside"),
            name: "outside".into(),
            path: outside.path().to_string_lossy().into_owned(),
        });
        assert_eq!(rust_packages(&project).count(), 1);
        Ok(())
    }
}
