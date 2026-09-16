#![forbid(unsafe_code)]

use std::{
    fs,
    path::{Path, PathBuf},
};

use anyhow::Context as _;
use quick_xml::{Reader, events::Event};
use ros_studio_model::{EntityId, Package, Project};
use walkdir::{DirEntry, WalkDir};

mod cpp_scan;
mod python_scan;
mod rust_scan;

pub use cpp_scan::{
    DetectedCppEndpoint, DetectedCppNode, detect_cpp_endpoints, detect_cpp_nodes, scan_cpp_source,
};
pub use python_scan::{
    DetectedPythonEndpoint, DetectedPythonNode, detect_python_endpoints, detect_python_nodes,
    scan_python_source,
};
pub use rust_scan::{
    DetectedRustEndpoint, DetectedRustNode, detect_rust_endpoints, detect_rust_nodes,
    scan_rust_source,
};
const IGNORED_DIRECTORIES: [&str; 5] = [".git", "build", "install", "log", "target"];

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DetectedPackage {
    pub package: Package,
    pub manifest_path: PathBuf,
    pub cargo_manifest_path: Option<PathBuf>,
    pub rust_source_paths: Vec<PathBuf>,
    pub python_source_paths: Vec<PathBuf>,
    pub cpp_source_paths: Vec<PathBuf>,
}

pub fn find_package_manifests(root: &Path) -> Result<Vec<PathBuf>, walkdir::Error> {
    let mut manifests = Vec::new();

    for entry in WalkDir::new(root).into_iter().filter_entry(should_visit) {
        let entry = entry?;

        if entry.file_type().is_file() && entry.file_name() == "package.xml" {
            manifests.push(entry.into_path());
        }
    }

    manifests.sort();

    Ok(manifests)
}

pub fn find_rust_sources(package_root: &Path) -> Result<Vec<PathBuf>, walkdir::Error> {
    find_sources_with_extensions(package_root, &["rs"])
}

pub fn find_python_sources(package_root: &Path) -> Result<Vec<PathBuf>, walkdir::Error> {
    find_sources_with_extensions(package_root, &["py"])
}

pub fn find_cpp_sources(package_root: &Path) -> Result<Vec<PathBuf>, walkdir::Error> {
    find_sources_with_extensions(
        package_root,
        &["c", "cc", "cpp", "cxx", "h", "hh", "hpp", "hxx"],
    )
}

fn find_sources_with_extensions(
    package_root: &Path,
    extensions: &[&str],
) -> Result<Vec<PathBuf>, walkdir::Error> {
    let mut sources = Vec::new();

    for entry in WalkDir::new(package_root)
        .into_iter()
        .filter_entry(should_visit)
    {
        let entry = entry?;

        let is_supported_source = entry.file_type().is_file()
            && entry
                .path()
                .extension()
                .and_then(|extension| extension.to_str())
                .is_some_and(|extension| extensions.contains(&extension));

        if is_supported_source {
            sources.push(entry.into_path());
        }
    }

    sources.sort();

    Ok(sources)
}

pub fn scan_workspace(root: &Path) -> anyhow::Result<Vec<DetectedPackage>> {
    let manifests = find_package_manifests(root)?;
    let mut packages = Vec::with_capacity(manifests.len());

    for manifest_path in manifests {
        let xml = fs::read_to_string(&manifest_path)
            .with_context(|| format!("failed to read {}", manifest_path.display()))?;

        let name = parse_package_name(&xml)?
            .with_context(|| format!("missing package name in {}", manifest_path.display()))?;

        let package_directory = manifest_path
            .parent()
            .with_context(|| format!("missing parent directory for {}", manifest_path.display()))?;

        let relative_directory = package_directory
            .strip_prefix(root)
            .with_context(|| {
                format!(
                    "{} is outside workspace {}",
                    package_directory.display(),
                    root.display()
                )
            })?
            .to_str()
            .with_context(|| {
                format!(
                    "package path is not valid UTF-8: {}",
                    package_directory.display()
                )
            })?
            .replace('\\', "/");

        let cargo_manifest_candidate = package_directory.join("Cargo.toml");
        let cargo_manifest_path = cargo_manifest_candidate
            .is_file()
            .then_some(cargo_manifest_candidate);

        let rust_source_paths = find_rust_sources(package_directory)?;
        let python_source_paths = find_python_sources(package_directory)?;
        let cpp_source_paths = find_cpp_sources(package_directory)?;

        packages.push(DetectedPackage {
            package: Package {
                id: EntityId::new(format!("package:{name}")),
                name,
                path: relative_directory,
            },
            manifest_path,
            cargo_manifest_path,
            rust_source_paths,
            python_source_paths,
            cpp_source_paths,
        });
    }

    packages.sort_by(|left, right| left.package.cmp(&right.package));

    Ok(packages)
}

pub fn scan_project(root: &Path, project_name: &str) -> anyhow::Result<Project> {
    let detected_packages = scan_workspace(root)?;
    let mut packages = Vec::with_capacity(detected_packages.len());
    let mut nodes = Vec::new();

    for detected_package in detected_packages {
        let package = detected_package.package;
        let executable = package.name.clone();

        for source_path in detected_package.rust_source_paths {
            let source = fs::read_to_string(&source_path)
                .with_context(|| format!("failed to read Rust source {}", source_path.display()))?;

            let relative_source_path = source_path.strip_prefix(root).with_context(|| {
                format!(
                    "{} is outside workspace {}",
                    source_path.display(),
                    root.display()
                )
            })?;

            let relative_source_path = normalized_path(relative_source_path)?;

            nodes.extend(scan_rust_source(
                &package,
                &executable,
                &source,
                &relative_source_path,
            )?);
        }

        for source_path in detected_package.python_source_paths {
            let source = fs::read_to_string(&source_path).with_context(|| {
                format!("failed to read Python source {}", source_path.display())
            })?;
            let relative_source_path =
                normalized_path(source_path.strip_prefix(root).with_context(|| {
                    format!(
                        "{} is outside workspace {}",
                        source_path.display(),
                        root.display()
                    )
                })?)?;

            nodes.extend(scan_python_source(
                &package,
                &executable,
                &source,
                &relative_source_path,
            )?);
        }

        for source_path in detected_package.cpp_source_paths {
            let source = fs::read_to_string(&source_path)
                .with_context(|| format!("failed to read C++ source {}", source_path.display()))?;
            let relative_source_path =
                normalized_path(source_path.strip_prefix(root).with_context(|| {
                    format!(
                        "{} is outside workspace {}",
                        source_path.display(),
                        root.display()
                    )
                })?)?;

            nodes.extend(scan_cpp_source(
                &package,
                &executable,
                &source,
                &relative_source_path,
            )?);
        }

        packages.push(package);
    }

    let mut project = Project {
        id: EntityId::new(format!("project:{project_name}")),
        name: project_name.to_owned(),
        root_path: normalized_path(root)?,
        packages,
        nodes,
    };

    project.sort_deterministically();

    Ok(project)
}

fn normalized_path(path: &Path) -> anyhow::Result<String> {
    Ok(path
        .to_str()
        .with_context(|| format!("path is not valid UTF-8: {}", path.display()))?
        .replace('\\', "/"))
}

fn parse_package_name(xml: &str) -> Result<Option<String>, quick_xml::Error> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    loop {
        match reader.read_event()? {
            Event::Start(element) if element.name().as_ref() == b"name" => {
                let name = reader.read_text(element.name())?;
                return Ok(Some(name.decode()?.into_owned()));
            }
            Event::Eof => return Ok(None),
            _ => {}
        }
    }
}

fn should_visit(entry: &DirEntry) -> bool {
    entry.depth() == 0
        || !entry.file_type().is_dir()
        || !IGNORED_DIRECTORIES
            .iter()
            .any(|directory| entry.file_name() == *directory)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_workspace() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/drone_demo_ws")
    }

    #[test]
    fn finds_package_manifests_in_workspace_fixture() -> Result<(), walkdir::Error> {
        let workspace = fixture_workspace();
        let manifests = find_package_manifests(&workspace)?;

        assert_eq!(
            manifests,
            vec![
                workspace.join("src/autopilot_bridge/package.xml"),
                workspace.join("src/camera/package.xml"),
                workspace.join("src/detector/package.xml"),
                workspace.join("src/navigation/package.xml"),
            ]
        );

        Ok(())
    }

    #[test]
    fn parses_package_name() -> Result<(), quick_xml::Error> {
        let xml = "<package><name>camera</name></package>";

        assert_eq!(parse_package_name(xml)?, Some("camera".to_owned()));

        Ok(())
    }

    #[test]
    fn scans_package_metadata() -> anyhow::Result<()> {
        let workspace = fixture_workspace();
        let packages = scan_workspace(&workspace)?;

        let summaries = packages
            .iter()
            .map(|detected| {
                (
                    detected.package.id.as_str(),
                    detected.package.name.as_str(),
                    detected.package.path.as_str(),
                )
            })
            .collect::<Vec<_>>();

        assert_eq!(
            summaries,
            [
                (
                    "package:autopilot_bridge",
                    "autopilot_bridge",
                    "src/autopilot_bridge",
                ),
                ("package:camera", "camera", "src/camera"),
                ("package:detector", "detector", "src/detector"),
                ("package:navigation", "navigation", "src/navigation"),
            ]
        );

        for detected in &packages {
            let package_root = workspace.join(detected.package.path.as_str());
            let source_name = format!("{}.rs", detected.package.name);

            assert_eq!(detected.manifest_path, package_root.join("package.xml"));

            assert_eq!(
                detected.cargo_manifest_path,
                Some(package_root.join("Cargo.toml"))
            );

            assert_eq!(
                detected.rust_source_paths,
                vec![package_root.join("src").join(source_name)]
            );
            assert!(detected.python_source_paths.is_empty());
            assert!(detected.cpp_source_paths.is_empty());
        }

        Ok(())
    }

    #[test]
    fn finds_rust_sources_in_package() -> Result<(), walkdir::Error> {
        let package_root = fixture_workspace().join("src/camera");
        let sources = find_rust_sources(&package_root)?;

        assert_eq!(sources, vec![package_root.join("src/camera.rs")]);

        Ok(())
    }

    #[test]
    fn scans_fixture_into_project_graph() -> anyhow::Result<()> {
        let workspace = fixture_workspace();
        let project = scan_project(&workspace, "drone_demo_ws")?;

        assert_eq!(project.id.as_str(), "project:drone_demo_ws");
        assert_eq!(project.name, "drone_demo_ws");
        assert_eq!(project.packages.len(), 4);
        assert_eq!(project.nodes.len(), 4);

        let graph = project
            .nodes
            .iter()
            .map(|node| {
                let topics = node
                    .endpoints
                    .iter()
                    .map(|endpoint| endpoint.name.as_str())
                    .collect::<Vec<_>>();

                (node.logical_name.as_str(), topics)
            })
            .collect::<Vec<_>>();

        assert_eq!(
            graph,
            [
                ("autopilot_bridge", vec!["/cmd_vel"]),
                ("camera", vec!["/camera/image"]),
                ("detector", vec!["/detections", "/camera/image"]),
                ("navigation", vec!["/cmd_vel", "/detections"]),
            ]
        );

        Ok(())
    }

    #[test]
    fn project_graph_matches_golden() -> anyhow::Result<()> {
        let workspace = fixture_workspace();
        let mut project = scan_project(&workspace, "drone_demo_ws")?;
        project.root_path = "$WORKSPACE".to_owned();

        let actual = serde_json::to_string_pretty(&project)?;
        let expected = include_str!("../../../examples/drone_demo_ws/expected_graph.json");

        assert_eq!(actual, expected.trim_end());

        Ok(())
    }
}
