use std::{
    collections::{BTreeMap, VecDeque},
    path::Path,
};

use anyhow::{Context as _, Result, bail, ensure};
use ros_studio_model::{
    EntityId, Node, Package, Project, RuntimeState, design_live::reconcile_project,
};
use ros_studio_protocol::{GraphPatch, OutputStream};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum GraphMode {
    #[default]
    Design,
    Live,
    Both,
}

impl GraphMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Design => "Design",
            Self::Live => "Live",
            Self::Both => "Both",
        }
    }
}

#[derive(Default)]
pub struct RuntimeOverlay {
    project_id: Option<EntityId>,
    packages: BTreeMap<EntityId, Package>,
    nodes: BTreeMap<EntityId, Node>,
}

impl RuntimeOverlay {
    pub fn apply(&mut self, patch: GraphPatch, expected: &EntityId) -> bool {
        if &patch.project_id != expected {
            return false;
        }
        self.project_id = Some(patch.project_id);
        for id in patch.removed_package_ids {
            self.packages.remove(&id);
        }
        for package in patch.upsert_packages {
            self.packages.insert(package.id.clone(), package);
        }
        for id in patch.removed_node_ids {
            self.nodes.remove(&id);
        }
        for node in patch.upsert_nodes {
            self.nodes.insert(node.id.clone(), node);
        }
        true
    }

    pub fn project(&self, design: &Project, mode: GraphMode) -> Project {
        if mode == GraphMode::Design {
            return design.clone();
        }
        let runtime = (self.project_id.as_ref() == Some(&design.id)).then(|| Project {
            id: design.id.clone(),
            name: design.name.clone(),
            root_path: design.root_path.clone(),
            packages: self.packages.values().cloned().collect(),
            nodes: self.nodes.values().cloned().collect(),
        });
        let mut visible = reconcile_project(design, runtime.as_ref());
        if mode == GraphMode::Live {
            visible.nodes.retain(|node| {
                matches!(
                    node.runtime_state,
                    RuntimeState::Online | RuntimeState::RuntimeOnly | RuntimeState::Conflict
                )
            });
            visible.packages.retain(|package| {
                visible
                    .nodes
                    .iter()
                    .any(|node| node.package_id == package.id)
            });
        }
        visible
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    Build,
    Run,
}

impl Operation {
    pub fn label(self) -> &'static str {
        match self {
            Self::Build => "Build",
            Self::Run => "Run",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Execution {
    pub operation: Operation,
    pub label: String,
    pub command: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RunTarget {
    Node(String),
    File(String),
    Make(crate::makefile::MakeTarget),
}

impl RunTarget {
    /// A graph selection is the immediate Build/Run target. With no selection,
    /// reuse the explicitly chosen or restored configuration.
    pub fn for_selection(selected_node: Option<&str>, configured: Option<&Self>) -> Option<Self> {
        selected_node
            .map(|id| Self::Node(id.to_owned()))
            .or_else(|| configured.cloned())
    }

    pub fn execution(&self, project: &Project, operation: Operation) -> Result<Execution> {
        match self {
            Self::Node(id) => node_execution(project, id, operation),
            Self::File(path) => source_execution(project, path, operation),
            Self::Make(target) => target.execution(Path::new(&project.root_path), operation),
        }
    }
}

pub fn source_execution(
    project: &Project,
    source: &str,
    operation: Operation,
) -> Result<Execution> {
    let root = Path::new(&project.root_path).canonicalize()?;
    let path = root
        .join(source)
        .canonicalize()
        .context("Source file is missing")?;
    ensure!(
        path.starts_with(&root) && path.is_file(),
        "Choose a source file inside the project"
    );
    // Prefix file arguments so a filename beginning with '-' cannot become a compiler option.
    let relative = format!(
        "./{}",
        path.strip_prefix(&root)?
            .to_str()
            .context("Source path is not UTF-8")?
    );
    let extension = path
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default();
    let command = match extension {
        "py" => match operation {
            Operation::Build => vec!["python3".into(), "-m".into(), "py_compile".into(), relative],
            Operation::Run => vec!["python3".into(), relative],
        },
        "c" | "cc" | "cpp" | "cxx" => {
            let compiler = if extension == "c" { "cc" } else { "c++" };
            match operation {
                Operation::Build => vec![compiler.into(), "-fsyntax-only".into(), relative],
                Operation::Run => temporary_source_run(compiler, relative),
            }
        }
        "rs" => rust_source_command(&root, &path, relative, operation)?,
        _ => bail!("Supported source files: Rust, Python, C, and C++"),
    };
    Ok(Execution {
        operation,
        label: format!("{} file {}", operation.label(), source),
        command,
    })
}

fn temporary_source_run(compiler: &str, source: String) -> Vec<String> {
    // Avoid collisions between projects; arguments never become shell source.
    let script = r#"output=$(mktemp /tmp/ros-studio-run.XXXXXX) || exit; trap 'rm -f -- "$output"' EXIT; "$1" "$2" -o "$output" && "$output""#;
    vec![
        "bash".into(),
        "-c".into(),
        script.into(),
        "ros-studio".into(),
        compiler.into(),
        source,
    ]
}

fn rust_source_command(
    root: &Path,
    path: &Path,
    relative: String,
    operation: Operation,
) -> Result<Vec<String>> {
    let mut manifest = None;
    for directory in path
        .ancestors()
        .skip(1)
        .take_while(|directory| directory.starts_with(root))
    {
        let candidate = directory.join("Cargo.toml");
        if candidate.is_file() {
            let candidate = candidate.canonicalize()?;
            ensure!(
                candidate.starts_with(root),
                "Manifest is outside the project"
            );
            let document: toml::Value = toml::from_str(&std::fs::read_to_string(&candidate)?)?;
            // A virtual workspace must not turn a single-file check into a full workspace build.
            if document.get("package").is_some() {
                manifest = Some((candidate, document));
                break;
            }
        }
    }
    let Some((manifest, document)) = manifest else {
        return Ok(match operation {
            Operation::Build => vec![
                "rustc".into(),
                "--emit=metadata".into(),
                relative,
                "-o".into(),
                "/dev/null".into(),
            ],
            Operation::Run => temporary_source_run("rustc", relative),
        });
    };
    let package_root = manifest.parent().context("Manifest has no parent")?;
    let binary = cargo_binary_for_source(package_root, &document, path)?;
    let mut command = vec![
        "cargo".into(),
        match (operation, binary.is_some()) {
            (Operation::Build, false) => "check",
            (Operation::Build, true) => "build",
            (Operation::Run, true) => "run",
            (Operation::Run, false) => bail!("This Rust file is not a binary entry point. Choose its ROS node or a Cargo binary source"),
        }.into(),
        "--manifest-path".into(),
        manifest.strip_prefix(root)?.to_string_lossy().into_owned(),
    ];
    if let Some(binary) = binary {
        command.extend(["--bin".into(), binary]);
    }
    Ok(command)
}

fn cargo_binary_for_source(
    package_root: &Path,
    document: &toml::Value,
    path: &Path,
) -> Result<Option<String>> {
    let targets =
        ros_studio_scan::cargo::binary_targets(package_root, &toml::to_string(document)?)?;
    Ok(ros_studio_scan::cargo::binary_for_source(&targets, path).map(str::to_owned))
}

pub fn node_execution(
    project: &Project,
    selected: &str,
    operation: Operation,
) -> Result<Execution> {
    let node = project
        .nodes
        .iter()
        .find(|node| node.id.as_str() == selected)
        .context("Select a source node in Design or Both before Build/Run")?;
    let package = project
        .packages
        .iter()
        .find(|package| package.id == node.package_id)
        .context("Node package is missing")?;
    ensure!(!node.executable.is_empty(), "Node has no executable name");
    let root = Path::new(&project.root_path).canonicalize()?;
    let package_root = root
        .join(&package.path)
        .canonicalize()
        .context("Package folder is missing")?;
    ensure!(
        package_root.starts_with(&root),
        "Package is outside the project"
    );
    let rust = node
        .source_locations
        .iter()
        .any(|location| location.path.ends_with(".rs"));
    let cpp = node.source_locations.iter().any(|location| {
        matches!(
            Path::new(&location.path)
                .extension()
                .and_then(|extension| extension.to_str()),
            Some("c" | "cc" | "cpp" | "cxx")
        )
    });
    let command = if rust {
        let manifest = package_root
            .join("Cargo.toml")
            .canonicalize()
            .context("Rust package has no Cargo.toml")?;
        ensure!(
            manifest.starts_with(&root),
            "Manifest is outside the project"
        );
        let targets = ros_studio_scan::cargo::binary_targets(
            &package_root,
            &std::fs::read_to_string(&manifest)?,
        )?;
        let mut source_binary = None;
        for location in node
            .source_locations
            .iter()
            .filter(|location| location.path.ends_with(".rs"))
        {
            let source = root.join(&location.path).canonicalize()?;
            ensure!(source.starts_with(&root), "Source is outside the project");
            if let Some(binary) = ros_studio_scan::cargo::binary_for_source(&targets, &source) {
                ensure!(
                    source_binary.is_none_or(|previous| previous == binary),
                    "Node belongs to several Cargo binaries"
                );
                source_binary = Some(binary);
            }
        }
        let binary = source_binary
            .or_else(|| {
                targets
                    .iter()
                    .find(|target| target.name == node.executable)
                    .map(|target| target.name.as_str())
            })
            .context("Cannot identify this node's Cargo binary; choose its binary source file")?;
        vec![
            "cargo".into(),
            match operation {
                Operation::Build => "build",
                Operation::Run => "run",
            }
            .into(),
            "--manifest-path".into(),
            manifest.strip_prefix(&root)?.to_string_lossy().into_owned(),
            "--bin".into(),
            binary.into(),
        ]
    } else if operation == Operation::Build {
        let mut command = vec![
            "colcon".into(),
            "build".into(),
            "--packages-select".into(),
            package.name.clone(),
        ];
        if cpp {
            command.extend([
                "--cmake-args".into(),
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON".into(),
            ]);
        }
        command
    } else {
        let cmake = if cpp {
            " --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        } else {
            ""
        };
        vec![
            "bash".into(),
            "-c".into(),
            format!(
                "colcon build --packages-select \"$1\"{cmake} && . install/setup.bash && exec ros2 run \"$1\" \"$2\""
            ),
            "ros-studio-run".into(),
            package.name.clone(),
            node.executable.clone(),
        ]
    };
    Ok(Execution {
        operation,
        label: format!("{} {}", operation.label(), node.logical_name),
        command,
    })
}

pub const MAX_CONSOLE_LINES: usize = 5000;
const MAX_LINE_BYTES: usize = 16384;

#[derive(Clone)]
pub struct ConsoleLine {
    pub text: String,
    pub stream: OutputStream,
}

#[derive(Default)]
pub struct ConsoleBuffer {
    pub lines: VecDeque<ConsoleLine>,
    escape: bool,
}

impl ConsoleBuffer {
    pub fn append(&mut self, text: &str, stream: OutputStream) -> bool {
        let mut trimmed = false;
        for character in text.chars() {
            if self.escape {
                if character.is_ascii_alphabetic() {
                    self.escape = false;
                }
                continue;
            }
            if character == '\u{1b}' {
                self.escape = true;
                continue;
            }
            if character == '\r' {
                continue;
            }
            if self.lines.is_empty() {
                self.lines.push_back(ConsoleLine {
                    text: String::new(),
                    stream,
                });
            }
            if character == '\n'
                || self
                    .lines
                    .back()
                    .is_some_and(|line| line.text.len() >= MAX_LINE_BYTES)
            {
                self.lines.push_back(ConsoleLine {
                    text: String::new(),
                    stream,
                });
            }
            if character != '\n'
                && let Some(line) = self.lines.back_mut()
            {
                line.stream = stream;
                line.text
                    .push(if character == '\t' { ' ' } else { character });
            }
            while self.lines.len() > MAX_CONSOLE_LINES {
                self.lines.pop_front();
                trimmed = true;
            }
        }
        trimmed
    }

    pub fn text(&self) -> String {
        self.lines
            .iter()
            .map(|line| line.text.as_str())
            .collect::<Vec<_>>()
            .join("\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GraphSnapshot;

    #[test]
    fn selected_node_is_a_target_without_configuration() {
        assert_eq!(
            RunTarget::for_selection(Some("node:camera"), None),
            Some(RunTarget::Node("node:camera".into()))
        );
        assert_eq!(RunTarget::for_selection(None, None), None);
    }

    #[test]
    fn selected_node_overrides_a_previous_node_or_file_configuration() {
        for configured in [
            RunTarget::Node("node:detector".into()),
            RunTarget::File("test.py".into()),
            RunTarget::Make(crate::makefile::MakeTarget {
                makefile: "Makefile".into(),
                target: "ros-run".into(),
            }),
        ] {
            assert_eq!(
                RunTarget::for_selection(Some("node:camera"), Some(&configured)),
                Some(RunTarget::Node("node:camera".into()))
            );
        }
    }

    #[test]
    fn no_selection_reuses_the_chosen_or_restored_configuration() {
        for configured in [
            RunTarget::Node("node:camera".into()),
            RunTarget::File("test.py".into()),
            RunTarget::Make(crate::makefile::MakeTarget {
                makefile: "Makefile".into(),
                target: "ros-run".into(),
            }),
        ] {
            assert_eq!(
                RunTarget::for_selection(None, Some(&configured)),
                Some(configured)
            );
        }
    }

    #[test]
    fn file_commands_keep_paths_as_arguments_and_reject_outside_sources() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let mut project = design()?;
        project.root_path = directory.path().to_string_lossy().into_owned();
        let source = "-camera ; $(touch nope).py";
        std::fs::write(directory.path().join(source), "print('safe')")?;
        let execution = RunTarget::File(source.into()).execution(&project, Operation::Run)?;
        assert_eq!(
            execution.command,
            vec!["python3", "./-camera ; $(touch nope).py"]
        );
        let build = source_execution(&project, source, Operation::Build)?;
        assert_eq!(build.command.first().map(String::as_str), Some("python3"));
        let outside = tempfile::NamedTempFile::new()?;
        assert!(
            source_execution(
                &project,
                outside.path().to_str().context("Non UTF-8 path")?,
                Operation::Run
            )
            .is_err()
        );
        assert!(source_execution(&project, "missing.py", Operation::Run).is_err());
        assert!(!directory.path().join("nope").exists());
        Ok(())
    }

    #[test]
    fn rust_source_uses_the_matching_cargo_binary_and_does_not_run_libraries() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let mut project = design()?;
        project.root_path = directory.path().to_string_lossy().into_owned();
        std::fs::create_dir_all(directory.path().join("src/bin"))?;
        std::fs::write(
            directory.path().join("Cargo.toml"),
            "[package]\nname='demo'\nversion='0.1.0'\nedition='2024'\n[[bin]]\nname='bridge'\npath='src/bridge.rs'\n",
        )?;
        for source in [
            "src/main.rs",
            "src/bridge.rs",
            "src/bin/camera.rs",
            "src/lib.rs",
        ] {
            std::fs::write(directory.path().join(source), "fn main() {}")?;
        }
        for (source, binary) in [
            ("src/main.rs", "demo"),
            ("src/bridge.rs", "bridge"),
            ("src/bin/camera.rs", "camera"),
        ] {
            let execution = source_execution(&project, source, Operation::Run)?;
            assert_eq!(
                execution.command,
                vec![
                    "cargo",
                    "run",
                    "--manifest-path",
                    "Cargo.toml",
                    "--bin",
                    binary
                ]
            );
        }
        assert!(source_execution(&project, "src/lib.rs", Operation::Run).is_err());
        assert_eq!(
            source_execution(&project, "src/lib.rs", Operation::Build)?
                .command
                .get(1)
                .map(String::as_str),
            Some("check")
        );
        Ok(())
    }

    #[test]
    fn graph_and_source_commands_use_cargo_names_instead_of_ros_package_names() -> Result<()> {
        for (source, bins, expected) in [
            ("src/main.rs", "", "cargo_name"),
            (
                "src/entry.rs",
                "[[bin]]\nname='explicit_name'\npath='src/entry.rs'\n",
                "explicit_name",
            ),
            ("src/bin/directory/main.rs", "", "directory"),
        ] {
            let root = tempfile::tempdir()?;
            let package = root.path().join("src/ros_name");
            let path = package.join(source);
            std::fs::create_dir_all(path.parent().context("Missing parent")?)?;
            std::fs::write(
                package.join("package.xml"),
                "<package><name>ros_name</name></package>",
            )?;
            std::fs::write(
                package.join("Cargo.toml"),
                format!("[package]\nname='cargo_name'\nversion='0.1.0'\nedition='2024'\n{bins}"),
            )?;
            std::fs::write(
                path,
                "fn main() { let node = executor.create_node(\"camera\")?; }",
            )?;
            let project = GraphSnapshot::scan(root.path())?.project;
            let node = project.nodes.first().context("Missing node")?;
            assert_eq!(node.executable, expected);
            for operation in [Operation::Build, Operation::Run] {
                let graph_command = node_execution(&project, node.id.as_str(), operation)?.command;
                let file_command =
                    source_execution(&project, &format!("src/ros_name/{source}"), operation)?
                        .command;
                assert_eq!(graph_command, file_command);
                assert_eq!(graph_command.last().map(String::as_str), Some(expected));
            }
        }
        Ok(())
    }

    #[test]
    fn standalone_c_and_rust_runs_use_unique_temporary_executables() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let mut project = design()?;
        project.root_path = directory.path().to_string_lossy().into_owned();
        for source in ["main.c", "main.cpp", "main.rs"] {
            std::fs::write(directory.path().join(source), "")?;
            let execution = source_execution(&project, source, Operation::Run)?;
            assert_eq!(execution.command.first().map(String::as_str), Some("bash"));
            assert!(
                execution
                    .command
                    .get(2)
                    .is_some_and(|script| script.contains("mktemp") && script.contains("trap"))
            );
            assert_eq!(
                execution.command.last().map(String::as_str),
                Some(format!("./{source}").as_str())
            );
        }
        Ok(())
    }

    #[test]
    fn a_virtual_workspace_does_not_build_every_package_for_an_unowned_file() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let mut project = design()?;
        project.root_path = directory.path().to_string_lossy().into_owned();
        std::fs::write(
            directory.path().join("Cargo.toml"),
            "[workspace]\nmembers=[]\n",
        )?;
        std::fs::write(directory.path().join("scratch.rs"), "fn main() {}")?;
        let execution = source_execution(&project, "scratch.rs", Operation::Build)?;
        assert_eq!(execution.command.first().map(String::as_str), Some("rustc"));
        Ok(())
    }

    fn design() -> Result<Project> {
        Ok(serde_json::from_str(include_str!(
            "../../../examples/drone_demo_ws/expected_graph.json"
        ))?)
    }

    #[test]
    fn merges_runtime_and_removals_without_leaking_into_design() -> Result<()> {
        let project = design()?;
        let mut node = project.nodes.first().context("Missing node")?.clone();
        node.id = EntityId::new("node:runtime:camera");
        node.runtime_state = RuntimeState::Online;
        let mut overlay = RuntimeOverlay::default();
        let patch = GraphPatch {
            project_id: project.id.clone(),
            upsert_packages: project.packages.clone(),
            removed_package_ids: vec![],
            upsert_nodes: vec![node.clone()],
            removed_node_ids: vec![],
        };
        let mut foreign = patch.clone();
        foreign.project_id = EntityId::new("project:other");
        assert!(!overlay.apply(foreign, &project.id));
        assert!(overlay.project(&project, GraphMode::Live).nodes.is_empty());
        assert!(overlay.apply(patch, &project.id));
        assert_eq!(overlay.project(&project, GraphMode::Live).nodes.len(), 1);
        assert_eq!(overlay.project(&project, GraphMode::Design), project);
        assert!(
            overlay
                .project(&project, GraphMode::Both)
                .nodes
                .iter()
                .any(|node| node.runtime_state == RuntimeState::Offline)
        );
        assert!(overlay.apply(
            GraphPatch {
                project_id: project.id.clone(),
                upsert_packages: vec![],
                removed_package_ids: vec![],
                upsert_nodes: vec![],
                removed_node_ids: vec![node.id]
            },
            &project.id
        ));
        assert!(overlay.project(&project, GraphMode::Live).nodes.is_empty());
        Ok(())
    }

    #[test]
    fn builds_rust_nodes_with_relative_manifest_and_separate_arguments() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::create_dir_all(root.path().join("src/camera/src"))?;
        std::fs::write(
            root.path().join("src/camera/package.xml"),
            "<package><name>camera</name></package>",
        )?;
        std::fs::write(
            root.path().join("src/camera/Cargo.toml"),
            "[package]\nname=\"camera\"\nversion=\"0.1.0\"\n",
        )?;
        std::fs::write(
            root.path().join("src/camera/src/main.rs"),
            "let node = executor.create_node(\"camera\")?;",
        )?;
        let project = GraphSnapshot::scan(root.path())?.project;
        let node = project.nodes.first().context("Missing node")?;
        let build = node_execution(&project, node.id.as_str(), Operation::Build)?;
        assert_eq!(
            build.command,
            [
                "cargo",
                "build",
                "--manifest-path",
                "src/camera/Cargo.toml",
                "--bin",
                "camera"
            ]
        );
        assert_eq!(
            node_execution(&project, node.id.as_str(), Operation::Run)?
                .command
                .get(1)
                .map(String::as_str),
            Some("run")
        );
        assert!(node_execution(&project, "runtime-only", Operation::Run).is_err());
        Ok(())
    }

    #[test]
    fn bounds_console_output_and_preserves_partial_lines() {
        let mut buffer = ConsoleBuffer::default();
        buffer.append("hel", OutputStream::Stdout);
        buffer.append("lo\u{1b}[31m world\u{1b}[0m\nerror", OutputStream::Stderr);
        assert_eq!(buffer.text(), "hello world\nerror");
        for _ in 0..MAX_CONSOLE_LINES {
            buffer.append("\nnext", OutputStream::Stdout);
        }
        assert_eq!(buffer.lines.len(), MAX_CONSOLE_LINES);
    }

    #[test]
    fn colcon_commands_keep_package_and_executable_names_as_arguments() -> Result<()> {
        let root = tempfile::tempdir()?;
        let package = root.path().join("src/controller");
        std::fs::create_dir_all(&package)?;
        let mut project = design()?;
        project.root_path = root.path().to_string_lossy().into_owned();
        let mut node = project.nodes.first().context("Missing node")?.clone();
        let package_id = node.package_id.clone();
        node.executable = "controller; printf injected".into();
        node.source_locations = vec![ros_studio_model::SourceLocation {
            path: "src/controller/main.py".into(),
            line: 0,
            column: 0,
        }];
        project.packages = vec![Package {
            id: package_id,
            name: "controller with spaces".into(),
            path: "src/controller".into(),
        }];
        project.nodes = vec![node.clone()];
        let build = node_execution(&project, node.id.as_str(), Operation::Build)?;
        assert_eq!(
            build.command,
            [
                "colcon",
                "build",
                "--packages-select",
                "controller with spaces"
            ]
        );
        let run = node_execution(&project, node.id.as_str(), Operation::Run)?;
        assert_eq!(run.command.last(), Some(&node.executable));
        assert!(
            !run.command
                .get(2)
                .context("Missing shell source")?
                .contains(&node.executable)
        );
        project
            .nodes
            .first_mut()
            .context("Missing node")?
            .source_locations
            .first_mut()
            .context("Missing source")?
            .path = "src/controller/main.cpp".into();
        assert!(
            node_execution(&project, node.id.as_str(), Operation::Build)?
                .command
                .contains(&"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON".into())
        );
        project
            .packages
            .first_mut()
            .context("Missing package")?
            .path = root
            .path()
            .parent()
            .context("Missing parent")?
            .to_string_lossy()
            .into_owned();
        assert!(node_execution(&project, node.id.as_str(), Operation::Run).is_err());
        Ok(())
    }
}
