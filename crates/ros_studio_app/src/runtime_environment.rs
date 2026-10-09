use std::{
    collections::BTreeMap,
    ffi::OsString,
    path::{Path, PathBuf},
    process::Stdio,
};

use anyhow::{Context as _, Result, bail, ensure};
use async_process::Command;
use serde::Deserialize;

const HOST_SETUP: &str = "/opt/ros/humble/setup.bash";
const RUST_SETUP: &str = "/opt/ros2-rust-overlay/install/setup.bash";
const CONTAINER_DAEMON: &str = "/tmp/ros-studio-daemon";
const SOURCE_AND_EXEC: &str = ". \"$1\" && if [ -f \"$2\" ]; then . \"$2\"; fi && if [ -f ./install/setup.bash ]; then . ./install/setup.bash; fi && exec \"$3\"";

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RuntimePreference {
    #[default]
    Auto,
    Host,
    Docker,
}

impl RuntimePreference {
    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto",
            Self::Host => "Host",
            Self::Docker => "Docker",
        }
    }
    pub fn from_environment() -> Result<Self> {
        match std::env::var("ROS_STUDIO_RUNTIME")
            .unwrap_or_else(|_| "auto".into())
            .to_lowercase()
            .as_str()
        {
            "auto" => Ok(Self::Auto),
            "host" => Ok(Self::Host),
            "docker" => Ok(Self::Docker),
            _ => bail!("ROS_STUDIO_RUNTIME must be auto, host, or docker"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct CommandSpec {
    pub program: OsString,
    pub arguments: Vec<OsString>,
    pub directory: PathBuf,
}

impl CommandSpec {
    pub fn command(&self) -> Command {
        let mut command = Command::new(&self.program);
        command
            .args(&self.arguments)
            .current_dir(&self.directory)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        command
    }
}

#[derive(Clone, Debug)]
pub struct LaunchPlan {
    pub daemon: CommandSpec,
    pub preparation: Option<CommandSpec>,
    pub workspace: String,
    pub label: String,
}

#[derive(Clone, Debug)]
struct ComposeEnvironment {
    file: PathBuf,
    service: String,
    requires_build: bool,
}

#[derive(Deserialize)]
struct ComposeConfiguration {
    services: BTreeMap<String, ComposeService>,
}

#[derive(Deserialize)]
struct ComposeService {
    #[serde(default)]
    build: Option<serde_yaml::Value>,
}

pub fn detect_runtime(root: &Path, preference: RuntimePreference) -> Result<LaunchPlan> {
    let root = root.canonicalize().context("Project folder is missing")?;
    ensure!(
        cfg!(target_os = "linux"),
        "ROS runtime in the standalone preview is currently Linux-only"
    );
    if let Some(path) = std::env::var_os("ROS_STUDIO_DAEMON_PATH") {
        let daemon = PathBuf::from(path)
            .canonicalize()
            .context("ROS_STUDIO_DAEMON_PATH does not exist")?;
        return Ok(LaunchPlan {
            daemon: CommandSpec {
                program: daemon.into_os_string(),
                arguments: vec![],
                directory: root.clone(),
            },
            preparation: None,
            workspace: root.to_string_lossy().into_owned(),
            label: "Custom daemon".into(),
        });
    }
    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repository = directory
        .parent()
        .and_then(Path::parent)
        .context("Cannot locate development repository")?;
    let daemon = resolve_daemon(&std::env::current_exe()?, repository)?.canonicalize()?;
    let host_available = Path::new(HOST_SETUP).is_file();
    let inherited_host = host_available
        && std::env::var("ROS_DISTRO").is_ok_and(|distribution| distribution == "humble");
    // An explicitly sourced host wins; otherwise prefer the project's Compose environment.
    if preference == RuntimePreference::Host
        || (preference == RuntimePreference::Auto && inherited_host)
    {
        ensure!(
            host_available,
            "ROS Humble was not found on the host at {HOST_SETUP}"
        );
        return Ok(host_plan(&root, &daemon));
    }
    if let Some(environment) = detect_compose(
        &root,
        std::env::var("ROS_STUDIO_COMPOSE_SERVICE").ok().as_deref(),
    )? {
        return container_plan(&root, &daemon, Some(environment), "");
    }
    if preference == RuntimePreference::Auto && host_available {
        return Ok(host_plan(&root, &daemon));
    }
    let image = std::env::var("ROS_STUDIO_DOCKER_IMAGE")
        .unwrap_or_else(|_| "rust-drone-ros2-humble:local".into());
    ensure!(
        !image.trim().is_empty() && !image.starts_with('-'),
        "Invalid ROS_STUDIO_DOCKER_IMAGE"
    );
    container_plan(&root, &daemon, None, &image)
}

fn host_plan(root: &Path, daemon: &Path) -> LaunchPlan {
    LaunchPlan {
        daemon: CommandSpec {
            program: "bash".into(),
            arguments: vec![
                "-c".into(),
                SOURCE_AND_EXEC.into(),
                "ros-studio-host".into(),
                HOST_SETUP.into(),
                RUST_SETUP.into(),
                daemon.as_os_str().to_owned(),
            ],
            directory: root.to_owned(),
        },
        preparation: None,
        workspace: root.to_string_lossy().into_owned(),
        label: "Host · ROS Humble".into(),
    }
}

fn container_plan(
    root: &Path,
    daemon: &Path,
    compose: Option<ComposeEnvironment>,
    image: &str,
) -> Result<LaunchPlan> {
    let name = root
        .file_name()
        .and_then(|name| name.to_str())
        .context("Project folder name is not UTF-8")?;
    let workspace = format!("/workspace/{name}");
    for path in [root, daemon] {
        ensure!(
            !path.to_string_lossy().contains([',', ':']),
            "Docker mount paths cannot contain commas or colons: {}",
            path.display()
        );
    }
    let mut arguments: Vec<OsString>;
    let preparation;
    let label;
    if let Some(environment) = &compose {
        arguments = vec![
            "compose".into(),
            "--file".into(),
            environment.file.as_os_str().to_owned(),
            "run".into(),
            "--rm".into(),
            "--no-deps".into(),
            "--no-tty".into(),
            "--interactive".into(),
        ];
        label = format!("Docker Compose · {}", environment.service);
        preparation = environment.requires_build.then(|| CommandSpec {
            program: "docker".into(),
            arguments: vec![
                "compose".into(),
                "--file".into(),
                environment.file.as_os_str().to_owned(),
                "build".into(),
                environment.service.clone().into(),
            ],
            directory: root.to_owned(),
        });
        arguments.extend([
            "--volume".into(),
            format!("{}:{CONTAINER_DAEMON}:ro", daemon.display()).into(),
            "--volume".into(),
            format!("{}:{workspace}", root.display()).into(),
        ]);
    } else {
        arguments = vec![
            "run".into(),
            "--rm".into(),
            "-i".into(),
            "--network".into(),
            "host".into(),
            "--ipc".into(),
            "host".into(),
            "--init".into(),
        ];
        arguments.extend([
            "--mount".into(),
            format!(
                "type=bind,src={},dst={CONTAINER_DAEMON},readonly",
                daemon.display()
            )
            .into(),
            "--mount".into(),
            format!("type=bind,src={},dst={workspace}", root.display()).into(),
        ]);
        label = format!("Docker · {image}");
        preparation = None;
    }
    arguments.extend([
        "--workdir".into(),
        workspace.clone().into(),
        "--entrypoint".into(),
        "bash".into(),
    ]);
    for variable in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"] {
        if let Some(value) = std::env::var_os(variable) {
            arguments.extend([
                "--env".into(),
                format!("{variable}={}", value.to_string_lossy()).into(),
            ]);
        }
    }
    arguments.push(
        compose
            .as_ref()
            .map_or_else(
                || image.to_owned(),
                |environment| environment.service.clone(),
            )
            .into(),
    );
    arguments.extend([
        "-c".into(),
        SOURCE_AND_EXEC.into(),
        "ros-studio-container".into(),
        HOST_SETUP.into(),
        RUST_SETUP.into(),
        CONTAINER_DAEMON.into(),
    ]);
    Ok(LaunchPlan {
        daemon: CommandSpec {
            program: "docker".into(),
            arguments,
            directory: root.to_owned(),
        },
        preparation,
        workspace,
        label,
    })
}

fn detect_compose(root: &Path, requested: Option<&str>) -> Result<Option<ComposeEnvironment>> {
    for name in [
        "compose.yaml",
        "compose.yml",
        "docker-compose.yaml",
        "docker-compose.yml",
    ] {
        let file = root.join(name);
        if !file.is_file() {
            continue;
        }
        let configuration: ComposeConfiguration =
            serde_yaml::from_str(&std::fs::read_to_string(&file)?)
                .with_context(|| format!("Cannot parse {}", file.display()))?;
        let service = if let Some(requested) = requested {
            ensure!(
                !requested.starts_with('-') && configuration.services.contains_key(requested),
                "Compose service {requested} is missing or invalid"
            );
            Some(requested.to_owned())
        } else if configuration.services.contains_key("ros2") {
            Some("ros2".into())
        } else {
            configuration
                .services
                .keys()
                .find(|name| name.to_lowercase().contains("ros"))
                .cloned()
        };
        if let Some(service) = service {
            let requires_build = configuration
                .services
                .get(&service)
                .is_some_and(|service| service.build.is_some());
            return Ok(Some(ComposeEnvironment {
                file,
                service,
                requires_build,
            }));
        }
        bail!(
            "Choose the ROS service from {} with ROS_STUDIO_COMPOSE_SERVICE",
            file.display()
        );
    }
    ensure!(
        requested.is_none(),
        "ROS_STUDIO_COMPOSE_SERVICE is set but no Compose file exists"
    );
    Ok(None)
}

fn resolve_daemon(executable: &Path, repository: &Path) -> Result<PathBuf> {
    let parent = executable
        .parent()
        .context("Application executable has no parent")?;
    for path in [
        parent.join("ros-studio-daemon"),
        parent.join("../libexec/ros-studio-daemon"),
        repository.join("target/ros-humble/release/ros-studio-daemon"),
        repository.join("target/ros-humble/debug/ros-studio-daemon"),
    ] {
        if path.is_file() {
            return Ok(path);
        }
    }
    let development_hint = if repository
        .join("script/build-ros2-studio-daemon-linux")
        .is_file()
    {
        format!(
            "\nDevelopment setup: from {}, run CARGO_BUILD_JOBS=1 ./script/build-ros2-studio-daemon-linux once, then retry Connect or Build/Run. This builds the ROS-enabled daemon using the configured Docker image.",
            repository.display()
        )
    } else {
        "\nInstall a complete ROS 2 Studio package containing libexec/ros-studio-daemon.".into()
    };
    bail!(
        "ROS runtime component is missing: ros-studio-daemon was not found. Auto selects the ROS host/Docker environment; it does not build the daemon. Place a daemon built with ros-runtime next to ros2-studio.{development_hint}"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mounts_container_workspace_and_keeps_paths_out_of_shell_source() -> Result<()> {
        let root = Path::new("/tmp/project with spaces");
        let plan = container_plan(
            root,
            Path::new("/tmp/daemon with spaces"),
            None,
            "test-image",
        )?;
        assert_eq!(plan.workspace, "/workspace/project with spaces");
        assert!(plan.daemon.arguments.contains(&OsString::from(
            "type=bind,src=/tmp/project with spaces,dst=/workspace/project with spaces"
        )));
        assert!(!SOURCE_AND_EXEC.contains("project with spaces"));
        assert!(
            container_plan(
                Path::new("/tmp/project,invalid"),
                Path::new("/tmp/daemon"),
                None,
                "test"
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn selects_compose_ros_service_and_prepares_build_separately() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("compose.yaml"),
            "services:\n  ros2:\n    build: .\n  database:\n    image: postgres\n",
        )?;
        let environment = detect_compose(root.path(), None)?.context("Missing service")?;
        let plan = container_plan(root.path(), Path::new("/tmp/daemon"), Some(environment), "")?;
        assert!(plan.preparation.is_some());
        assert!(!plan.daemon.arguments.contains(&OsString::from("--build")));
        assert!(detect_compose(root.path(), Some("missing")).is_err());
        Ok(())
    }

    #[test]
    fn prefers_packaged_daemon_and_reports_missing_binary() -> Result<()> {
        let root = tempfile::tempdir()?;
        let error = resolve_daemon(&root.path().join("ros2-studio"), root.path())
            .err()
            .context("a missing daemon must be reported")?;
        assert!(error.to_string().contains("Auto selects"));
        assert!(error.to_string().contains("complete ROS 2 Studio package"));
        std::fs::write(root.path().join("ros-studio-daemon"), "test")?;
        assert_eq!(
            resolve_daemon(&root.path().join("ros2-studio"), root.path())?,
            root.path().join("ros-studio-daemon")
        );
        Ok(())
    }

    #[test]
    fn missing_development_daemon_explains_how_to_build_it() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::create_dir(root.path().join("script"))?;
        std::fs::write(
            root.path().join("script/build-ros2-studio-daemon-linux"),
            "test",
        )?;
        let error = resolve_daemon(&root.path().join("ros2-studio"), root.path())
            .err()
            .context("a missing development daemon must be reported")?;
        let message = error.to_string();
        assert!(message.contains("CARGO_BUILD_JOBS=1 ./script/build-ros2-studio-daemon-linux"));
        assert!(message.contains("does not build the daemon"));
        Ok(())
    }
}
