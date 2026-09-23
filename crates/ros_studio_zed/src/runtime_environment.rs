use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    process::Stdio,
};

use anyhow::{Context as _, Result, bail};
use async_channel::Sender;
use async_process::{Child, Command};
use futures::{StreamExt as _, future, io::AsyncBufReadExt as _, io::AsyncRead, io::BufReader};
use serde::Deserialize;

const CONTAINER_DAEMON_PATH: &str = "/tmp/ros-studio-daemon";
const CONTAINER_DAEMON_COMMAND: &str = concat!(
    ". /opt/ros/humble/setup.bash && ",
    "if [ -f /opt/ros2-rust-overlay/install/setup.bash ]; then ",
    ". /opt/ros2-rust-overlay/install/setup.bash; fi && ",
    "if [ -f ./install/setup.bash ]; then . ./install/setup.bash; fi && ",
    "exec /tmp/ros-studio-daemon",
);
const HOST_DAEMON_COMMAND: &str = concat!(
    ". \"$1\" && ",
    "if [ -f \"$2\" ]; then . \"$2\"; fi && ",
    "if [ -f \"$3\" ]; then . \"$3\"; fi && ",
    "exec \"$4\"",
);
const DEFAULT_DOCKER_IMAGE: &str = "rust-drone-ros2-humble:local";
const ROS_HUMBLE_SETUP: &str = "/opt/ros/humble/setup.bash";
const ROS_RUST_OVERLAY_SETUP: &str = "/opt/ros2-rust-overlay/install/setup.bash";
const COMPOSE_FILE_NAMES: [&str; 4] = [
    "compose.yaml",
    "compose.yml",
    "docker-compose.yaml",
    "docker-compose.yml",
];
const FORWARDED_ROS_VARIABLES: [&str; 3] =
    ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"];

pub(super) struct DaemonLaunch {
    command: Command,
    workspace_root: String,
    environment_label: String,
    preparation: Option<Command>,
}

impl DaemonLaunch {
    pub(super) fn workspace_root(&self) -> &str {
        &self.workspace_root
    }

    pub(super) fn environment_label(&self) -> &str {
        &self.environment_label
    }

    pub(super) async fn spawn(
        mut self,
        on_preparation_output: impl FnMut(String) -> Result<()>,
    ) -> Result<Child> {
        if let Some(mut preparation) = self.preparation.take() {
            prepare_environment(&mut preparation, on_preparation_output).await?;
        }

        self.command.spawn().context("failed to start ROS 2 daemon")
    }
}

async fn prepare_environment(
    command: &mut Command,
    mut on_output: impl FnMut(String) -> Result<()>,
) -> Result<()> {
    command.stdout(Stdio::piped()).stderr(Stdio::piped());
    let mut child = command
        .spawn()
        .context("failed to prepare the ROS 2 Docker environment")?;
    let stdout = child
        .stdout
        .take()
        .context("ROS 2 Docker preparation has no stdout")?;
    let stderr = child
        .stderr
        .take()
        .context("ROS 2 Docker preparation has no stderr")?;
    let (output_sender, output_receiver) = async_channel::unbounded();
    let stdout_task = forward_preparation_output(stdout, output_sender.clone());
    let stderr_task = forward_preparation_output(stderr, output_sender);
    let output_task = async move {
        while let Ok(text) = output_receiver.recv().await {
            on_output(text)?;
        }
        Result::<()>::Ok(())
    };
    let status_task = async {
        child
            .status()
            .await
            .context("failed to wait for ROS 2 Docker preparation")
    };
    let (_, _, _, status) =
        future::try_join4(stdout_task, stderr_task, output_task, status_task).await?;
    if !status.success() {
        bail!("ROS 2 Docker environment preparation failed ({status})");
    }
    Ok(())
}

async fn forward_preparation_output(
    reader: impl AsyncRead + Unpin,
    sender: Sender<String>,
) -> Result<()> {
    let mut lines = BufReader::new(reader).lines();
    while let Some(line) = lines.next().await {
        sender
            .send(format!(
                "{}\n",
                line.context("failed to read Docker output")?
            ))
            .await?;
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RuntimePreference {
    Auto,
    Host,
    Docker,
}

impl RuntimePreference {
    fn from_environment() -> Result<Self> {
        match std::env::var("ROS_STUDIO_RUNTIME")
            .unwrap_or_else(|_| "auto".to_owned())
            .to_ascii_lowercase()
            .as_str()
        {
            "auto" => Ok(Self::Auto),
            "host" => Ok(Self::Host),
            "docker" => Ok(Self::Docker),
            _ => bail!("ROS_STUDIO_RUNTIME must be auto, host, or docker"),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum RuntimeBackend {
    Host { distribution: String },
    Compose(ComposeEnvironment),
    Docker { image: String },
}

#[derive(Clone, Debug, Eq, PartialEq)]
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

pub(super) fn detect_runtime(workspace_root: &Path) -> Result<DaemonLaunch> {
    if let Some(path) = std::env::var_os("ROS_STUDIO_DAEMON_PATH") {
        return Ok(custom_daemon_launch(workspace_root, path.into()));
    }

    let daemon_binary = resolve_default_daemon_binary()?
        .canonicalize()
        .context("failed to resolve ROS 2 daemon path")?;
    let preference = RuntimePreference::from_environment()?;
    let inherited_host_distribution = inherited_host_distribution();
    let discovered_host_distribution = discovered_host_distribution();
    let compose_environment = detect_compose_environment(
        workspace_root,
        std::env::var("ROS_STUDIO_COMPOSE_SERVICE").ok().as_deref(),
    )?;
    let fallback_image = std::env::var("ROS_STUDIO_DOCKER_IMAGE")
        .unwrap_or_else(|_| DEFAULT_DOCKER_IMAGE.to_owned());
    let backend = select_runtime_backend(
        preference,
        inherited_host_distribution,
        discovered_host_distribution,
        compose_environment,
        fallback_image,
    )?;

    match backend {
        RuntimeBackend::Host { distribution } => Ok(host_daemon_launch(
            workspace_root,
            &daemon_binary,
            &distribution,
        )),
        RuntimeBackend::Compose(environment) => {
            compose_daemon_launch(workspace_root, &daemon_binary, &environment)
        }
        RuntimeBackend::Docker { image } => {
            docker_daemon_launch(workspace_root, &daemon_binary, &image)
        }
    }
}

fn select_runtime_backend(
    preference: RuntimePreference,
    inherited_host_distribution: Option<String>,
    discovered_host_distribution: Option<String>,
    compose_environment: Option<ComposeEnvironment>,
    fallback_image: String,
) -> Result<RuntimeBackend> {
    match preference {
        RuntimePreference::Host => inherited_host_distribution
            .or(discovered_host_distribution)
            .map(|distribution| RuntimeBackend::Host { distribution })
            .context("ROS Humble was not found on the host at /opt/ros/humble"),
        RuntimePreference::Docker => Ok(compose_environment
            .map(RuntimeBackend::Compose)
            .unwrap_or(RuntimeBackend::Docker {
                image: fallback_image,
            })),
        RuntimePreference::Auto => {
            if let Some(distribution) = inherited_host_distribution {
                Ok(RuntimeBackend::Host { distribution })
            } else if let Some(environment) = compose_environment {
                Ok(RuntimeBackend::Compose(environment))
            } else if let Some(distribution) = discovered_host_distribution {
                Ok(RuntimeBackend::Host { distribution })
            } else {
                Ok(RuntimeBackend::Docker {
                    image: fallback_image,
                })
            }
        }
    }
}

fn inherited_host_distribution() -> Option<String> {
    std::env::var("ROS_DISTRO")
        .ok()
        .filter(|distribution| distribution == "humble")
        .filter(|_| Path::new(ROS_HUMBLE_SETUP).is_file())
}

fn discovered_host_distribution() -> Option<String> {
    Path::new(ROS_HUMBLE_SETUP)
        .is_file()
        .then(|| "humble".to_owned())
}

fn custom_daemon_launch(workspace_root: &Path, daemon_binary: PathBuf) -> DaemonLaunch {
    let mut command = Command::new(daemon_binary);
    configure_protocol_command(&mut command, workspace_root);
    DaemonLaunch {
        command,
        workspace_root: workspace_root.to_string_lossy().into_owned(),
        environment_label: "CUSTOM DAEMON".to_owned(),
        preparation: None,
    }
}

fn host_daemon_launch(
    workspace_root: &Path,
    daemon_binary: &Path,
    distribution: &str,
) -> DaemonLaunch {
    let workspace_setup = workspace_root.join("install/setup.bash");
    let mut command = Command::new("bash");
    command
        .args([
            "-lc",
            HOST_DAEMON_COMMAND,
            "ros-studio-host",
            ROS_HUMBLE_SETUP,
        ])
        .arg(ROS_RUST_OVERLAY_SETUP)
        .arg(workspace_setup)
        .arg(daemon_binary);
    configure_protocol_command(&mut command, workspace_root);
    DaemonLaunch {
        command,
        workspace_root: workspace_root.to_string_lossy().into_owned(),
        environment_label: format!("HOST · {}", distribution.to_ascii_uppercase()),
        preparation: None,
    }
}

fn compose_daemon_launch(
    workspace_root: &Path,
    daemon_binary: &Path,
    environment: &ComposeEnvironment,
) -> Result<DaemonLaunch> {
    let daemon_workspace_root = container_workspace_root(workspace_root)?;
    let mut command = Command::new("docker");
    command
        .args(["compose", "--file"])
        .arg(&environment.file)
        .args(["run", "--rm", "--no-deps", "--no-tty"])
        .arg("--volume")
        .arg(format!(
            "{}:{CONTAINER_DAEMON_PATH}:ro",
            daemon_binary.display()
        ))
        .arg("--volume")
        .arg(format!(
            "{}:{daemon_workspace_root}",
            workspace_root.display()
        ))
        .arg("--workdir")
        .arg(&daemon_workspace_root);
    append_ros_environment(&mut command);
    command
        .arg(&environment.service)
        .args(["bash", "-lc", CONTAINER_DAEMON_COMMAND]);
    configure_protocol_command(&mut command, workspace_root);

    let preparation = environment.requires_build.then(|| {
        let mut preparation = Command::new("docker");
        preparation
            .args(["compose", "--file"])
            .arg(&environment.file)
            .arg("build")
            .arg(&environment.service)
            .current_dir(workspace_root)
            .stdin(Stdio::null())
            .kill_on_drop(true);
        preparation
    });

    Ok(DaemonLaunch {
        command,
        workspace_root: daemon_workspace_root,
        environment_label: format!("DOCKER COMPOSE · {}", environment.service),
        preparation,
    })
}

fn docker_daemon_launch(
    workspace_root: &Path,
    daemon_binary: &Path,
    image: &str,
) -> Result<DaemonLaunch> {
    let daemon_workspace_root = container_workspace_root(workspace_root)?;
    let mut command = Command::new("docker");
    command
        .args(["run", "--rm", "-i", "--network", "host", "--ipc", "host"])
        .arg("--mount")
        .arg(format!(
            "type=bind,src={},dst={CONTAINER_DAEMON_PATH},readonly",
            daemon_binary.display()
        ))
        .arg("--mount")
        .arg(format!(
            "type=bind,src={},dst={daemon_workspace_root}",
            workspace_root.display(),
        ))
        .arg("--workdir")
        .arg(&daemon_workspace_root);
    append_ros_environment(&mut command);
    command
        .arg(image)
        .args(["bash", "-lc", CONTAINER_DAEMON_COMMAND]);
    configure_protocol_command(&mut command, workspace_root);

    Ok(DaemonLaunch {
        command,
        workspace_root: daemon_workspace_root,
        environment_label: format!("DOCKER · {image}"),
        preparation: None,
    })
}

fn configure_protocol_command(command: &mut Command, workspace_root: &Path) {
    command
        .current_dir(workspace_root)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .kill_on_drop(true);
}

fn append_ros_environment(command: &mut Command) {
    for variable in FORWARDED_ROS_VARIABLES {
        if let Some(value) = std::env::var_os(variable) {
            command
                .arg("--env")
                .arg(format!("{variable}={}", value.to_string_lossy()));
        }
    }
}

fn container_workspace_root(workspace_root: &Path) -> Result<String> {
    let workspace_name = workspace_root
        .file_name()
        .and_then(|name| name.to_str())
        .context("open workspace has no valid UTF-8 directory name")?;
    Ok(format!("/workspace/{workspace_name}"))
}

fn detect_compose_environment(
    workspace_root: &Path,
    requested_service: Option<&str>,
) -> Result<Option<ComposeEnvironment>> {
    for file_name in COMPOSE_FILE_NAMES {
        let file = workspace_root.join(file_name);
        if !file.is_file() {
            continue;
        }
        let contents = std::fs::read_to_string(&file)
            .with_context(|| format!("failed to read {}", file.display()))?;
        let configuration: ComposeConfiguration = serde_yaml::from_str(&contents)
            .with_context(|| format!("failed to parse {}", file.display()))?;
        let service = select_compose_service(&configuration.services, requested_service, &file)?;
        let Some(service) = service else {
            continue;
        };
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

    Ok(None)
}

fn select_compose_service(
    services: &BTreeMap<String, ComposeService>,
    requested_service: Option<&str>,
    compose_file: &Path,
) -> Result<Option<String>> {
    if let Some(requested_service) = requested_service {
        return services
            .contains_key(requested_service)
            .then(|| Some(requested_service.to_owned()))
            .with_context(|| {
                format!(
                    "Docker Compose service {requested_service} does not exist in {}",
                    compose_file.display()
                )
            });
    }
    if services.contains_key("ros2") {
        return Ok(Some("ros2".to_owned()));
    }
    if let Some(service) = services
        .keys()
        .find(|service| service.to_ascii_lowercase().contains("ros"))
    {
        return Ok(Some(service.clone()));
    }
    if services.len() == 1 {
        return Ok(services.keys().next().cloned());
    }
    Ok(None)
}

fn resolve_default_daemon_binary() -> Result<PathBuf> {
    let manifest_directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repository_root = manifest_directory
        .parent()
        .and_then(Path::parent)
        .context("could not locate ROS 2 Studio repository")?;
    let editor_executable = std::env::current_exe().context("could not locate ROS 2 Studio")?;
    resolve_daemon_binary(&editor_executable, repository_root)
}

fn resolve_daemon_binary(editor_executable: &Path, repository_root: &Path) -> Result<PathBuf> {
    let packaged_daemon = editor_executable
        .parent()
        .context("ROS 2 Studio executable has no parent directory")?
        .join("ros-studio-daemon");
    let release_daemon = repository_root.join("target/ros-humble/release/ros-studio-daemon");
    let debug_daemon = repository_root.join("target/ros-humble/debug/ros-studio-daemon");

    for candidate in [&packaged_daemon, &release_daemon, &debug_daemon] {
        if candidate.is_file() {
            return Ok(candidate.clone());
        }
    }

    bail!(
        "ROS 2 daemon is missing; expected a packaged daemon at {} or a development build at {} or {}",
        packaged_daemon.display(),
        release_daemon.display(),
        debug_daemon.display()
    )
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::*;

    fn compose_environment(service: &str) -> ComposeEnvironment {
        ComposeEnvironment {
            file: PathBuf::from("compose.yaml"),
            service: service.to_owned(),
            requires_build: false,
        }
    }

    #[test]
    fn automatic_selection_prefers_inherited_host() -> Result<()> {
        let selected = select_runtime_backend(
            RuntimePreference::Auto,
            Some("humble".to_owned()),
            Some("humble".to_owned()),
            Some(compose_environment("ros2")),
            "fallback".to_owned(),
        )?;
        assert_eq!(
            selected,
            RuntimeBackend::Host {
                distribution: "humble".to_owned()
            }
        );
        Ok(())
    }

    #[test]
    fn automatic_selection_prefers_compose_over_discovered_host() -> Result<()> {
        let environment = compose_environment("ros2");
        let selected = select_runtime_backend(
            RuntimePreference::Auto,
            None,
            Some("humble".to_owned()),
            Some(environment.clone()),
            "fallback".to_owned(),
        )?;
        assert_eq!(selected, RuntimeBackend::Compose(environment));
        Ok(())
    }

    #[test]
    fn docker_selection_uses_compose_when_available() -> Result<()> {
        let environment = compose_environment("simulation");
        let selected = select_runtime_backend(
            RuntimePreference::Docker,
            Some("humble".to_owned()),
            Some("humble".to_owned()),
            Some(environment.clone()),
            "fallback".to_owned(),
        )?;
        assert_eq!(selected, RuntimeBackend::Compose(environment));
        Ok(())
    }

    #[test]
    fn detects_ros2_compose_service() -> Result<()> {
        let root = tempfile::tempdir()?;
        fs::write(
            root.path().join("compose.yaml"),
            "services:\n  database:\n    image: postgres\n  ros2:\n    image: ros:humble\n",
        )?;

        assert_eq!(
            detect_compose_environment(root.path(), None)?,
            Some(ComposeEnvironment {
                file: root.path().join("compose.yaml"),
                service: "ros2".to_owned(),
                requires_build: false,
            })
        );
        Ok(())
    }

    #[test]
    fn honors_requested_compose_service() -> Result<()> {
        let root = tempfile::tempdir()?;
        fs::write(
            root.path().join("docker-compose.yml"),
            "services:\n  development:\n    image: ros:humble\n  simulation:\n    image: ros:humble\n",
        )?;

        assert_eq!(
            detect_compose_environment(root.path(), Some("simulation"))?,
            Some(ComposeEnvironment {
                file: root.path().join("docker-compose.yml"),
                service: "simulation".to_owned(),
                requires_build: false,
            })
        );
        Ok(())
    }

    #[test]
    fn marks_compose_service_that_requires_a_build() -> Result<()> {
        let root = tempfile::tempdir()?;
        fs::write(
            root.path().join("compose.yaml"),
            "services:\n  ros2:\n    build:\n      context: .\n    image: project-ros:local\n",
        )?;

        assert_eq!(
            detect_compose_environment(root.path(), None)?,
            Some(ComposeEnvironment {
                file: root.path().join("compose.yaml"),
                service: "ros2".to_owned(),
                requires_build: true,
            })
        );
        Ok(())
    }

    #[test]
    fn packaged_daemon_takes_priority() -> Result<()> {
        let root = tempfile::tempdir()?;
        let editor_executable = root.path().join("app/libexec/zed-editor");
        let packaged_daemon = root.path().join("app/libexec/ros-studio-daemon");
        let repository_root = root.path().join("repository");
        let development_daemon =
            repository_root.join("target/ros-humble/release/ros-studio-daemon");
        fs::create_dir_all(
            editor_executable
                .parent()
                .context("fixture executable has no parent")?,
        )?;
        fs::create_dir_all(
            development_daemon
                .parent()
                .context("fixture daemon has no parent")?,
        )?;
        fs::write(&packaged_daemon, [])?;
        fs::write(development_daemon, [])?;

        assert_eq!(
            resolve_daemon_binary(&editor_executable, &repository_root)?,
            packaged_daemon
        );
        Ok(())
    }

    #[test]
    fn development_release_daemon_precedes_debug_daemon() -> Result<()> {
        let root = tempfile::tempdir()?;
        let editor_executable = root.path().join("target/release-fast/zed");
        let repository_root = root.path().join("repository");
        let release_daemon = repository_root.join("target/ros-humble/release/ros-studio-daemon");
        let debug_daemon = repository_root.join("target/ros-humble/debug/ros-studio-daemon");
        fs::create_dir_all(
            release_daemon
                .parent()
                .context("fixture release daemon has no parent")?,
        )?;
        fs::create_dir_all(
            debug_daemon
                .parent()
                .context("fixture debug daemon has no parent")?,
        )?;
        fs::write(&release_daemon, [])?;
        fs::write(debug_daemon, [])?;

        assert_eq!(
            resolve_daemon_binary(&editor_executable, &repository_root)?,
            release_daemon
        );
        Ok(())
    }
}
