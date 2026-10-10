use std::{
    collections::BTreeMap,
    io::Write as _,
    path::{Path, PathBuf},
};

use anyhow::{Context as _, Result, ensure};
use ros_studio_protocol::health::{TopicWatch, validate_watches};
use serde::{Deserialize, Serialize};

use crate::{ExternalEditor, runtime::RunTarget, runtime_environment::RuntimePreference};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct Preferences {
    pub version: u32,
    pub editor: ExternalEditor,
    pub environment: RuntimePreference,
    pub auto_mode: bool,
    pub console_height: f32,
    pub last_project: Option<PathBuf>,
    pub configurations: BTreeMap<String, RunTarget>,
    pub health_topics: BTreeMap<String, Vec<TopicWatch>>,
}

impl Default for Preferences {
    fn default() -> Self {
        Self {
            version: 1,
            editor: ExternalEditor::default(),
            environment: RuntimePreference::default(),
            auto_mode: true,
            console_height: 240.0,
            last_project: None,
            configurations: BTreeMap::new(),
            health_topics: BTreeMap::new(),
        }
    }
}

impl Preferences {
    /// Explicit CLI paths take precedence, including invalid ones: the normal
    /// project-opening flow reports their errors instead of opening another project.
    pub fn startup_project(&self, explicit: Option<PathBuf>) -> Option<PathBuf> {
        explicit.or_else(|| {
            let saved = self.last_project.as_ref()?;
            // Remembered paths must not depend on the launcher's working directory.
            if !saved.is_absolute() {
                return None;
            }
            let root = saved.canonicalize().ok()?;
            std::fs::read_dir(&root).ok()?;
            Some(root)
        })
    }

    pub fn load(path: &Path) -> Result<Self> {
        let contents = match std::fs::read(path) {
            Ok(contents) => contents,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                return Ok(Self::default());
            }
            Err(error) => return Err(error).context("Cannot read application preferences"),
        };
        let preferences: Self = serde_json::from_slice(&contents)
            .context("Invalid preferences JSON; the file has not been overwritten")?;
        ensure!(preferences.version == 1, "Unsupported preferences version");
        ensure!(
            preferences.console_height.is_finite(),
            "Invalid console height"
        );
        for topics in preferences.health_topics.values() {
            validate_watches(topics).map_err(anyhow::Error::msg)?;
        }
        Ok(preferences)
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        let directory = path.parent().context("Preferences path has no parent")?;
        std::fs::create_dir_all(directory).context("Cannot create preferences directory")?;
        let mut temporary = tempfile::NamedTempFile::new_in(directory)?;
        serde_json::to_writer_pretty(&mut temporary, self)?;
        temporary.write_all(b"\n")?;
        temporary.as_file().sync_all()?;
        temporary
            .persist(path)
            .map_err(|error| error.error)
            .context("Cannot save preferences atomically")?;
        Ok(())
    }
}

pub fn preferences_path() -> Result<PathBuf> {
    if let Some(path) = std::env::var_os("ROS_STUDIO_SETTINGS_PATH") {
        return Ok(PathBuf::from(path));
    }
    let directory = if cfg!(target_os = "windows") {
        std::env::var_os("APPDATA").map(PathBuf::from)
    } else if cfg!(target_os = "macos") {
        std::env::var_os("HOME").map(|home| PathBuf::from(home).join("Library/Application Support"))
    } else {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".config")))
    }
    .context("Cannot locate user configuration directory; set ROS_STUDIO_SETTINGS_PATH")?;
    Ok(directory.join("ros2-studio/preferences.json"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn persists_preferences_and_configuration_without_project_writes() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let path = directory.path().join("settings/preferences.json");
        let mut preferences = Preferences {
            editor: ExternalEditor::VsCode,
            auto_mode: false,
            last_project: Some(directory.path().canonicalize()?),
            ..Default::default()
        };
        preferences
            .configurations
            .insert("/project".into(), RunTarget::File("src/a file.py".into()));
        preferences.configurations.insert(
            "/make-project".into(),
            RunTarget::Make(crate::makefile::MakeTarget {
                makefile: "GNUmakefile".into(),
                target: "ros-run".into(),
            }),
        );
        preferences.save(&path)?;
        preferences.console_height = 320.0;
        preferences.health_topics.insert(
            "/project".into(),
            vec![TopicWatch {
                topic: "/imu".into(),
                type_name: "sensor_msgs/msg/Imu".into(),
                expected_hz: 200.0,
                tolerance: 0.2,
                timeout_ms: 3000,
            }],
        );
        preferences.save(&path)?;
        let loaded = Preferences::load(&path)?;
        assert_eq!(loaded.editor, ExternalEditor::VsCode);
        assert!(!loaded.auto_mode);
        assert_eq!(loaded.console_height, 320.0);
        assert_eq!(
            loaded.configurations.get("/project"),
            preferences.configurations.get("/project")
        );
        assert_eq!(loaded.configurations, preferences.configurations);
        assert_eq!(loaded.health_topics, preferences.health_topics);
        assert_eq!(loaded.last_project, preferences.last_project);
        Ok(())
    }

    #[test]
    fn restores_the_saved_project_without_depending_on_the_working_directory() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let project = directory.path().join("project with spaces");
        std::fs::create_dir(&project)?;
        let project = project.canonicalize()?;
        let path = directory.path().join("preferences.json");
        let preferences = Preferences {
            last_project: Some(project.clone()),
            ..Default::default()
        };
        preferences.save(&path)?;
        let loaded = Preferences::load(&path)?;
        assert_eq!(loaded.startup_project(None), Some(project));
        Ok(())
    }

    #[test]
    fn explicit_projects_take_precedence_even_when_the_explicit_path_is_invalid() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let preferences = Preferences {
            last_project: Some(directory.path().canonicalize()?),
            ..Default::default()
        };
        let explicit = directory.path().join("not-present");
        assert_eq!(
            preferences.startup_project(Some(explicit.clone())),
            Some(explicit)
        );
        // Leave explicit relative paths intact for the usual scanner to resolve.
        let relative = PathBuf::from("examples/drone_demo_ws");
        assert_eq!(
            preferences.startup_project(Some(relative.clone())),
            Some(relative)
        );
        Ok(())
    }

    #[test]
    fn unavailable_saved_projects_open_an_empty_page_without_forgetting_the_path() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let file = directory.path().join("not-a-directory");
        std::fs::write(&file, "not a project")?;
        for saved in [
            directory.path().join("missing-project"),
            file,
            PathBuf::from("relative-project"),
        ] {
            let preferences = Preferences {
                last_project: Some(saved.clone()),
                ..Default::default()
            };
            assert_eq!(preferences.startup_project(None), None);
            assert_eq!(preferences.last_project.as_ref(), Some(&saved));
        }
        assert_eq!(Preferences::default().startup_project(None), None);
        Ok(())
    }

    #[test]
    fn older_preferences_without_a_recent_project_keep_the_empty_startup() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let path = directory.path().join("preferences.json");
        std::fs::write(&path, r#"{"version":1,"auto_mode":false}"#)?;
        let preferences = Preferences::load(&path)?;
        assert!(!preferences.auto_mode);
        assert!(preferences.last_project.is_none());
        assert_eq!(preferences.startup_project(None), None);
        Ok(())
    }

    #[test]
    fn missing_is_default_but_corrupt_or_future_preferences_are_not_overwritten() -> Result<()> {
        let directory = tempfile::tempdir()?;
        let path = directory.path().join("preferences.json");
        assert!(Preferences::load(&path)?.auto_mode);
        for contents in ["bad JSON", "{\"version\":2}"] {
            std::fs::write(&path, contents)?;
            assert!(Preferences::load(&path).is_err());
            assert_eq!(std::fs::read_to_string(&path)?, contents);
        }
        Ok(())
    }
}
