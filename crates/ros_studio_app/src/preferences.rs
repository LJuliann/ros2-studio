use std::{
    collections::BTreeMap,
    io::Write as _,
    path::{Path, PathBuf},
};

use anyhow::{Context as _, Result, ensure};
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
    pub configurations: BTreeMap<String, RunTarget>,
}

impl Default for Preferences {
    fn default() -> Self {
        Self {
            version: 1,
            editor: ExternalEditor::default(),
            environment: RuntimePreference::default(),
            auto_mode: true,
            console_height: 240.0,
            configurations: BTreeMap::new(),
        }
    }
}

impl Preferences {
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
            ..Default::default()
        };
        preferences
            .configurations
            .insert("/project".into(), RunTarget::File("src/a file.py".into()));
        preferences.save(&path)?;
        preferences.console_height = 320.0;
        preferences.save(&path)?;
        let loaded = Preferences::load(&path)?;
        assert_eq!(loaded.editor, ExternalEditor::VsCode);
        assert!(!loaded.auto_mode);
        assert_eq!(loaded.console_height, 320.0);
        assert_eq!(
            loaded.configurations.get("/project"),
            preferences.configurations.get("/project")
        );
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
