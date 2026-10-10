//! Conservative, read-only discovery. Never invoke make to inspect a project:
//! even `make -n` can evaluate shell expressions or rebuild included files.

use std::{
    collections::{BTreeMap, BTreeSet},
    io::Read as _,
    path::Path,
};

use anyhow::{Context as _, Result, ensure};
use ros_studio_model::Project;
use serde::{Deserialize, Serialize};

use crate::runtime::{Execution, Operation};

mod packages;

const MAX_FILES: usize = 32;
const MAX_BYTES: u64 = 1024 * 1024;

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Deserialize, Serialize)]
pub struct MakeTarget {
    pub makefile: String,
    pub target: String,
}

#[derive(Clone, Debug, Default)]
pub struct MakeCatalog {
    pub targets: Vec<MakeTarget>,
    pub groups: Vec<MakeGroup>,
}

#[derive(Clone, Debug)]
pub struct MakeGroup {
    /// None means project-wide, shared, or no unambiguous package association.
    pub package: Option<String>,
    /// Indices into MakeCatalog::targets, kept stable when filtering the list.
    pub target_indices: Vec<usize>,
}

impl MakeGroup {
    pub fn label(&self) -> &str {
        self.package.as_deref().unwrap_or("Project / shared")
    }
}

impl MakeCatalog {
    pub fn filtered_groups(&self, query: &str) -> Vec<MakeGroup> {
        let query = query.trim().to_lowercase();
        self.groups
            .iter()
            .filter_map(|group| {
                let group_matches = group.label().to_lowercase().contains(&query);
                let target_indices = group
                    .target_indices
                    .iter()
                    .copied()
                    .filter(|index| {
                        self.targets
                            .get(*index)
                            .is_some_and(|target| group_matches || target.matches_filter(&query))
                    })
                    .collect::<Vec<_>>();
                (!target_indices.is_empty()).then(|| MakeGroup {
                    package: group.package.clone(),
                    target_indices,
                })
            })
            .collect()
    }
}

#[derive(Default)]
struct RuleMetadata {
    recipes: Vec<String>,
    files: BTreeSet<String>,
    prerequisites: BTreeSet<String>,
}

/// Use GNU make's root-file precedence and follow only literal local includes.
/// Computed targets/includes and pattern rules are intentionally not evaluated.
pub fn discover(root: &Path) -> Result<Vec<MakeTarget>> {
    let (name, rules) = read_rules(root)?;
    Ok(targets(&name, &rules))
}

pub fn catalog(root: &Path, project: &Project) -> Result<MakeCatalog> {
    let (name, rules) = read_rules(root)?;
    let targets = targets(&name, &rules);
    let groups = packages::groups(&targets, &rules, project);
    Ok(MakeCatalog { targets, groups })
}

fn targets(name: &str, rules: &BTreeMap<String, RuleMetadata>) -> Vec<MakeTarget> {
    rules
        .keys()
        .map(|target| MakeTarget {
            makefile: name.into(),
            target: target.clone(),
        })
        .collect()
}

fn read_rules(root: &Path) -> Result<(String, BTreeMap<String, RuleMetadata>)> {
    let root = root.canonicalize()?;
    let Some(name) = ["GNUmakefile", "makefile", "Makefile"]
        .into_iter()
        .find(|name| root.join(name).is_file())
    else {
        return Ok((String::new(), BTreeMap::new()));
    };
    let mut visited = BTreeSet::new();
    let mut rules = BTreeMap::new();
    read_targets(&root, name, &mut visited, &mut rules)?;
    Ok((name.into(), rules))
}

fn read_targets(
    root: &Path,
    relative: &str,
    visited: &mut BTreeSet<std::path::PathBuf>,
    rules: &mut BTreeMap<String, RuleMetadata>,
) -> Result<()> {
    let path = root
        .join(relative)
        .canonicalize()
        .with_context(|| format!("Cannot read make include {relative}"))?;
    ensure!(
        path.starts_with(root),
        "Makefile is outside the project: {relative}"
    );
    let source_file = path
        .strip_prefix(root)?
        .to_str()
        .context("Makefile path is not UTF-8")?
        .to_owned();
    if visited.contains(&path) {
        return Ok(());
    }
    ensure!(visited.len() < MAX_FILES, "Too many Makefile includes");
    visited.insert(path.clone());
    let mut contents = String::new();
    std::fs::File::open(path)?
        .take(MAX_BYTES + 1)
        .read_to_string(&mut contents)?;
    ensure!(
        contents.len() as u64 <= MAX_BYTES,
        "Makefile exceeds 1 MiB: {relative}"
    );
    let mut continued = String::new();
    let mut defining = false;
    let mut custom_recipes = false;
    let mut recipe_continues = false;
    let mut active_targets = Vec::<String>::new();
    for raw in contents.lines() {
        if raw.starts_with('\t') || recipe_continues {
            recipe_continues = raw.trim_end().ends_with('\\');
            if !defining && !custom_recipes {
                for target in &active_targets {
                    rules
                        .entry(target.clone())
                        .or_default()
                        .recipes
                        .push(raw.trim().to_owned());
                }
            }
            continue;
        }
        let line = raw.split('#').next().unwrap_or_default().trim_end();
        if let Some(part) = line.strip_suffix('\\') {
            continued.push_str(part);
            continued.push(' ');
            continue;
        }
        continued.push_str(line);
        let line = std::mem::take(&mut continued);
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        active_targets.clear();
        if line == "endef" {
            defining = false;
            continue;
        }
        if defining {
            continue;
        }
        if line.starts_with("define ") || line.starts_with("override define ") {
            defining = true;
            continue;
        }
        if line.starts_with(".RECIPEPREFIX") {
            custom_recipes = true;
        }
        // Alternative recipe prefixes cannot be distinguished safely without
        // evaluating make variables. Keep previously found targets only.
        if custom_recipes {
            continue;
        }
        if let Some((directive, rest)) = line.split_once(char::is_whitespace)
            && matches!(directive, "include" | "-include" | "sinclude")
        {
            for include in rest.split_whitespace().filter(|name| literal_include(name)) {
                if root.join(include).is_file() {
                    read_targets(root, include, visited, rules)?;
                }
            }
            continue;
        }
        let Some((left, right)) = line.split_once(':') else {
            continue;
        };
        if left.contains('=') || right.starts_with('=') {
            continue;
        }
        let names = if left.trim() == ".PHONY" { right } else { left };
        for name in names.split_whitespace().filter(|name| literal_target(name)) {
            rules.entry(name.into()).or_default();
            if left.trim() != ".PHONY" {
                active_targets.push(name.into());
            }
        }
        let body = right.trim_start_matches(':');
        let (prerequisites, inline) = body
            .split_once(';')
            .map_or((body, None), |(prerequisites, recipe)| {
                (prerequisites, Some(recipe))
            });
        for target in &active_targets {
            let rule = rules.entry(target.clone()).or_default();
            rule.files.insert(source_file.clone());
            rule.prerequisites.extend(
                prerequisites
                    .split_whitespace()
                    .filter(|name| literal_target(name))
                    .map(str::to_owned),
            );
            if let Some(recipe) = inline {
                rule.recipes.push(recipe.into());
            }
        }
    }
    Ok(())
}

fn literal_include(name: &str) -> bool {
    !name.is_empty() && !name.contains(['$', '*', '?', '[', '\\']) && !Path::new(name).is_absolute()
}

fn literal_target(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with(['.', '-', '/'])
        && name
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"_./-".contains(&byte))
}

impl MakeTarget {
    pub fn matches_filter(&self, query: &str) -> bool {
        let query = query.trim().to_lowercase();
        format!("make {}", self.target)
            .to_lowercase()
            .contains(&query)
            || self.makefile.to_lowercase().contains(&query)
    }

    pub fn execution(&self, root: &Path, operation: Operation) -> Result<Execution> {
        ensure!(cfg!(unix), "Make targets currently require a Unix host");
        ensure!(
            discover(root)?.contains(self),
            "Make target no longer exists; refresh and choose another"
        );
        Ok(Execution {
            operation,
            label: format!(
                "{} make {} · {} (host)",
                operation.label(),
                self.target,
                self.makefile
            ),
            command: vec![
                "make".into(),
                "--file".into(),
                format!("./{}", self.makefile),
                "--".into(),
                self.target.clone(),
            ],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filters_make_commands_by_target_or_file_without_hiding_the_unfiltered_list() {
        let target = MakeTarget {
            makefile: "Makefile".into(),
            target: "ros-build".into(),
        };
        for query in ["", "  ", "ROS-BUILD", " build ", "makefile"] {
            assert!(target.matches_filter(query));
        }
        assert!(!target.matches_filter("camera-run"));
    }

    #[test]
    fn finds_image_by_its_displayed_command_label() {
        let target = MakeTarget {
            makefile: "Makefile".into(),
            target: "image".into(),
        };
        for query in ["image", "make image", "  MAKE IMAGE  ", "Makefile", ""] {
            assert!(
                target.matches_filter(query),
                "Query did not match: {query:?}"
            );
        }
        assert!(!target.matches_filter("make ros-build"));
    }

    #[test]
    fn discovers_launch_and_build_targets_without_evaluating_make() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            concat!(
                ".PHONY: ros-build ros-run \\\n ros-test\n",
                "ROS_SETUP = source /opt/ros/setup.bash && \\\n echo macro:not-a-target\n",
                "VALUE := $(shell touch should-not-exist)\ndefine RULE\nnot-a-target:\nendef\n",
                "ros-build:\n\tdocker compose run ros2 colcon build\n",
                "ros-run:\n\tdocker compose run ros2 ros2 launch drone_bringup drone.launch.py\n",
                "lidar-run camera-run: prerequisites\n\techo recipe:not-a-target \\\ncontinued:not-a-rule\n",
                "%.o: %.c\n.hidden: \n",
            ),
        )?;
        let targets = discover(root.path())?;
        assert_eq!(
            targets
                .iter()
                .map(|target| target.target.as_str())
                .collect::<Vec<_>>(),
            [
                "camera-run",
                "lidar-run",
                "ros-build",
                "ros-run",
                "ros-test"
            ]
        );
        assert!(!root.path().join("should-not-exist").exists());
        #[cfg(unix)]
        {
            let execution = targets[3].execution(root.path(), Operation::Run)?;
            assert_eq!(
                execution.command,
                ["make", "--file", "./Makefile", "--", "ros-run"]
            );
        }
        #[cfg(not(unix))]
        assert!(
            targets[3]
                .execution(root.path(), Operation::Run)
                .err()
                .context("Unix Make execution must be rejected")?
                .to_string()
                .contains("Unix host")
        );
        Ok(())
    }

    #[test]
    fn follows_literal_includes_with_cycles_and_root_file_precedence() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::create_dir(root.path().join("tasks"))?;
        std::fs::write(
            root.path().join("GNUmakefile"),
            "include tasks/ros.mk\n-include missing.mk $(COMPUTED)\nall:\n",
        )?;
        std::fs::write(
            root.path().join("tasks/ros.mk"),
            "include GNUmakefile\nros-run:\nros-build:\n",
        )?;
        std::fs::write(root.path().join("Makefile"), "ignored:\n")?;
        let targets = discover(root.path())?;
        assert_eq!(targets.len(), 3);
        assert!(
            targets
                .iter()
                .all(|target| target.makefile == "GNUmakefile")
        );
        std::fs::write(root.path().join("tasks/ros.mk"), "changed:\n")?;
        assert!(targets[2].execution(root.path(), Operation::Run).is_err());
        Ok(())
    }

    #[test]
    fn missing_makefiles_are_valid_and_invalid_targets_cannot_be_executed() -> Result<()> {
        let root = tempfile::tempdir()?;
        assert!(discover(root.path())?.is_empty());
        std::fs::write(
            root.path().join("Makefile"),
            "-f: \n$(GENERATED):\nthing;touch:\nnormal:\n",
        )?;
        assert_eq!(discover(root.path())?.len(), 1);
        assert!(
            MakeTarget {
                makefile: "../Makefile".into(),
                target: "normal".into()
            }
            .execution(root.path(), Operation::Run)
            .is_err()
        );
        Ok(())
    }

    #[cfg(unix)]
    #[test]
    fn rejects_outside_symlinks_and_oversized_inputs() -> Result<()> {
        let root = tempfile::tempdir()?;
        let outside = tempfile::tempdir()?;
        std::fs::write(outside.path().join("rules.mk"), "external:\n")?;
        std::os::unix::fs::symlink(
            outside.path().join("rules.mk"),
            root.path().join("rules.mk"),
        )?;
        std::fs::write(root.path().join("Makefile"), "include rules.mk\n")?;
        assert!(discover(root.path()).is_err());
        std::fs::write(
            root.path().join("Makefile"),
            "x".repeat(MAX_BYTES as usize + 1),
        )?;
        assert!(discover(root.path()).is_err());
        Ok(())
    }
}
