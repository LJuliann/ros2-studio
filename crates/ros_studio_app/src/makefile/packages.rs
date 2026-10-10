//! Package labels are conservative hints, never an interpreter for make/shell.
//! Prefer explicit run/launch/build commands over message-type references and
//! infer topic/service ownership only from the project's static graph.

use super::{MakeGroup, MakeTarget, RuleMetadata};
use ros_studio_model::{EndpointKind, Package, Project};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

pub(super) fn groups(
    targets: &[MakeTarget],
    rules: &BTreeMap<String, RuleMetadata>,
    project: &Project,
) -> Vec<MakeGroup> {
    let mut owners = rules
        .iter()
        .map(|(name, rule)| (name.clone(), rule_packages(rule, project)))
        .collect::<BTreeMap<_, _>>();
    // Unclassified aliases inherit an association only when every prerequisite
    // has a known association. Cycles/unknown/global prerequisites stay shared.
    for _ in 0..rules.len() {
        let updates = rules
            .iter()
            .filter_map(|(name, rule)| {
                if !owners[name].is_empty() || rule.prerequisites.is_empty() {
                    return None;
                }
                let mut packages = BTreeSet::new();
                for prerequisite in &rule.prerequisites {
                    let related = owners.get(prerequisite)?;
                    if related.is_empty() {
                        return None;
                    }
                    packages.extend(related.iter().cloned());
                }
                Some((name.clone(), packages))
            })
            .collect::<Vec<_>>();
        if updates.is_empty() {
            break;
        }
        owners.extend(updates);
    }
    let mut grouped = BTreeMap::<Option<String>, Vec<usize>>::new();
    for (index, target) in targets.iter().enumerate() {
        let related = &owners[&target.target];
        let package = if related.len() == 1 {
            related.first().cloned()
        } else {
            None
        };
        grouped.entry(package).or_default().push(index);
    }
    grouped
        .into_iter()
        .map(|(package, target_indices)| MakeGroup {
            package,
            target_indices,
        })
        .collect()
}

fn rule_packages(rule: &RuleMetadata, project: &Project) -> BTreeSet<String> {
    let recipe = without_variables(&rule.recipes.join(" "));
    // Splitting literal words also sees commands inside Docker's bash -lc quote.
    // No variable expansion, command execution, or shell evaluation takes place.
    let tokens = recipe
        .split(|character: char| character.is_whitespace() || "'\";&|(){}<>\\".contains(character))
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();
    let mut explicit = BTreeSet::new();
    for command in tokens.windows(3) {
        if command[0].trim_start_matches(['@', '-', '+']) == "ros2"
            && matches!(command[1], "run" | "launch")
        {
            explicit.extend(
                project
                    .packages
                    .iter()
                    .filter(|package| package.name == command[2])
                    .map(|package| package.name.clone()),
            );
        }
    }
    for (index, token) in tokens.iter().enumerate() {
        if matches!(*token, "--packages-select" | "--packages-up-to") {
            for name in tokens[index + 1..]
                .iter()
                .take_while(|name| !name.starts_with('-'))
            {
                explicit.extend(
                    project
                        .packages
                        .iter()
                        .filter(|package| package.name == **name)
                        .map(|package| package.name.clone()),
                );
            }
        }
        if token.ends_with("Cargo.toml") {
            explicit.extend(path_packages(token, project));
        }
    }
    if !explicit.is_empty() {
        return explicit;
    }
    let endpoints = endpoint_packages(&tokens, project);
    if !endpoints.is_empty() {
        return endpoints;
    }
    let literal = project
        .packages
        .iter()
        .filter(|package| {
            tokens.iter().any(|token| {
                *token == package.name
                    || token.starts_with(&format!("{}/", package.name))
                    || path_reference(token, package)
            })
        })
        .map(|package| package.name.clone())
        .collect::<BTreeSet<_>>();
    if !literal.is_empty() {
        return literal;
    }
    if !rule.prerequisites.is_empty() {
        return BTreeSet::new();
    }
    // Rules in a package-owned included .mk file belong to that package unless
    // their recipes explicitly reference another package or several packages.
    let mut file_owners = BTreeSet::new();
    for file in &rule.files {
        let candidates = project
            .packages
            .iter()
            .filter(|package| Path::new(file).starts_with(Path::new(&package.path)))
            .collect::<Vec<_>>();
        let nearest = candidates
            .iter()
            .map(|package| Path::new(&package.path).components().count())
            .max();
        file_owners.extend(
            candidates
                .into_iter()
                .filter(|package| Some(Path::new(&package.path).components().count()) == nearest)
                .map(|package| package.name.clone()),
        );
    }
    file_owners
}

fn without_variables(recipe: &str) -> String {
    let mut literal = String::new();
    let mut characters = recipe.chars().peekable();
    while let Some(character) = characters.next() {
        if character == '$'
            && let Some(opening @ ('(' | '{')) = characters.peek().copied()
        {
            characters.next();
            let closing = if opening == '(' { ')' } else { '}' };
            let mut depth = 1;
            for next in characters.by_ref() {
                if next == opening {
                    depth += 1;
                }
                if next == closing {
                    depth -= 1;
                }
                if depth == 0 {
                    break;
                }
            }
            // Keep an argument placeholder, so removing $(PACKAGE) cannot shift
            // another word into ros2 run's package argument.
            literal.push('?');
        } else {
            literal.push(character);
        }
    }
    literal
}

fn path_packages(token: &str, project: &Project) -> BTreeSet<String> {
    let candidates = project
        .packages
        .iter()
        .filter(|package| path_reference(token, package))
        .collect::<Vec<_>>();
    let nearest = candidates
        .iter()
        .map(|package| Path::new(&package.path).components().count())
        .max();
    candidates
        .into_iter()
        .filter(|package| Some(Path::new(&package.path).components().count()) == nearest)
        .map(|package| package.name.clone())
        .collect()
}

fn path_reference(token: &str, package: &Package) -> bool {
    if package.path.is_empty() || package.path == "." {
        return matches!(token.trim_start_matches("./"), "Cargo.toml");
    }
    let path = package.path.trim_end_matches('/');
    let token = token
        .split_once('=')
        .map_or(token, |(_, value)| value)
        .trim_start_matches("./");
    token == path || token.starts_with(&format!("{path}/")) || token.contains(&format!("/{path}/"))
}

fn endpoint_packages(tokens: &[&str], project: &Project) -> BTreeSet<String> {
    let mut related = BTreeSet::new();
    for command in tokens.windows(3) {
        if command[0].trim_start_matches(['@', '-', '+']) != "ros2" {
            continue;
        }
        let kind = match (command[1], command[2]) {
            ("topic", "echo" | "hz" | "info" | "bw") => EndpointKind::Publisher,
            ("topic", "pub") => EndpointKind::Subscription,
            ("service", "call") => EndpointKind::ServiceServer,
            ("action", "send_goal") => EndpointKind::ActionServer,
            _ => continue,
        };
        for node in &project.nodes {
            if node.endpoints.iter().any(|endpoint| {
                endpoint.kind == kind
                    && tokens.iter().any(|token| {
                        token.trim_start_matches('/') == endpoint.name.trim_start_matches('/')
                    })
            }) && let Some(package) = project
                .packages
                .iter()
                .find(|package| package.id == node.package_id)
            {
                related.insert(package.name.clone());
            }
        }
    }
    related
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::makefile::{MakeCatalog, catalog, discover};
    use anyhow::{Context as _, Result};
    use ros_studio_model::{Endpoint, EntityId, Node};

    fn project(root: &Path) -> Project {
        let names = [
            "drone_camera",
            "drone_image_detection",
            "drone_flight",
            "water_gun_zeroing",
            "drone_interfaces",
            "drone_bringup",
        ];
        let packages = names
            .into_iter()
            .map(|name| Package {
                id: EntityId::new(format!("package:{name}")),
                name: name.into(),
                path: format!("ros2/{name}"),
            })
            .collect::<Vec<_>>();
        let mut project = Project {
            id: EntityId::new("project:test"),
            name: "test".into(),
            root_path: root.to_string_lossy().into_owned(),
            packages,
            nodes: Vec::new(),
        };
        for (package, kind, name) in [
            ("drone_camera", EndpointKind::Publisher, "/camera/image_raw"),
            (
                "drone_image_detection",
                EndpointKind::Subscription,
                "/camera/image_raw",
            ),
            ("drone_flight", EndpointKind::Publisher, "flight/status"),
            (
                "drone_flight",
                EndpointKind::ServiceServer,
                "/flight/set_armed",
            ),
            (
                "drone_flight",
                EndpointKind::ActionServer,
                "/flight/navigate_to",
            ),
            (
                "water_gun_zeroing",
                EndpointKind::ServiceServer,
                "/water_gun/shoot",
            ),
            (
                "water_gun_zeroing",
                EndpointKind::Subscription,
                "/water_gun/aim_angles",
            ),
        ] {
            project.nodes.push(Node {
                id: EntityId::new(format!("node:{package}:{name}")),
                logical_name: package.into(),
                package_id: EntityId::new(format!("package:{package}")),
                executable: package.into(),
                source_locations: Vec::new(),
                endpoints: vec![Endpoint {
                    id: EntityId::new(format!("endpoint:{package}:{name}")),
                    kind,
                    name: name.into(),
                    type_name: String::new(),
                    source_location: None,
                    confidence: Default::default(),
                    evidence: Vec::new(),
                }],
                evidence: Vec::new(),
                runtime_state: Default::default(),
            });
        }
        project
    }

    fn association<'a>(catalog: &'a MakeCatalog, target: &str) -> Result<Option<&'a str>> {
        let index = catalog
            .targets
            .iter()
            .position(|item| item.target == target)
            .context("Target was not discovered")?;
        let group = catalog
            .groups
            .iter()
            .find(|group| group.target_indices.contains(&index))
            .context("Target has no group")?;
        Ok(group.package.as_deref())
    }

    #[test]
    fn categorizes_real_recipe_references_not_target_name_prefixes() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            concat!(
                "ROS_SETUP = source install/drone_interfaces/share/setup.bash\n",
                "camera-run-1:\n\t@docker compose run ros2 bash -lc '$(ROS_SETUP) && ros2 run drone_camera camera_node'\n",
                "camera-run-2:\n\t@ros2 run drone_image_detection yolo_node\n",
                "zeroing-run:\n\t@ros2 run water_gun_zeroing zeroing\n",
                "ros-run:\n\t@ros2 launch drone_bringup drone.launch.py\n",
                "px4-probe:\n\t@cargo run --manifest-path=./ros2/drone_flight/Cargo.toml\n",
                "ros-build:\n\t@colcon build --packages-select drone_camera drone_flight water_gun_zeroing\n",
                "ros-test:\n\t@cargo test --manifest-path ros2/drone_flight/Cargo.toml && cargo test --manifest-path ros2/water_gun_zeroing/Cargo.toml\n",
                "px4-start:\n\t@docker compose -f compose.px4.yaml up -d\n",
                "unknown:\n\t@echo $(drone_camera) $(shell touch should-not-exist)\n",
            ),
        )?;
        let catalog = catalog(root.path(), &project(root.path()))?;
        for (target, package) in [
            ("camera-run-1", "drone_camera"),
            ("camera-run-2", "drone_image_detection"),
            ("zeroing-run", "water_gun_zeroing"),
            ("ros-run", "drone_bringup"),
            ("px4-probe", "drone_flight"),
        ] {
            assert_eq!(association(&catalog, target)?, Some(package));
        }
        for target in ["ros-build", "ros-test", "px4-start", "unknown"] {
            assert_eq!(association(&catalog, target)?, None);
        }
        assert!(!root.path().join("should-not-exist").exists());
        assert_eq!(catalog.targets, discover(root.path())?);
        let mut indices = catalog
            .groups
            .iter()
            .flat_map(|group| group.target_indices.iter().copied())
            .collect::<Vec<_>>();
        indices.sort_unstable();
        assert_eq!(indices, (0..catalog.targets.len()).collect::<Vec<_>>());
        Ok(())
    }

    #[test]
    fn associates_topics_services_and_actions_with_owners_before_message_packages() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            concat!(
                "camera-topic:\n\t@ros2 topic hz /camera/image_raw\n",
                "flight-topic:\n\t@ros2 topic echo /flight/status\n",
                "ros-arm:\n\t@ros2 service call /flight/set_armed drone_interfaces/srv/SetArmed '{}'\n",
                "navigate:\n\t@ros2 action send_goal /flight/navigate_to drone_interfaces/action/NavigateTo '{}'\n",
                "aim:\n\t@ros2 topic pub --once /water_gun/aim_angles drone_interfaces/msg/AimSolution '{}'\n",
                "shoot:\n\t@ros2 service call /water_gun/shoot drone_interfaces/srv/Shoot '{}'\n",
                "interface:\n\t@ros2 interface show drone_interfaces/msg/FlightStatus\n",
            ),
        )?;
        let mut project = project(root.path());
        let grouped = catalog(root.path(), &project)?;
        for (target, package) in [
            ("camera-topic", "drone_camera"),
            ("flight-topic", "drone_flight"),
            ("ros-arm", "drone_flight"),
            ("navigate", "drone_flight"),
            ("aim", "water_gun_zeroing"),
            ("shoot", "water_gun_zeroing"),
            ("interface", "drone_interfaces"),
        ] {
            assert_eq!(association(&grouped, target)?, Some(package));
        }
        let mut other_publisher = project.nodes[0].clone();
        other_publisher.package_id = EntityId::new("package:drone_image_detection");
        project.nodes.push(other_publisher);
        assert_eq!(
            association(&catalog(root.path(), &project)?, "camera-topic")?,
            None
        );
        Ok(())
    }

    #[test]
    fn groups_included_rules_and_aliases_but_keeps_ambiguous_cycles_shared() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::create_dir_all(root.path().join("ros2/drone_camera"))?;
        std::fs::write(
            root.path().join("Makefile"),
            concat!(
                ".PHONY: local-check camera-alias\ninclude ./ros2/drone_camera/tasks.mk\n",
                "camera-alias: local-check\nsecond-alias:: camera-alias\n",
                "detector:; ros2 run drone_image_detection yolo\nall: second-alias detector\n",
                "image:; docker compose build\nshared: camera-alias image\n",
                "cycle-a: cycle-b\ncycle-b: cycle-a\n",
            ),
        )?;
        std::fs::write(
            root.path().join("ros2/drone_camera/tasks.mk"),
            "local-check:\n\t@echo checking\n",
        )?;
        let mut project = project(root.path());
        // The closest owning package wins over a parent package directory.
        project.packages.push(Package {
            id: EntityId::new("package:outer"),
            name: "outer".into(),
            path: "ros2".into(),
        });
        let grouped = catalog(root.path(), &project)?;
        for target in ["local-check", "camera-alias", "second-alias"] {
            assert_eq!(association(&grouped, target)?, Some("drone_camera"));
        }
        assert_eq!(
            association(&grouped, "detector")?,
            Some("drone_image_detection")
        );
        for target in ["all", "shared", "cycle-a", "cycle-b"] {
            assert_eq!(association(&grouped, target)?, None);
        }
        Ok(())
    }

    #[test]
    fn searching_by_package_preserves_the_correct_target_indices() -> Result<()> {
        let root = tempfile::tempdir()?;
        std::fs::write(
            root.path().join("Makefile"),
            "camera-run-1:; ros2 run drone_camera camera\ncamera-run-2:; ros2 run drone_image_detection yolo\nimage:; docker compose build\n",
        )?;
        let grouped = catalog(root.path(), &project(root.path()))?;
        let matches = grouped.filtered_groups(" DRONE_IMAGE_DETECTION ");
        assert_eq!(matches.len(), 1);
        assert_eq!(
            grouped.targets[matches[0].target_indices[0]].target,
            "camera-run-2"
        );
        assert_eq!(
            grouped
                .filtered_groups("camera-run")
                .iter()
                .map(|group| group.target_indices.len())
                .sum::<usize>(),
            2
        );
        assert_eq!(grouped.filtered_groups("project").len(), 1);
        assert!(grouped.filtered_groups("missing").is_empty());
        assert_eq!(
            grouped
                .filtered_groups("")
                .iter()
                .map(|group| group.target_indices.len())
                .sum::<usize>(),
            grouped.targets.len()
        );
        Ok(())
    }
}
