//! Read-only Cargo binary discovery. Never build or run a project to find its targets.

use anyhow::{Context as _, Result};
use std::{
    fs,
    path::{Path, PathBuf},
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CargoBinary {
    pub name: String,
    pub path: PathBuf,
}

pub fn binary_targets(root: &Path, manifest: &str) -> Result<Vec<CargoBinary>> {
    let document: toml::Value = toml::from_str(manifest)?;
    let Some(package) = document.get("package") else {
        return Ok(Vec::new());
    };
    let package_name = package
        .get("name")
        .and_then(toml::Value::as_str)
        .context("Cargo package has no name")?;
    let mut inferred = Vec::new();
    let main = root.join("src/main.rs");
    if main.is_file() {
        inferred.push(CargoBinary {
            name: package_name.into(),
            path: main.clone(),
        });
    }
    match fs::read_dir(root.join("src/bin")) {
        Ok(entries) => {
            for entry in entries {
                let path = entry?.path();
                let (name, source) =
                    if path.is_file() && path.extension().is_some_and(|ext| ext == "rs") {
                        (path.file_stem(), path.clone())
                    } else if path.is_dir() && path.join("main.rs").is_file() {
                        (path.file_name(), path.join("main.rs"))
                    } else {
                        continue;
                    };
                inferred.push(CargoBinary {
                    name: name
                        .and_then(|name| name.to_str())
                        .context("Cargo binary name is not UTF-8")?
                        .into(),
                    path: source,
                });
            }
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error).context("Cannot read Cargo binary directory"),
    }
    let entries = document.get("bin").and_then(toml::Value::as_array);
    let mut binaries = Vec::new();
    for entry in entries.into_iter().flatten() {
        let name = entry
            .get("name")
            .and_then(toml::Value::as_str)
            .context("Cargo binary has no name")?;
        let path = if let Some(path) = entry.get("path").and_then(toml::Value::as_str) {
            root.join(path)
        } else if let Some(target) = inferred.iter().find(|target| target.name == name) {
            target.path.clone()
        } else if entries.is_some_and(|entries| entries.len() == 1)
            && main.is_file()
            && package_edition(root, package)? == "2015"
        {
            main.clone()
        } else {
            anyhow::bail!("Cannot resolve source for Cargo binary {name}");
        };
        binaries.push(CargoBinary {
            name: name.into(),
            path,
        });
    }
    if autobins_enabled(root, &document)? {
        for target in inferred {
            let claimed = binaries.iter().any(|binary| {
                binary.name == target.name || same_source(&binary.path, &target.path)
            });
            if !claimed {
                binaries.push(target);
            }
        }
    }
    binaries.sort_by(|left, right| (&left.name, &left.path).cmp(&(&right.name, &right.path)));
    Ok(binaries)
}

pub fn automatic_binaries_enabled(root: &Path, manifest: &str) -> Result<bool> {
    autobins_enabled(root, &toml::from_str(manifest)?)
}

fn autobins_enabled(root: &Path, document: &toml::Value) -> Result<bool> {
    let package = document
        .get("package")
        .context("Cargo.toml does not declare a package")?;
    if let Some(enabled) = package.get("autobins").and_then(toml::Value::as_bool) {
        return Ok(enabled);
    }
    // Cargo's pre-2018 default disables discovery when explicit bins exist.
    let edition = package_edition(root, package)?;
    let entries = document.get("bin").and_then(toml::Value::as_array);
    Ok(edition != "2015" || entries.is_none_or(Vec::is_empty))
}

fn package_edition(root: &Path, package: &toml::Value) -> Result<String> {
    if let Some(edition) = package.get("edition").and_then(toml::Value::as_str) {
        Ok(edition.to_owned())
    } else if package
        .get("edition")
        .and_then(|edition| edition.get("workspace"))
        .and_then(toml::Value::as_bool)
        == Some(true)
    {
        inherited_edition(root, package)
    } else {
        Ok("2015".into())
    }
}

fn inherited_edition(root: &Path, package: &toml::Value) -> Result<String> {
    let explicit = package
        .get("workspace")
        .and_then(toml::Value::as_str)
        .map(|path| root.join(path));
    let directories = explicit
        .iter()
        .map(PathBuf::as_path)
        .chain(root.ancestors());
    for directory in directories {
        let path = directory.join("Cargo.toml");
        if !path.is_file() {
            continue;
        }
        let manifest: toml::Value = toml::from_str(&fs::read_to_string(path)?)?;
        if let Some(workspace) = manifest.get("workspace") {
            return workspace
                .get("package")
                .and_then(|package| package.get("edition"))
                .and_then(toml::Value::as_str)
                .map(str::to_owned)
                .context("Cargo workspace has no inherited edition");
        }
    }
    anyhow::bail!("Cannot locate the Cargo workspace's inherited edition")
}

pub fn binary_for_source<'a>(targets: &'a [CargoBinary], source: &Path) -> Option<&'a str> {
    targets
        .iter()
        .find(|target| same_source(&target.path, source))
        .map(|target| target.name.as_str())
}

fn same_source(left: &Path, right: &Path) -> bool {
    left == right
        || left
            .canonicalize()
            .ok()
            .zip(right.canonicalize().ok())
            .is_some_and(|(left, right)| left == right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(
        clippy::disallowed_methods,
        reason = "Synchronous Cargo metadata checks run only in unit tests"
    )]
    fn inherited_edition_keeps_automatic_binaries_enabled() -> Result<()> {
        let root = tempfile::tempdir()?;
        let member = root.path().join("member");
        fs::create_dir_all(member.join("src/bin"))?;
        fs::write(
            root.path().join("Cargo.toml"),
            "[workspace]\nmembers=['member']\nresolver='3'\n[workspace.package]\nedition='2024'\n",
        )?;
        let manifest = "[package]\nname='demo'\nversion='0.1.0'\nedition.workspace=true\n[[bin]]\nname='renamed'\npath='src/main.rs'\n";
        fs::write(member.join("Cargo.toml"), manifest)?;
        fs::write(member.join("src/main.rs"), "fn main() {}")?;
        fs::write(member.join("src/bin/extra.rs"), "fn main() {}")?;
        let names = binary_targets(&member, manifest)?
            .into_iter()
            .map(|target| target.name)
            .collect::<Vec<_>>();
        assert_eq!(names, ["extra", "renamed"]);
        let output = std::process::Command::new("cargo")
            .args([
                "metadata",
                "--offline",
                "--no-deps",
                "--format-version",
                "1",
            ])
            .current_dir(&member)
            .output()?;
        anyhow::ensure!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)?;
        let expected = metadata["packages"][0]["targets"]
            .as_array()
            .unwrap()
            .iter()
            .map(|target| target["name"].as_str().unwrap())
            .collect::<Vec<_>>();
        assert_eq!(names, expected);
        Ok(())
    }

    #[test]
    #[allow(
        clippy::disallowed_methods,
        reason = "Synchronous Cargo metadata checks run only in unit tests"
    )]
    fn resolves_explicit_and_implicit_targets_like_cargo_without_building() -> Result<()> {
        let root = tempfile::tempdir()?;
        fs::create_dir_all(root.path().join("src/bin/directory"))?;
        for name in ["main.rs", "bin/automatic.rs", "bin/directory/main.rs"] {
            fs::write(root.path().join("src").join(name), "fn main() {}")?;
        }
        for manifest in [
            "[package]\nname='example'\nversion='0.1.0'\n[workspace]\n",
            "[package]\nname='example'\nversion='0.1.0'\n[workspace]\n[[bin]]\nname='renamed'\n",
            "[package]\nname='example'\nversion='0.1.0'\n[workspace]\n[[bin]]\nname='renamed'\npath='src/./main.rs'\n",
            "[package]\nname='example'\nversion='0.1.0'\nedition='2024'\n[workspace]\n[[bin]]\nname='automatic'\n",
            "[package]\nname='example'\nversion='0.1.0'\nedition='2024'\n[workspace]\n[[bin]]\nname='directory'\n",
            "[package]\nname='example'\nversion='0.1.0'\nedition='2024'\n[workspace]\n[[bin]]\nname='renamed'\npath='src/./main.rs'\n",
            "[package]\nname='example'\nversion='0.1.0'\nautobins=false\n[workspace]\n[[bin]]\nname='renamed'\npath='src/./main.rs'\n",
        ] {
            fs::write(root.path().join("Cargo.toml"), manifest)?;
            let targets = binary_targets(root.path(), manifest)?;
            let output = std::process::Command::new("cargo")
                .args([
                    "metadata",
                    "--offline",
                    "--no-deps",
                    "--format-version",
                    "1",
                ])
                .current_dir(root.path())
                .output()?;
            anyhow::ensure!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)?;
            let mut expected = metadata["packages"][0]["targets"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|target| target["kind"] == serde_json::json!(["bin"]))
                .map(|target| target["name"].as_str().unwrap().to_owned())
                .collect::<Vec<_>>();
            expected.sort();
            assert_eq!(
                targets
                    .iter()
                    .map(|target| target.name.clone())
                    .collect::<Vec<_>>(),
                expected
            );
            assert_eq!(
                binary_for_source(&targets, &root.path().join("src/main.rs")),
                Some(if manifest.contains("renamed") {
                    "renamed"
                } else {
                    "example"
                })
            );
        }
        let invalid = "[package]\nname='example'\nversion='0.1.0'\nedition='2024'\n[workspace]\n[[bin]]\nname='renamed'\n";
        assert!(binary_targets(root.path(), invalid).is_err());
        Ok(())
    }
}
