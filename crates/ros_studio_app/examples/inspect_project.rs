//! Inspect a real ROS project without a display, Docker, or a ROS installation.
use std::path::PathBuf;

use anyhow::{Context as _, Result, ensure};
use ros_studio_app::{
    GraphSnapshot,
    runtime::{Operation, node_execution},
    wizard::rust_packages,
};

fn main() -> Result<()> {
    let mut arguments = std::env::args_os().skip(1);
    let root = PathBuf::from(
        arguments
            .next()
            .context("Usage: inspect_project PROJECT_FOLDER")?,
    );
    ensure!(arguments.next().is_none(), "Choose only one project folder");
    let graph = GraphSnapshot::scan(&root)?;
    println!("Project: {}", root.display());
    println!(
        "Packages: {} | Nodes: {} | Links: {} | Rust generator targets: {}",
        graph.project.packages.len(),
        graph.project.nodes.len(),
        graph.edges.len(),
        rust_packages(&graph.project).count()
    );
    for node in &graph.project.nodes {
        let build = node_execution(&graph.project, node.id.as_str(), Operation::Build)?;
        println!("{}: {:?}", node.logical_name, build.command);
    }
    Ok(())
}
