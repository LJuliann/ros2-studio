#![forbid(unsafe_code)]

pub mod health;
mod layout;
pub mod local_process;
pub mod makefile;
pub mod preferences;
pub mod runtime;
pub mod runtime_client;
pub mod runtime_environment;
pub mod text_buffer;
pub mod wizard;

use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::OsString,
    path::{Path, PathBuf},
};

use anyhow::{Context as _, Result, bail, ensure};
use layout::graph_positions;
use ros_studio_model::{
    EndpointKind, Project, SourceLocation, design_live::canonical_ros_type_name,
};

pub const NODE_WIDTH: f32 = 260.0;
pub const NODE_HEIGHT: f32 = 180.0;

/// Keep the normal desktop size, but leave space for window decorations on
/// small/scaled screens. Inputs are the display's visible logical dimensions.
pub fn initial_window_size(available_width: f32, available_height: f32) -> (f32, f32) {
    (
        (available_width - 48.0).clamp(1.0, 1280.0),
        (available_height - 80.0).clamp(1.0, 800.0),
    )
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct GraphEdge {
    pub publisher_id: String,
    pub subscriber_id: String,
    pub topic: String,
    pub type_name: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodePosition {
    pub node_id: String,
    pub x: f32,
    pub y: f32,
}

pub struct GraphSnapshot {
    pub root: PathBuf,
    pub project: Project,
    pub edges: Vec<GraphEdge>,
    pub positions: Vec<NodePosition>,
}

impl GraphSnapshot {
    pub fn from_project(root: PathBuf, project: Project) -> Self {
        let edges = graph_edges(&project);
        let positions = graph_positions(&project, &edges);
        Self {
            root,
            project,
            edges,
            positions,
        }
    }
    pub fn scan(root: &Path) -> Result<Self> {
        let root = root
            .canonicalize()
            .with_context(|| format!("Cannot open project {}", root.display()))?;
        ensure!(root.is_dir(), "Choose a project folder, not a file");
        let name = root
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("ROS project");
        let project = ros_studio_scan::scan_project(&root, name)?;
        let edges = graph_edges(&project);
        let positions = graph_positions(&project, &edges);
        Ok(Self {
            root,
            project,
            edges,
            positions,
        })
    }

    pub fn topic_count(&self) -> usize {
        self.project
            .nodes
            .iter()
            .flat_map(|node| &node.endpoints)
            .filter(|endpoint| {
                matches!(
                    endpoint.kind,
                    EndpointKind::Publisher | EndpointKind::Subscription
                )
            })
            .map(|endpoint| &endpoint.name)
            .collect::<BTreeSet<_>>()
            .len()
    }

    pub fn preserve_positions(&mut self, previous: &Self) {
        if self.root != previous.root {
            return;
        }
        let previous_positions = previous
            .positions
            .iter()
            .map(|position| (position.node_id.as_str(), position))
            .collect::<BTreeMap<_, _>>();
        for position in &mut self.positions {
            if let Some(previous) = previous_positions.get(position.node_id.as_str()) {
                position.x = previous.x;
                position.y = previous.y;
            }
        }
        // New nodes must not occupy a slot already used by a node the user moved.
        let mut occupied = self
            .positions
            .iter()
            .filter(|position| previous_positions.contains_key(position.node_id.as_str()))
            .map(|position| (position.x, position.y))
            .collect::<Vec<_>>();
        for position in &mut self.positions {
            if !previous_positions.contains_key(position.node_id.as_str()) {
                while occupied.iter().any(|&(x, y)| {
                    (position.x - x).abs() < NODE_WIDTH + 24.0
                        && (position.y - y).abs() < NODE_HEIGHT + 24.0
                }) {
                    position.y += NODE_HEIGHT + 100.0;
                }
                occupied.push((position.x, position.y));
            }
        }
    }
}

pub fn graph_edges(project: &Project) -> Vec<GraphEdge> {
    let mut subscriptions = BTreeMap::<(String, String), BTreeSet<String>>::new();
    for node in &project.nodes {
        for endpoint in node
            .endpoints
            .iter()
            .filter(|endpoint| endpoint.kind == EndpointKind::Subscription)
        {
            subscriptions
                .entry((
                    endpoint.name.clone(),
                    canonical_ros_type_name(&endpoint.type_name),
                ))
                .or_default()
                .insert(node.id.as_str().to_owned());
        }
    }
    let mut edges = BTreeSet::new();
    for node in &project.nodes {
        for endpoint in node
            .endpoints
            .iter()
            .filter(|endpoint| endpoint.kind == EndpointKind::Publisher)
        {
            let type_name = canonical_ros_type_name(&endpoint.type_name);
            if let Some(subscribers) =
                subscriptions.get(&(endpoint.name.clone(), type_name.clone()))
            {
                for subscriber_id in subscribers {
                    edges.insert(GraphEdge {
                        publisher_id: node.id.as_str().to_owned(),
                        subscriber_id: subscriber_id.clone(),
                        topic: endpoint.name.clone(),
                        type_name: type_name.clone(),
                    });
                }
            }
        }
    }
    edges.into_iter().collect()
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viewport {
    pub x: f32,
    pub y: f32,
    pub scale: f32,
}

impl Default for Viewport {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            scale: 1.0,
        }
    }
}

impl Viewport {
    /// Visible grid dots, computed lazily without allocating or tessellating a path.
    pub fn grid_points(self, width: f32, height: f32) -> impl Iterator<Item = (f32, f32)> {
        let valid = [self.x, self.y, self.scale, width, height]
            .into_iter()
            .all(f32::is_finite)
            && width > 0.0
            && height > 0.0;
        let spacing = (32.0 * self.scale).max(16.0);
        let start_x = self.x.rem_euclid(spacing);
        let start_y = self.y.rem_euclid(spacing);
        let columns = if valid {
            ((width - start_x).max(0.0) / spacing).ceil() as usize
        } else {
            0
        };
        let rows = if valid {
            ((height - start_y).max(0.0) / spacing).ceil() as usize
        } else {
            0
        };
        (0..rows).flat_map(move |row| {
            (0..columns).map(move |column| {
                (
                    start_x + column as f32 * spacing,
                    start_y + row as f32 * spacing,
                )
            })
        })
    }

    pub fn screen_point(self, x: f32, y: f32) -> (f32, f32) {
        (self.x + x * self.scale, self.y + y * self.scale)
    }

    pub fn zoom_at(&mut self, scale: f32, x: f32, y: f32) {
        let scale = scale.clamp(0.25, 2.0);
        let graph_x = (x - self.x) / self.scale;
        let graph_y = (y - self.y) / self.scale;
        self.x = x - graph_x * scale;
        self.y = y - graph_y * scale;
        self.scale = scale;
    }

    pub fn fit(&mut self, positions: &[NodePosition], width: f32, height: f32) {
        if positions.is_empty() || width <= 0.0 || height <= 0.0 {
            return;
        }
        let left = positions
            .iter()
            .map(|position| position.x)
            .fold(f32::INFINITY, f32::min);
        let top = positions
            .iter()
            .map(|position| position.y)
            .fold(f32::INFINITY, f32::min);
        let right = positions
            .iter()
            .map(|position| position.x + NODE_WIDTH)
            .fold(f32::NEG_INFINITY, f32::max);
        let bottom = positions
            .iter()
            .map(|position| position.y + NODE_HEIGHT)
            .fold(f32::NEG_INFINITY, f32::max);
        self.scale = ((width - 80.0).max(1.0) / (right - left))
            .min((height - 80.0).max(1.0) / (bottom - top))
            .clamp(0.25, 1.0);
        self.x = width / 2.0 - (left + right) * self.scale / 2.0;
        self.y = height / 2.0 - (top + bottom) * self.scale / 2.0;
    }

    /// Opening a project favors legible cards over an overview of every family.
    /// Dimensions are logical canvas pixels: GPUI already accounts for monitor
    /// DPI, toolbars and sidebars. Do not multiply them by the display scale.
    pub fn open_project(&mut self, positions: &[NodePosition], width: f32, height: f32) {
        if positions.is_empty()
            || !width.is_finite()
            || !height.is_finite()
            || width <= 0.0
            || height <= 0.0
            || positions
                .iter()
                .any(|p| !p.x.is_finite() || !p.y.is_finite())
        {
            return;
        }
        let left = positions.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
        let top = positions.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
        let right = positions
            .iter()
            .map(|p| p.x + NODE_WIDTH)
            .fold(f32::NEG_INFINITY, f32::max);
        let bottom = positions
            .iter()
            .map(|p| p.y + NODE_HEIGHT)
            .fold(f32::NEG_INFINITY, f32::max);
        if ![left, top, right, bottom].into_iter().all(f32::is_finite) {
            return;
        }

        let padding = 24.0;
        let available_width = (width - padding * 2.0).max(1.0);
        let available_height = (height - padding * 2.0).max(1.0);
        let preferred = (height / 720.0).sqrt().clamp(1.0, 1.15);
        // Try to show the full width, but never shrink a large project into tiny
        // cards just because it has many columns or disconnected families.
        self.scale = (available_width / (right - left))
            .clamp(0.9, preferred)
            .min(available_width / NODE_WIDTH)
            .min(available_height / NODE_HEIGHT)
            .clamp(0.25, 1.15);
        // Center an axis if it fits. Otherwise start at the first row/column,
        // leaving the rest accessible by panning instead of clipping the roots.
        self.x = if (right - left) * self.scale <= available_width {
            width / 2.0 - (left + right) * self.scale / 2.0
        } else {
            padding - left * self.scale
        };
        self.y = if (bottom - top) * self.scale <= available_height {
            height / 2.0 - (top + bottom) * self.scale / 2.0
        } else {
            padding - top * self.scale
        };
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ExternalEditor {
    #[default]
    System,
    VsCode,
    VsCodium,
    Zed,
    Clion,
}

impl ExternalEditor {
    pub const ALL: [Self; 5] = [
        Self::System,
        Self::VsCode,
        Self::VsCodium,
        Self::Zed,
        Self::Clion,
    ];

    pub fn parse(name: &str) -> Result<Self> {
        match name {
            "system" => Ok(Self::System),
            "code" | "vscode" => Ok(Self::VsCode),
            "codium" | "vscodium" => Ok(Self::VsCodium),
            "zed" => Ok(Self::Zed),
            "clion" => Ok(Self::Clion),
            _ => bail!("Unknown editor {name:?}. Choose system, code, codium, zed, or clion"),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::System => "System default",
            Self::VsCode => "VS Code",
            Self::VsCodium => "VSCodium",
            Self::Zed => "Zed",
            Self::Clion => "CLion",
        }
    }

    pub fn source_command(self, root: &Path, source: &SourceLocation) -> Result<EditorCommand> {
        let root = root
            .canonicalize()
            .context("Cannot locate project folder")?;
        ensure!(
            !Path::new(&source.path).is_absolute(),
            "Source path must be project-relative"
        );
        let path = root
            .join(&source.path)
            .canonicalize()
            .with_context(|| format!("Cannot open source {}", source.path))?;
        ensure!(
            path.starts_with(&root),
            "Source is outside the project folder"
        );
        ensure!(path.is_file(), "Source must be a file");
        let line = source.line.saturating_add(1).to_string();
        // Scanner columns are UTF-8 byte offsets, while editors disagree about character units.
        // Opening the exact line at column 1 avoids misleading positions for non-ASCII text.
        let mut target = path.as_os_str().to_owned();
        target.push(format!(":{line}:1"));
        let (program, arguments) = match self {
            Self::VsCode | Self::VsCodium => (
                match (self, cfg!(target_os = "windows")) {
                    (Self::VsCode, true) => "code.cmd",
                    (Self::VsCode, false) => "code",
                    (_, true) => "codium.cmd",
                    (_, false) => "codium",
                }
                .into(),
                vec![
                    "--reuse-window".into(),
                    root.as_os_str().to_owned(),
                    "--goto".into(),
                    target,
                ],
            ),
            Self::Zed => ("zed".into(), vec![root.as_os_str().to_owned(), target]),
            Self::Clion => (
                if cfg!(target_os = "windows") {
                    "clion64.exe"
                } else {
                    "clion"
                }
                .into(),
                vec!["--line".into(), line.into(), path.into_os_string()],
            ),
            Self::System => system_open_command(&path),
        };
        Ok(EditorCommand {
            program,
            arguments,
            working_directory: root,
        })
    }
}

pub struct EditorCommand {
    pub program: OsString,
    pub arguments: Vec<OsString>,
    pub working_directory: PathBuf,
}

fn system_open_command(path: &Path) -> (OsString, Vec<OsString>) {
    #[cfg(target_os = "windows")]
    {
        ("explorer.exe".into(), vec![path.as_os_str().to_owned()])
    }
    #[cfg(target_os = "macos")]
    {
        ("open".into(), vec![path.as_os_str().to_owned()])
    }
    #[cfg(not(any(target_os = "windows", target_os = "macos")))]
    {
        ("xdg-open".into(), vec![path.as_os_str().to_owned()])
    }
}

pub fn relevant_project_path(root: &Path, path: &Path) -> bool {
    let Ok(relative) = path.strip_prefix(root) else {
        return false;
    };
    !relative.components().any(|component| {
        matches!(
            component.as_os_str().to_str(),
            Some(".git" | "target" | "build" | "install" | "log")
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_points_follow_pan_without_leaving_visible_bounds() {
        let viewport = Viewport {
            x: -17.0,
            y: 38.0,
            scale: 1.0,
        };
        let points = viewport.grid_points(64.0, 64.0).collect::<Vec<_>>();
        assert_eq!(
            points,
            vec![(15.0, 6.0), (47.0, 6.0), (15.0, 38.0), (47.0, 38.0)]
        );
    }

    #[test]
    fn grid_density_stays_bounded_when_zooming_out() {
        let viewport = Viewport {
            scale: 0.25,
            ..Viewport::default()
        };
        assert_eq!(viewport.grid_points(1280.0, 800.0).count(), 4000);
        assert_eq!(Viewport::default().grid_points(1280.0, 800.0).count(), 1000);
    }

    #[test]
    fn grid_points_ignore_empty_or_invalid_canvas_dimensions() {
        for (width, height) in [
            (0.0, 800.0),
            (1280.0, -1.0),
            (f32::NAN, 800.0),
            (1280.0, f32::INFINITY),
        ] {
            assert_eq!(Viewport::default().grid_points(width, height).count(), 0);
        }
    }

    fn fixture() -> Result<GraphSnapshot> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/drone_demo_ws");
        let project: Project = serde_json::from_str(include_str!(
            "../../../examples/drone_demo_ws/expected_graph.json"
        ))?;
        let edges = graph_edges(&project);
        let positions = graph_positions(&project, &edges);
        Ok(GraphSnapshot {
            root,
            project,
            edges,
            positions,
        })
    }

    #[test]
    fn scans_existing_ros_fixture_without_zed() -> Result<()> {
        let original = fixture()?;
        let directory = tempfile::tempdir()?;
        let mut files = BTreeSet::new();
        for package in &original.project.packages {
            files.insert(format!("{}/package.xml", package.path));
            files.insert(format!("{}/Cargo.toml", package.path));
        }
        for source in original
            .project
            .nodes
            .iter()
            .flat_map(|node| &node.source_locations)
        {
            files.insert(source.path.clone());
        }
        // Copy only the golden fixture's inputs, not locally generated test binaries.
        for relative in files {
            let destination = directory.path().join(&relative);
            std::fs::create_dir_all(destination.parent().context("Fixture path has no parent")?)?;
            std::fs::copy(original.root.join(relative), destination)?;
        }
        let graph = GraphSnapshot::scan(directory.path())?;
        assert_eq!(graph.project.packages.len(), 4);
        assert_eq!(graph.project.nodes.len(), 5);
        for name in [
            "camera",
            "detector",
            "navigation",
            "autopilot_bridge",
            "test",
        ] {
            assert!(
                graph
                    .project
                    .nodes
                    .iter()
                    .any(|node| node.logical_name == name)
            );
        }
        assert!(graph.edges.iter().any(|edge| edge.topic == "/camera/image"));
        assert!(graph.edges.iter().any(|edge| edge.topic == "/cmd_vel"));
        assert_eq!(graph.positions.len(), graph.project.nodes.len());
        Ok(())
    }

    #[test]
    fn refresh_preserves_moved_positions() -> Result<()> {
        let mut previous = fixture()?;
        let moved = previous
            .positions
            .first_mut()
            .context("Fixture has no positions")?;
        moved.x = -250.0;
        moved.y = 900.0;
        let moved = moved.clone();
        let mut next = fixture()?;
        next.preserve_positions(&previous);
        assert!(next.positions.contains(&moved));
        Ok(())
    }

    #[test]
    fn new_nodes_do_not_overlap_preserved_positions() -> Result<()> {
        let mut previous = fixture()?;
        let moved = previous
            .positions
            .first_mut()
            .context("Fixture has no positions")?;
        moved.x = 48.0;
        moved.y = 48.0;
        let mut next = fixture()?;
        next.positions.push(NodePosition {
            node_id: "new-node".to_owned(),
            x: 48.0,
            y: 48.0,
        });
        next.preserve_positions(&previous);
        let position = next.positions.last().context("Missing new node")?;
        assert!(
            next.positions
                .iter()
                .filter(|other| other.node_id != "new-node")
                .all(|other| (position.x - other.x).abs() >= NODE_WIDTH + 24.0
                    || (position.y - other.y).abs() >= NODE_HEIGHT + 24.0)
        );
        Ok(())
    }

    #[test]
    fn edges_match_canonical_types_and_deduplicate_endpoints() -> Result<()> {
        let mut project = fixture()?.project;
        let camera = project
            .nodes
            .iter_mut()
            .find(|node| node.logical_name == "camera")
            .context("Missing camera")?;
        let publisher = camera
            .endpoints
            .iter()
            .find(|endpoint| endpoint.name == "/camera/image")
            .context("Missing publisher")?
            .clone();
        camera.endpoints.push(publisher);
        let camera_id = camera.id.as_str().to_owned();
        let detector = project
            .nodes
            .iter_mut()
            .find(|node| node.logical_name == "detector")
            .context("Missing detector")?;
        let detector_id = detector.id.as_str().to_owned();
        let subscription = detector
            .endpoints
            .iter_mut()
            .find(|endpoint| endpoint.name == "/camera/image")
            .context("Missing subscriber")?;
        subscription.type_name = "sensor_msgs/msg/Image".to_owned();
        assert_eq!(
            graph_edges(&project)
                .iter()
                .filter(|edge| edge.topic == "/camera/image"
                    && edge.publisher_id == camera_id
                    && edge.subscriber_id == detector_id)
                .count(),
            1
        );
        let detector = project
            .nodes
            .iter_mut()
            .find(|node| node.logical_name == "detector")
            .context("Missing detector")?;
        let subscription = detector
            .endpoints
            .iter_mut()
            .find(|endpoint| endpoint.name == "/camera/image")
            .context("Missing subscriber")?;
        subscription.type_name = "std_msgs/msg/String".to_owned();
        assert!(
            !graph_edges(&project)
                .iter()
                .any(|edge| edge.topic == "/camera/image"
                    && edge.publisher_id == camera_id
                    && edge.subscriber_id == detector_id)
        );
        Ok(())
    }

    #[test]
    fn layout_handles_cycles_and_self_subscriptions() -> Result<()> {
        let project = fixture()?.project;
        let first = project.nodes.first().context("Fixture has no nodes")?;
        let second = project.nodes.get(1).context("Fixture has only one node")?;
        let edge = |publisher: &str, subscriber: &str| GraphEdge {
            publisher_id: publisher.to_owned(),
            subscriber_id: subscriber.to_owned(),
            topic: "/cycle".to_owned(),
            type_name: "std_msgs/msg/String".to_owned(),
        };
        let positions = graph_positions(
            &project,
            &[
                edge(first.id.as_str(), second.id.as_str()),
                edge(second.id.as_str(), first.id.as_str()),
                edge(first.id.as_str(), first.id.as_str()),
            ],
        );
        assert_eq!(positions.len(), project.nodes.len());
        assert_eq!(
            positions
                .iter()
                .map(|position| &position.node_id)
                .collect::<BTreeSet<_>>()
                .len(),
            project.nodes.len()
        );
        Ok(())
    }

    #[test]
    fn zoom_keeps_the_point_under_the_mouse_stationary() {
        let mut viewport = Viewport {
            x: -50.0,
            y: 80.0,
            scale: 1.0,
        };
        let (x, y) = viewport.screen_point(300.0, 200.0);
        viewport.zoom_at(1.5, x, y);
        assert_eq!(viewport.screen_point(300.0, 200.0), (x, y));
        viewport.zoom_at(100.0, x, y);
        assert_eq!(viewport.scale, 2.0);
    }

    #[test]
    fn fit_centers_nodes_inside_the_canvas() {
        let positions = [NodePosition {
            node_id: "camera".to_owned(),
            x: -700.0,
            y: 900.0,
        }];
        let mut viewport = Viewport::default();
        viewport.fit(&positions, 1000.0, 700.0);
        assert_eq!(
            viewport.screen_point(-700.0 + NODE_WIDTH / 2.0, 900.0 + NODE_HEIGHT / 2.0),
            (500.0, 350.0)
        );
    }

    fn opening_positions() -> Vec<NodePosition> {
        // A narrow project with enough disconnected families to exceed one page.
        (0..6)
            .flat_map(|row| {
                (0..2).map(move |column| NodePosition {
                    node_id: format!("node-{row}-{column}"),
                    x: -400.0 + column as f32 * 520.0,
                    y: 800.0 + row as f32 * 260.0,
                })
            })
            .collect()
    }

    #[test]
    fn initial_window_stays_inside_small_or_scaled_displays() {
        assert_eq!(initial_window_size(1280.0, 720.0), (1232.0, 640.0));
        assert_eq!(initial_window_size(1920.0, 1080.0), (1280.0, 800.0));
        assert_eq!(initial_window_size(2560.0, 1440.0), (1280.0, 800.0));
        assert_eq!(
            initial_window_size(1920.0 / 1.5, 1080.0 / 1.5),
            (1232.0, 640.0)
        );
        assert_eq!(
            initial_window_size(2560.0 / 2.0, 1440.0 / 2.0),
            (1232.0, 640.0)
        );
    }

    #[test]
    fn opening_zoom_is_readable_for_720p_1080p_and_1440p_canvases() {
        let positions = opening_positions();
        // Logical canvas sizes after toolbar/status space and both sidebars.
        for (width, height, expected_scale) in [
            (680.0, 540.0, 0.9),
            (1320.0, 900.0, (900.0_f32 / 720.0).sqrt()),
            (1960.0, 1260.0, 1.15),
        ] {
            let mut opening = Viewport::default();
            opening.open_project(&positions, width, height);
            assert!((opening.scale - expected_scale).abs() < 0.001);
            let (x, y) = opening.screen_point(positions[0].x, positions[0].y);
            assert!(x >= 24.0 && y >= 24.0);
            assert!(x + NODE_WIDTH * opening.scale <= width);
            assert!(y + NODE_HEIGHT * opening.scale <= height);
            // The explicit overview still fits every card on these canvases.
            let mut overview = opening;
            overview.fit(&positions, width, height);
            assert!(overview.scale < opening.scale);
            for position in &positions {
                let (x, y) = overview.screen_point(position.x, position.y);
                assert!(x >= 0.0 && y >= 0.0);
                assert!(x + NODE_WIDTH * overview.scale <= width);
                assert!(y + NODE_HEIGHT * overview.scale <= height);
            }
        }
    }

    #[test]
    fn opening_zoom_uses_logical_canvas_dimensions_not_physical_monitor_pixels() {
        let positions = opening_positions();
        let mut at_720p = Viewport::default();
        at_720p.open_project(&positions, 1280.0 - 600.0, 720.0 - 180.0);
        let mut at_1080p_scaled = Viewport::default();
        at_1080p_scaled.open_project(&positions, 1920.0 / 1.5 - 600.0, 1080.0 / 1.5 - 180.0);
        let mut at_1440p_scaled = Viewport::default();
        at_1440p_scaled.open_project(&positions, 2560.0 / 2.0 - 600.0, 1440.0 / 2.0 - 180.0);
        assert_eq!(at_720p, at_1080p_scaled);
        assert_eq!(at_720p, at_1440p_scaled);
    }

    #[test]
    fn opening_centers_small_projects_and_anchors_oversized_projects_at_the_top_left() {
        let mut positions = opening_positions();
        let mut viewport = Viewport::default();
        viewport.open_project(&positions[..1], 1320.0, 900.0);
        assert_eq!(
            viewport.screen_point(
                positions[0].x + NODE_WIDTH / 2.0,
                positions[0].y + NODE_HEIGHT / 2.0
            ),
            (660.0, 450.0)
        );
        positions[1].x += 4000.0;
        viewport.open_project(&positions, 1320.0, 900.0);
        assert_eq!(viewport.scale, 0.9);
        assert_eq!(
            viewport.screen_point(positions[0].x, positions[0].y),
            (24.0, 24.0)
        );
        assert!(viewport.x.is_finite() && viewport.y.is_finite());
    }

    #[test]
    fn opening_handles_tiny_empty_and_invalid_canvases_without_losing_the_viewport() {
        let positions = opening_positions();
        let mut viewport = Viewport::default();
        viewport.open_project(&positions, 240.0, 180.0);
        assert!((0.25..0.9).contains(&viewport.scale));
        let (x, y) = viewport.screen_point(positions[0].x, positions[0].y);
        assert!(x + NODE_WIDTH * viewport.scale <= 240.0);
        assert!(y + NODE_HEIGHT * viewport.scale <= 180.0);
        let previous = viewport;
        for (width, height) in [
            (0.0, 600.0),
            (600.0, 0.0),
            (-1.0, 600.0),
            (f32::NAN, 600.0),
            (600.0, f32::INFINITY),
        ] {
            viewport.open_project(&positions, width, height);
            assert_eq!(viewport, previous);
        }
        viewport.open_project(&[], 1320.0, 900.0);
        assert_eq!(viewport, previous);
        let mut invalid_positions = positions;
        invalid_positions[0].x = f32::NAN;
        viewport.open_project(&invalid_positions, 1320.0, 900.0);
        assert_eq!(viewport, previous);
    }

    #[test]
    fn editor_arguments_preserve_spaces_and_use_one_based_lines() -> Result<()> {
        let root = tempfile::tempdir()?;
        let source_path = "camera backup;echo.rs";
        std::fs::write(root.path().join(source_path), "fn main() {}")?;
        let source = SourceLocation {
            path: source_path.to_owned(),
            line: 12,
            column: 4,
        };
        let command = ExternalEditor::VsCode.source_command(root.path(), &source)?;
        assert_eq!(
            command.program,
            OsString::from(if cfg!(target_os = "windows") {
                "code.cmd"
            } else {
                "code"
            })
        );
        assert_eq!(command.arguments.len(), 4);
        assert_eq!(command.arguments.get(2), Some(&OsString::from("--goto")));
        let target = command
            .arguments
            .last()
            .context("Missing source argument")?
            .to_string_lossy();
        assert!(target.ends_with("camera backup;echo.rs:13:1"));
        Ok(())
    }

    #[test]
    fn refuses_sources_outside_the_project() -> Result<()> {
        let parent = tempfile::tempdir()?;
        let project = parent.path().join("project");
        std::fs::create_dir(&project)?;
        std::fs::write(parent.path().join("outside.rs"), "")?;
        let source = SourceLocation {
            path: "../outside.rs".to_owned(),
            line: 0,
            column: 0,
        };
        assert!(
            ExternalEditor::Zed
                .source_command(&project, &source)
                .is_err()
        );
        Ok(())
    }

    #[test]
    fn ignores_build_outputs_when_watching_sources() {
        let root = Path::new("/project");
        assert!(relevant_project_path(root, &root.join("src/camera.rs")));
        for directory in [".git", "target", "build", "install", "log"] {
            assert!(!relevant_project_path(
                root,
                &root.join(directory).join("file")
            ));
        }
        assert!(!relevant_project_path(
            root,
            Path::new("/other/src/camera.rs")
        ));
    }
}
