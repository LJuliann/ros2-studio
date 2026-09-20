#![forbid(unsafe_code)]

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    time::Duration,
};

#[cfg(test)]
use anyhow::Context as _;
use editor::{Editor, SelectionEffects, scroll::Autoscroll};
use gpui::{
    Action, App, Bounds, ClickEvent, Context, Div, Entity, EventEmitter, FocusHandle, Focusable,
    InteractiveElement, KeyContext, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent,
    PathBuilder, Pixels, Point, Render, ScrollDelta, ScrollWheelEvent, SharedString, Subscription,
    Task, TaskExt, WeakEntity, Window, actions, anchored, canvas, deferred, point,
};
use rope::Point as TextPoint;
use ros_studio_model::{
    Confidence, EndpointKind, EntityId, Node, Package, Project, RuntimeState, SourceLocation,
    design_live::{canonical_ros_type_name, reconcile_project},
};
use ros_studio_protocol::GraphPatch;
use ui::{
    Button, ButtonLike, ContextMenu, Headline, HeadlineSize, Label, LabelSize, PopoverMenu,
    PopoverMenuHandle, SplitButton, Tooltip, prelude::*,
};
use workspace::{
    Pane, StatusItemView, ToolbarItemEvent, ToolbarItemLocation, ToolbarItemView, Workspace,
    dock::{DockPosition, Panel, PanelEvent},
    item::{Item, ItemEvent},
};

mod node_wizard;
mod runtime_client;

use runtime_client::RuntimeMessage;

#[cfg(test)]
const FIXTURE_GRAPH: &str = include_str!("../../../examples/drone_demo_ws/expected_graph.json");
const RESCAN_DEBOUNCE: Duration = Duration::from_millis(200);
const MINIMUM_ZOOM_PERCENT: u16 = 75;
const MAXIMUM_ZOOM_PERCENT: u16 = 150;
const ZOOM_STEP_PERCENT: u16 = 25;
const WHEEL_ZOOM_STEP_PERCENT: u16 = 10;
#[cfg(test)]
const GRAPH_MINIMUM_HEIGHT: f32 = 720.0;
const GRAPH_TOP_PADDING: f32 = 64.0;
#[cfg(test)]
const GRAPH_BOTTOM_PADDING: f32 = 64.0;
const GRAPH_ROW_GAP: f32 = 120.0;
const GRAPH_DOT_SPACING: f32 = 32.0;
const NODE_CARD_WIDTH: f32 = 240.0;
const NODE_CARD_HEIGHT: f32 = 176.0;
const NEW_NODE_HORIZONTAL_GAP: f32 = 80.0;
const NEW_NODE_VERTICAL_GAP: f32 = 64.0;
const DEFAULT_VIEWPORT_CENTER_X: f32 = 600.0;
const DEFAULT_VIEWPORT_CENTER_Y: f32 = 360.0;
const EDGE_LABEL_WIDTH: f32 = 200.0;
const EDGE_LABEL_OFFSET: f32 = 20.0;
const LEFT_COLUMN_X: f32 = 48.0;
const RIGHT_COLUMN_X: f32 = 500.0;
const LEFT_COLUMN_OFFSET_X: f32 = 52.0;
const RIGHT_COLUMN_OFFSET_X: f32 = 48.0;
const MAXIMUM_PROCESS_LOG_CHUNKS: usize = 256;

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct GraphEdge {
    publisher_node_id: String,
    subscriber_node_id: String,
    topic: String,
    type_name: String,
}

#[derive(Clone, Debug, PartialEq)]
struct GraphNodeLayout {
    node_id: String,
    x: f32,
    y: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GraphEdgeGeometry {
    start_x: f32,
    start_y: f32,
    end_x: f32,
    end_y: f32,
    label_x: f32,
    label_y: f32,
}

#[derive(Clone, Debug)]
struct GraphNodeDrag {
    node_id: String,
    mouse_start_x: f32,
    mouse_start_y: f32,
    node_start_x: f32,
    node_start_y: f32,
}

#[derive(Clone, Copy, Debug)]
struct GraphCanvasPan {
    mouse_start_x: f32,
    mouse_start_y: f32,
    camera_start_x: f32,
    camera_start_y: f32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum GraphMode {
    Design,
    Live,
    Both,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum InspectorView {
    #[default]
    Selection,
    Packages,
    Nodes,
    Topics,
}

impl GraphMode {
    fn label(self) -> &'static str {
        match self {
            Self::Design => "DESIGN",
            Self::Live => "LIVE",
            Self::Both => "BOTH",
        }
    }
}

enum RuntimeStatus {
    NotStarted,
    Connecting,
    Ready,
    Failed(String),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ProcessKind {
    Build,
    Run,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RunTargetKind {
    Node,
    File,
}

impl ProcessKind {
    fn label(self) -> &'static str {
        match self {
            Self::Build => "Build",
            Self::Run => "Run",
        }
    }
}

#[derive(Default)]
enum ProcessState {
    #[default]
    Idle,
    Starting,
    Running(String),
    Stopping(String),
    Exited {
        success: bool,
        code: Option<i32>,
    },
    Failed(String),
}

#[derive(Clone)]
struct ProcessLogChunk {
    text: String,
}

#[derive(Default)]
struct ProcessPanel {
    label: String,
    state: ProcessState,
    output: Vec<ProcessLogChunk>,
}

impl ProcessPanel {
    fn is_active(&self) -> bool {
        matches!(
            &self.state,
            ProcessState::Starting | ProcessState::Running(_) | ProcessState::Stopping(_)
        )
    }

    fn status_label(&self) -> Option<String> {
        match &self.state {
            ProcessState::Idle => None,
            ProcessState::Starting => Some(format!("{} · STARTING", self.label)),
            ProcessState::Running(_) => Some(format!("{} · RUNNING", self.label)),
            ProcessState::Stopping(_) => Some(format!("{} · STOPPING", self.label)),
            ProcessState::Exited { success: true, .. } => {
                Some(format!("{} · FINISHED", self.label))
            }
            ProcessState::Exited {
                success: false,
                code,
            } => Some(format!(
                "{} · FAILED{}",
                self.label,
                code.map(|code| format!(" ({code})")).unwrap_or_default()
            )),
            ProcessState::Failed(error) => Some(format!("{} · {error}", self.label)),
        }
    }
}

#[derive(Default)]
struct RuntimeOverlay {
    project_id: Option<EntityId>,
    received_snapshot: bool,
    packages: BTreeMap<EntityId, Package>,
    nodes: BTreeMap<EntityId, Node>,
}

impl RuntimeOverlay {
    fn apply_patch(&mut self, patch: GraphPatch, expected_project_id: &EntityId) -> bool {
        if &patch.project_id != expected_project_id {
            return false;
        }

        self.project_id = Some(patch.project_id);
        self.received_snapshot = true;
        for package in patch.upsert_packages {
            self.packages.insert(package.id.clone(), package);
        }
        for package_id in patch.removed_package_ids {
            self.packages.remove(&package_id);
        }
        for node in patch.upsert_nodes {
            self.nodes.insert(node.id.clone(), node);
        }
        for node_id in patch.removed_node_ids {
            self.nodes.remove(&node_id);
        }
        true
    }

    fn snapshot(&self, design: &Project) -> Option<Project> {
        if !self.received_snapshot || self.project_id.as_ref() != Some(&design.id) {
            return None;
        }

        Some(Project {
            id: design.id.clone(),
            name: design.name.clone(),
            root_path: design.root_path.clone(),
            packages: self.packages.values().cloned().collect(),
            nodes: self.nodes.values().cloned().collect(),
        })
    }
}

fn project_for_mode(design: &Project, merged: &Project, mode: GraphMode) -> Project {
    match mode {
        GraphMode::Design => design.clone(),
        GraphMode::Both => merged.clone(),
        GraphMode::Live => {
            let mut live = merged.clone();
            live.nodes.retain(|node| {
                matches!(
                    node.runtime_state,
                    RuntimeState::Online | RuntimeState::RuntimeOnly | RuntimeState::Conflict
                )
            });
            let visible_package_ids = live
                .nodes
                .iter()
                .map(|node| node.package_id.clone())
                .collect::<BTreeSet<_>>();
            live.packages
                .retain(|package| visible_package_ids.contains(&package.id));
            live
        }
    }
}

actions!(
    ros_studio,
    [
        /// Opens the ROS 2 Studio graph.
        OpenGraph,
        /// Shows or closes the ROS 2 Studio graph.
        ToggleGraph,
        /// Creates a Rust ROS node in the open workspace.
        CreateNode,
        /// Builds the selected ROS node.
        BuildNode,
        /// Compiles the active source file.
        BuildFile,
        /// Builds the configured target for the active file.
        BuildContext,
        /// Runs the selected ROS node.
        RunNode,
        /// Runs the active source file.
        RunFile,
        /// Runs the ROS node matching the active file, or the file itself.
        RunContext,
        /// Chooses whether to run a matching ROS node or the active file.
        ChooseRunTarget,
        /// Stops the running ROS node.
        StopNode,
        /// Shows or closes ROS build and run output.
        ToggleProcessPanel
    ]
);

pub fn init(cx: &mut App) {
    cx.observe_new(|pane: &mut Pane, window, cx| {
        if let Some(window) = window {
            let controls = cx.new(|_| RosRunToolbar::new());
            pane.toolbar().update(cx, |toolbar, cx| {
                toolbar.add_item(controls, window, cx);
            });
        }
    })
    .detach();

    cx.observe_new(|workspace: &mut Workspace, window, cx| {
        workspace.register_action(|workspace, _: &OpenGraph, window, cx| {
            open_graph(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &ToggleGraph, window, cx| {
            toggle_graph(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &CreateNode, window, cx| {
            open_node_wizard(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &BuildNode, window, cx| {
            set_active_run_target(workspace, RunTargetKind::Node, cx);
            start_workspace_process(workspace, ProcessKind::Build, window, cx);
        });
        workspace.register_action(|workspace, _: &BuildFile, window, cx| {
            set_active_run_target(workspace, RunTargetKind::File, cx);
            start_active_file_process(workspace, ProcessKind::Build, window, cx);
        });
        workspace.register_action(|workspace, _: &BuildContext, window, cx| {
            start_build_context_process(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &RunNode, window, cx| {
            set_active_run_target(workspace, RunTargetKind::Node, cx);
            start_workspace_process(workspace, ProcessKind::Run, window, cx);
        });
        workspace.register_action(|workspace, _: &RunFile, window, cx| {
            set_active_run_target(workspace, RunTargetKind::File, cx);
            start_active_file_process(workspace, ProcessKind::Run, window, cx);
        });
        workspace.register_action(|workspace, _: &RunContext, window, cx| {
            start_context_process(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &ChooseRunTarget, window, cx| {
            show_run_target_menu(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &StopNode, window, cx| {
            stop_workspace_process(workspace, window, cx);
        });
        workspace.register_action(|workspace, _: &ToggleProcessPanel, window, cx| {
            if !workspace.toggle_panel_focus::<RosProcessPanel>(window, cx) {
                workspace.close_panel::<RosProcessPanel>(window, cx);
            }
        });

        if let Some(window) = window {
            let process_panel = cx.new(|cx| RosProcessPanel::new(cx));
            workspace.add_panel(process_panel, window, cx);
            let status_button = cx.new(|_| RosGraphStatusButton::new());
            workspace.status_bar().update(cx, |status_bar, cx| {
                status_bar.add_left_item(status_button, window, cx);
            });
        }
    })
    .detach();
}

fn graph_item(workspace: &Workspace, cx: &App) -> Option<(Entity<Pane>, Entity<RosGraph>)> {
    workspace.panes().iter().find_map(|pane| {
        pane.read(cx)
            .items()
            .find_map(|item| item.downcast::<RosGraph>())
            .map(|graph| (pane.clone(), graph))
    })
}

fn open_graph(workspace: &mut Workspace, window: &mut Window, cx: &mut Context<Workspace>) {
    if let Some((_, graph)) = graph_item(workspace, cx) {
        workspace.activate_item(&graph, true, true, window, cx);
        return;
    }

    let workspace_handle = cx.weak_entity();
    let workspace_root = workspace
        .worktrees(cx)
        .next()
        .map(|worktree| worktree.read(cx).abs_path().to_path_buf());
    let project = workspace.project().clone();
    let graph = cx.new(|cx| RosGraph::new(workspace_handle, workspace_root, project, window, cx));
    if let Some(process_panel) = workspace.panel::<RosProcessPanel>(cx) {
        process_panel.update(cx, |panel, cx| panel.set_graph(graph.clone(), cx));
    }
    workspace.add_item_to_active_pane(Box::new(graph), None, true, window, cx);
}

fn start_workspace_process(
    workspace: &mut Workspace,
    kind: ProcessKind,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    workspace.reveal_panel::<RosProcessPanel>(window, cx);
    if let Some((_, graph)) = graph_item(workspace, cx) {
        let active_source_path = active_source_path(workspace, cx);
        graph.update(cx, |graph, cx| {
            if let Some(active_source_path) = active_source_path.as_deref() {
                graph.select_node_for_source(active_source_path);
            }
            graph.start_selected_process(kind, cx);
        });
    }
}

fn start_active_file_process(
    workspace: &mut Workspace,
    kind: ProcessKind,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    workspace.reveal_panel::<RosProcessPanel>(window, cx);
    let active_source_path = active_source_path(workspace, cx);
    if let (Some(active_source_path), Some((_, graph))) =
        (active_source_path, graph_item(workspace, cx))
    {
        graph.update(cx, |graph, cx| {
            graph.start_source_file_process(&active_source_path, kind, cx);
        });
    }
}

fn start_context_process(
    workspace: &mut Workspace,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    match active_run_target(workspace, cx) {
        Some(RunTargetKind::Node) => {
            start_workspace_process(workspace, ProcessKind::Run, window, cx)
        }
        Some(RunTargetKind::File) => {
            start_active_file_process(workspace, ProcessKind::Run, window, cx)
        }
        None => show_run_target_menu(workspace, window, cx),
    }
}

fn start_build_context_process(
    workspace: &mut Workspace,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    match active_run_target(workspace, cx) {
        Some(RunTargetKind::Node) => {
            start_workspace_process(workspace, ProcessKind::Build, window, cx)
        }
        Some(RunTargetKind::File) => {
            start_active_file_process(workspace, ProcessKind::Build, window, cx)
        }
        None => show_build_target_menu(workspace, window, cx),
    }
}

fn active_source_path(workspace: &Workspace, cx: &App) -> Option<String> {
    let path = workspace.active_item(cx)?.project_path(cx)?.path;
    let path = path.as_unix_str();
    is_supported_source_path(path).then(|| path.to_owned())
}

fn show_run_target_menu(workspace: &Workspace, window: &mut Window, cx: &mut Context<Workspace>) {
    if let Some(controls) = active_run_toolbar(workspace, cx) {
        let menu_handle = controls.read(cx).run_menu_handle.clone();
        menu_handle.show(window, cx);
    }
}

fn show_build_target_menu(workspace: &Workspace, window: &mut Window, cx: &mut Context<Workspace>) {
    if let Some(controls) = active_run_toolbar(workspace, cx) {
        let menu_handle = controls.read(cx).build_menu_handle.clone();
        menu_handle.show(window, cx);
    }
}

fn active_run_toolbar(workspace: &Workspace, cx: &App) -> Option<Entity<RosRunToolbar>> {
    let toolbar = workspace.active_pane().read(cx).toolbar().clone();
    toolbar.read(cx).item_of_type::<RosRunToolbar>()
}

fn active_run_target(workspace: &Workspace, cx: &App) -> Option<RunTargetKind> {
    let controls = active_run_toolbar(workspace, cx)?;
    controls.read(cx).active_run_target()
}

fn set_active_run_target(
    workspace: &Workspace,
    target: RunTargetKind,
    cx: &mut Context<Workspace>,
) {
    if let Some(controls) = active_run_toolbar(workspace, cx) {
        controls.update(cx, |controls, cx| {
            controls.set_active_run_target(target);
            cx.notify();
        });
    }
}

fn is_supported_source_path(path: &str) -> bool {
    matches!(
        std::path::Path::new(path)
            .extension()
            .and_then(|extension| extension.to_str()),
        Some("rs" | "c" | "cc" | "cpp" | "cxx" | "py")
    )
}

fn stop_workspace_process(
    workspace: &mut Workspace,
    window: &mut Window,
    cx: &mut Context<Workspace>,
) {
    workspace.reveal_panel::<RosProcessPanel>(window, cx);
    if let Some((_, graph)) = graph_item(workspace, cx) {
        graph.update(cx, RosGraph::stop_selected_process);
    }
}

fn toggle_graph(workspace: &mut Workspace, window: &mut Window, cx: &mut Context<Workspace>) {
    if let Some((pane, graph)) = graph_item(workspace, cx) {
        pane.update(cx, |pane, cx| {
            pane.remove_item(graph.entity_id(), false, false, window, cx);
        });
    } else {
        open_graph(workspace, window, cx);
    }
}

fn open_node_wizard(workspace: &mut Workspace, window: &mut Window, cx: &mut Context<Workspace>) {
    let workspace_root = workspace
        .worktrees(cx)
        .next()
        .map(|worktree| worktree.read(cx).abs_path().to_path_buf());
    let Some(workspace_root) = workspace_root else {
        open_graph(workspace, window, cx);
        return;
    };
    let initial_project = graph_item(workspace, cx)
        .and_then(|(_, graph)| graph.read(cx).project.as_ref().ok().cloned());
    let workspace_handle = cx.weak_entity();
    workspace.toggle_modal(window, cx, move |window, cx| {
        node_wizard::NodeWizard::new(
            workspace_handle,
            workspace_root,
            initial_project,
            window,
            cx,
        )
    });
}

struct RosGraphStatusButton {
    graph_is_active: bool,
}

impl RosGraphStatusButton {
    fn new() -> Self {
        Self {
            graph_is_active: false,
        }
    }
}

impl Render for RosGraphStatusButton {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        Button::new("ros-graph-status-button", "ROS Graph")
            .label_size(LabelSize::Small)
            .toggle_state(self.graph_is_active)
            .tooltip(Tooltip::text("Show or close ROS Graph"))
            .on_click(cx.listener(|_, _, window, cx| {
                window.dispatch_action(Box::new(ToggleGraph), cx);
            }))
    }
}

struct RosRunToolbar {
    active_source_path: Option<String>,
    run_targets: BTreeMap<String, RunTargetKind>,
    build_menu_handle: PopoverMenuHandle<ContextMenu>,
    run_menu_handle: PopoverMenuHandle<ContextMenu>,
}

impl RosRunToolbar {
    fn new() -> Self {
        Self {
            active_source_path: None,
            run_targets: BTreeMap::new(),
            build_menu_handle: PopoverMenuHandle::default(),
            run_menu_handle: PopoverMenuHandle::default(),
        }
    }

    fn active_run_target(&self) -> Option<RunTargetKind> {
        self.active_source_path
            .as_ref()
            .and_then(|path| self.run_targets.get(path))
            .copied()
    }

    fn set_active_run_target(&mut self, target: RunTargetKind) {
        if let Some(path) = self.active_source_path.clone() {
            self.run_targets.insert(path, target);
        }
    }
}

impl EventEmitter<ToolbarItemEvent> for RosRunToolbar {}

impl ToolbarItemView for RosRunToolbar {
    fn set_active_pane_item(
        &mut self,
        active_pane_item: Option<&dyn workspace::ItemHandle>,
        _: &mut Window,
        cx: &mut Context<Self>,
    ) -> ToolbarItemLocation {
        self.active_source_path = active_pane_item
            .and_then(|item| item.project_path(cx))
            .map(|path| path.path.as_unix_str().to_owned())
            .filter(|path| is_supported_source_path(path));
        if self.active_source_path.is_some() {
            ToolbarItemLocation::PrimaryRight
        } else {
            ToolbarItemLocation::Hidden
        }
    }

    fn contribute_context(&self, context: &mut KeyContext, _: &App) {
        if self.active_source_path.is_some() {
            context.add("ROSStudioSource");
        }
    }
}

impl Render for RosRunToolbar {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        let build_menu_handle = self.build_menu_handle.clone();
        let run_menu_handle = self.run_menu_handle.clone();
        let (build_label, run_label) = match self.active_run_target() {
            Some(RunTargetKind::Node) => ("Build node", "Run node"),
            Some(RunTargetKind::File) => ("Build file", "Run file"),
            None => ("Build", "Run"),
        };
        h_flex()
            .gap_1()
            .child(SplitButton::new(
                ButtonLike::new("ros-toolbar-build")
                    .child(Label::new(build_label).size(LabelSize::Small))
                    .on_click(|_, window, cx| {
                        window.dispatch_action(Box::new(BuildContext), cx);
                    }),
                PopoverMenu::new("ros-toolbar-build-menu")
                    .anchor(gpui::Anchor::TopRight)
                    .with_handle(build_menu_handle)
                    .trigger(
                        IconButton::new("ros-toolbar-build-menu-trigger", IconName::ChevronDown)
                            .icon_size(IconSize::XSmall),
                    )
                    .menu(|window, cx| {
                        Some(ContextMenu::build(window, cx, |menu, _, _| {
                            menu.entry(
                                "Build matching ROS node",
                                Some(Box::new(BuildNode)),
                                |window, cx| {
                                    window.dispatch_action(Box::new(BuildNode), cx);
                                },
                            )
                            .entry(
                                "Compile current file",
                                Some(Box::new(BuildFile)),
                                |window, cx| {
                                    window.dispatch_action(Box::new(BuildFile), cx);
                                },
                            )
                        }))
                    })
                    .into_any_element(),
            ))
            .child(SplitButton::new(
                ButtonLike::new("ros-toolbar-run")
                    .child(Label::new(run_label).size(LabelSize::Small))
                    .on_click(|_, window, cx| {
                        window.dispatch_action(Box::new(RunContext), cx);
                    }),
                PopoverMenu::new("ros-toolbar-run-menu")
                    .anchor(gpui::Anchor::TopRight)
                    .with_handle(run_menu_handle)
                    .trigger(
                        IconButton::new("ros-toolbar-run-menu-trigger", IconName::ChevronDown)
                            .icon_size(IconSize::XSmall),
                    )
                    .menu(|window, cx| {
                        Some(ContextMenu::build(window, cx, |menu, _, _| {
                            menu.entry(
                                "Run matching ROS node",
                                Some(Box::new(RunNode)),
                                |window, cx| {
                                    window.dispatch_action(Box::new(RunNode), cx);
                                },
                            )
                            .entry(
                                "Run current file",
                                Some(Box::new(RunFile)),
                                |window, cx| {
                                    window.dispatch_action(Box::new(RunFile), cx);
                                },
                            )
                        }))
                    })
                    .into_any_element(),
            ))
            .child(
                Button::new("ros-toolbar-stop", "Stop").on_click(|_, window, cx| {
                    window.dispatch_action(Box::new(StopNode), cx);
                }),
            )
            .child(
                Button::new("ros-toolbar-output", "Output").on_click(|_, window, cx| {
                    window.dispatch_action(Box::new(ToggleProcessPanel), cx);
                }),
            )
    }
}

struct RosProcessPanel {
    graph: Option<WeakEntity<RosGraph>>,
    _graph_subscription: Option<Subscription>,
    focus_handle: FocusHandle,
}

impl RosProcessPanel {
    fn new(cx: &mut Context<Self>) -> Self {
        Self {
            graph: None,
            _graph_subscription: None,
            focus_handle: cx.focus_handle(),
        }
    }

    fn set_graph(&mut self, graph: Entity<RosGraph>, cx: &mut Context<Self>) {
        self._graph_subscription = Some(cx.observe(&graph, |_, _, cx| cx.notify()));
        self.graph = Some(graph.downgrade());
        cx.notify();
    }
}

impl EventEmitter<PanelEvent> for RosProcessPanel {}

impl Focusable for RosProcessPanel {
    fn focus_handle(&self, _: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl Panel for RosProcessPanel {
    fn persistent_name() -> &'static str {
        "RosProcessPanel"
    }

    fn panel_key() -> &'static str {
        "RosProcessPanel"
    }

    fn position(&self, _: &Window, _: &App) -> DockPosition {
        DockPosition::Bottom
    }

    fn position_is_valid(&self, position: DockPosition) -> bool {
        position == DockPosition::Bottom
    }

    fn set_position(&mut self, _: DockPosition, _: &mut Window, _: &mut Context<Self>) {}

    fn default_size(&self, _: &Window, _: &App) -> Pixels {
        px(240.0)
    }

    fn min_size(&self, _: &Window, _: &App) -> Option<Pixels> {
        Some(px(100.0))
    }

    fn icon(&self, _: &Window, _: &App) -> Option<IconName> {
        Some(IconName::TerminalAlt)
    }

    fn icon_tooltip(&self, _: &Window, _: &App) -> Option<&'static str> {
        Some("ROS Build and Run Output")
    }

    fn toggle_action(&self) -> Box<dyn Action> {
        Box::new(ToggleProcessPanel)
    }

    fn activation_priority(&self) -> u32 {
        8
    }
}

impl Render for RosProcessPanel {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let graph = self.graph.as_ref().and_then(WeakEntity::upgrade);
        let (status, can_start, can_stop, log_editor) = graph
            .as_ref()
            .map(|graph| {
                let graph = graph.read(cx);
                let can_start = graph.project.as_ref().is_ok_and(|project| {
                    ros_node_command(
                        project,
                        graph.selected_node_id.as_deref(),
                        ProcessKind::Build,
                    )
                    .is_some()
                }) && !graph.process_panel.is_active();
                (
                    graph.process_panel.status_label(),
                    can_start,
                    matches!(&graph.process_panel.state, ProcessState::Running(_)),
                    Some(graph.process_log_editor.clone()),
                )
            })
            .unwrap_or((None, false, false, None));

        v_flex()
            .size_full()
            .track_focus(&self.focus_handle)
            .bg(cx.theme().colors().editor_background)
            .child(
                h_flex()
                    .flex_none()
                    .justify_between()
                    .px_2()
                    .py_1()
                    .border_b_1()
                    .border_color(cx.theme().colors().border_variant)
                    .child(
                        Label::new(status.unwrap_or_else(|| "ROS build and run output".to_owned()))
                            .size(LabelSize::Small),
                    )
                    .child(
                        h_flex()
                            .gap_1()
                            .child(
                                Button::new("ros-process-panel-build", "Build")
                                    .disabled(!can_start)
                                    .on_click(|_, window, cx| {
                                        window.dispatch_action(Box::new(BuildNode), cx);
                                    }),
                            )
                            .child(
                                Button::new("ros-process-panel-run", "Run")
                                    .disabled(!can_start)
                                    .on_click(|_, window, cx| {
                                        window.dispatch_action(Box::new(RunNode), cx);
                                    }),
                            )
                            .child(
                                Button::new("ros-process-panel-stop", "Stop")
                                    .disabled(!can_stop)
                                    .on_click(|_, window, cx| {
                                        window.dispatch_action(Box::new(StopNode), cx);
                                    }),
                            )
                            .child(
                                Button::new("ros-process-panel-close", "Close").on_click(
                                    cx.listener(|_, _, _, cx| cx.emit(PanelEvent::Close)),
                                ),
                            ),
                    ),
            )
            .child(
                div()
                    .flex_1()
                    .overflow_hidden()
                    .when_some(log_editor, |output, editor| output.child(editor))
                    .when(graph.is_none(), |output| {
                        output.items_center().justify_center().child(
                            Label::new("Open ROS Graph and select a Rust node to build or run.")
                                .size(LabelSize::Small)
                                .color(Color::Muted),
                        )
                    }),
            )
    }
}

impl StatusItemView for RosGraphStatusButton {
    fn set_active_pane_item(
        &mut self,
        active_pane_item: Option<&dyn workspace::ItemHandle>,
        _window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        self.graph_is_active = active_pane_item
            .and_then(|item| item.downcast::<RosGraph>())
            .is_some();
        cx.notify();
    }

    fn hide_setting(&self, _: &App) -> Option<workspace::HideStatusItem> {
        None
    }
}

pub struct RosGraph {
    workspace: WeakEntity<Workspace>,
    workspace_root: Option<PathBuf>,
    project: Result<Project, SharedString>,
    displayed_project: Result<Project, SharedString>,
    graph_mode: GraphMode,
    auto_follow_process: bool,
    runtime_status: RuntimeStatus,
    runtime_diagnostic: Option<String>,
    runtime_overlay: RuntimeOverlay,
    runtime_commands: Option<async_channel::Sender<runtime_client::RuntimeCommand>>,
    process_panel: ProcessPanel,
    process_log_editor: Entity<Editor>,
    is_scanning: bool,
    node_layouts: Vec<GraphNodeLayout>,
    selected_node_id: Option<String>,
    inspector_view: InspectorView,
    node_drag: Option<GraphNodeDrag>,
    canvas_pan: Option<GraphCanvasPan>,
    context_menu: Option<(Entity<ContextMenu>, Point<Pixels>, Subscription)>,
    camera_offset_x: f32,
    camera_offset_y: f32,
    canvas_bounds: Option<Bounds<Pixels>>,
    zoom_percent: u16,
    focus_handle: FocusHandle,
    _rescan_task: Task<()>,
    _runtime_task: Task<()>,
    _subscriptions: Vec<Subscription>,
}

impl RosGraph {
    fn new(
        workspace: WeakEntity<Workspace>,
        workspace_root: Option<PathBuf>,
        project: Entity<project::Project>,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> Self {
        let subscriptions = vec![cx.subscribe(&project, Self::handle_project_event)];
        let process_log_editor = cx.new(|cx| {
            let mut editor = Editor::multi_line(window, cx);
            editor.set_read_only(true);
            editor.set_show_gutter(false, cx);
            editor.set_show_line_numbers(false, cx);
            editor.set_show_wrap_guides(false, cx);
            editor.set_show_indent_guides(false, cx);
            editor.set_autoindent(false);
            editor.set_input_enabled(false);
            editor
        });

        let mut graph = Self {
            workspace,
            workspace_root,
            project: Err("Scanning ROS workspace…".into()),
            displayed_project: Err("Scanning ROS workspace…".into()),
            graph_mode: GraphMode::Design,
            auto_follow_process: true,
            runtime_status: RuntimeStatus::NotStarted,
            runtime_diagnostic: None,
            runtime_overlay: RuntimeOverlay::default(),
            runtime_commands: None,
            process_panel: ProcessPanel::default(),
            process_log_editor,
            is_scanning: true,
            node_layouts: Vec::new(),
            selected_node_id: None,
            inspector_view: InspectorView::Selection,
            node_drag: None,
            canvas_pan: None,
            context_menu: None,
            camera_offset_x: 0.0,
            camera_offset_y: 0.0,
            canvas_bounds: None,
            zoom_percent: 100,
            focus_handle: cx.focus_handle(),
            _rescan_task: Task::ready(()),
            _runtime_task: Task::ready(()),
            _subscriptions: subscriptions,
        };
        graph.schedule_rescan(Duration::ZERO, cx);
        graph
    }

    fn handle_project_event(
        &mut self,
        _project: Entity<project::Project>,
        event: &project::Event,
        cx: &mut Context<Self>,
    ) {
        let project::Event::WorktreeUpdatedEntries(_, entries) = event else {
            return;
        };
        let contains_ros_source_change = entries.iter().any(|(path, _, _)| {
            matches!(
                path.extension(),
                Some("rs" | "py" | "c" | "cc" | "cpp" | "cxx" | "h" | "hh" | "hpp" | "hxx")
            ) || matches!(path.file_name(), Some("package.xml" | "Cargo.toml"))
        });

        if contains_ros_source_change {
            self.schedule_rescan(RESCAN_DEBOUNCE, cx);
        }
    }

    fn schedule_rescan(&mut self, delay: Duration, cx: &mut Context<Self>) {
        let Some(workspace_root) = self.workspace_root.clone() else {
            self.project = Err("No local workspace is open.".into());
            self.displayed_project = self.project.clone();
            self.is_scanning = false;
            cx.notify();
            return;
        };
        let project_name = workspace_root
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("ros_workspace")
            .to_owned();

        self.is_scanning = true;
        cx.notify();
        self._rescan_task = cx.spawn(async move |this, cx| {
            if !delay.is_zero() {
                cx.background_executor().timer(delay).await;
            }
            let scan_result = cx
                .background_spawn(async move {
                    ros_studio_scan::scan_project(&workspace_root, &project_name)
                })
                .await;

            let Some(this) = this.upgrade() else {
                return;
            };
            this.update(cx, |this, cx| {
                this.apply_scan_result(scan_result);
                cx.notify();
            });
        });
    }

    fn apply_scan_result(&mut self, scan_result: anyhow::Result<Project>) {
        self.is_scanning = false;
        match scan_result {
            Ok(project) => {
                self.project = Ok(project);
            }
            Err(error) => {
                self.project = Err(format!("Failed to scan ROS workspace: {error:#}").into());
            }
        }
        self.refresh_display();
    }

    fn refresh_display(&mut self) {
        let Ok(design) = &self.project else {
            self.displayed_project = self.project.clone();
            return;
        };

        let runtime = self.runtime_overlay.snapshot(design);
        let merged = reconcile_project(design, runtime.as_ref());
        let edges = graph_edges(&merged);
        let viewport_center = self.visible_graph_center();
        self.node_layouts =
            merge_graph_layouts(&self.node_layouts, &merged, &edges, viewport_center);
        self.selected_node_id = self.selected_node_id.take().filter(|selected_node_id| {
            merged
                .nodes
                .iter()
                .any(|node| node.id.as_str() == selected_node_id.as_str())
        });
        self.displayed_project = Ok(project_for_mode(design, &merged, self.graph_mode));
    }

    fn set_graph_mode(&mut self, mode: GraphMode, cx: &mut Context<Self>) {
        self.graph_mode = mode;
        self.refresh_display();
        if mode != GraphMode::Design
            && matches!(
                &self.runtime_status,
                RuntimeStatus::NotStarted | RuntimeStatus::Failed(_)
            )
        {
            self.start_runtime_discovery(cx);
        }
        cx.notify();
    }

    fn start_runtime_discovery(&mut self, cx: &mut Context<Self>) {
        if self.runtime_commands.is_some() {
            return;
        }
        let Some(workspace_root) = self.workspace_root.clone() else {
            self.runtime_status = RuntimeStatus::Failed("No local workspace is open".to_owned());
            return;
        };

        self.runtime_overlay = RuntimeOverlay::default();
        self.runtime_diagnostic = None;
        self.runtime_status = RuntimeStatus::Connecting;
        self.refresh_display();
        let (sender, receiver) = async_channel::unbounded();
        let (command_sender, command_receiver) = async_channel::unbounded();
        self.runtime_commands = Some(command_sender);
        let process_task = cx.background_spawn(async move {
            runtime_client::connect(workspace_root, sender, command_receiver).await
        });
        self._runtime_task = cx.spawn(async move |this, cx| {
            while let Ok(message) = receiver.recv().await {
                let Some(graph) = this.upgrade() else {
                    return;
                };
                graph.update(cx, |graph, cx| {
                    graph.handle_runtime_message(message, cx);
                    cx.notify();
                });
            }

            let result = process_task.await;
            if let Some(graph) = this.upgrade() {
                graph.update(cx, |graph, cx| {
                    graph.runtime_overlay = RuntimeOverlay::default();
                    graph.runtime_commands = None;
                    graph.runtime_status = RuntimeStatus::Failed(match result {
                        Ok(()) => "ROS 2 daemon disconnected".to_owned(),
                        Err(error) => format!("{error:#}"),
                    });
                    if graph.process_panel.is_active() {
                        graph.process_panel.state =
                            ProcessState::Failed("daemon disconnected".to_owned());
                    }
                    graph.refresh_display();
                    cx.notify();
                });
            }
        });
    }

    fn handle_runtime_message(&mut self, message: RuntimeMessage, cx: &mut Context<Self>) {
        match message {
            RuntimeMessage::GraphPatch(patch) => {
                let Some(project_id) = self.project.as_ref().ok().map(|project| project.id.clone())
                else {
                    return;
                };
                if self.runtime_overlay.apply_patch(patch, &project_id) {
                    self.runtime_status = RuntimeStatus::Ready;
                    self.runtime_diagnostic = None;
                    self.refresh_display();
                }
            }
            RuntimeMessage::Diagnostic(message) => {
                self.runtime_diagnostic = Some(message);
            }
            RuntimeMessage::ProcessStarted(id) => {
                if matches!(self.process_panel.state, ProcessState::Starting) {
                    self.process_panel.state = ProcessState::Running(id);
                }
            }
            RuntimeMessage::ProcessOutput { id, text } => {
                let is_current_process = matches!(
                    &self.process_panel.state,
                    ProcessState::Running(process_id) | ProcessState::Stopping(process_id)
                        if process_id == &id
                );
                if is_current_process {
                    self.process_panel.output.push(ProcessLogChunk { text });
                    if self.process_panel.output.len() > MAXIMUM_PROCESS_LOG_CHUNKS {
                        self.process_panel.output.remove(0);
                    }
                    self.sync_process_log_editor(cx);
                }
            }
            RuntimeMessage::ProcessExited { id, success, code } => {
                let is_current_process = matches!(
                    &self.process_panel.state,
                    ProcessState::Running(process_id) | ProcessState::Stopping(process_id)
                        if process_id == &id
                );
                if is_current_process {
                    self.process_panel.state = ProcessState::Exited { success, code };
                }
            }
            RuntimeMessage::RequestFailed(message) => {
                if self.process_panel.is_active() {
                    self.process_panel.state = ProcessState::Failed(message);
                } else {
                    self.runtime_diagnostic = Some(message);
                }
            }
        }
    }

    fn sync_process_log_editor(&self, cx: &mut Context<Self>) {
        let text = self
            .process_panel
            .output
            .iter()
            .map(|chunk| chunk.text.as_str())
            .collect::<String>();
        self.process_log_editor.update(cx, |editor, cx| {
            let buffer = editor.buffer().read(cx).as_singleton();
            if let Some(buffer) = buffer {
                buffer.update(cx, |buffer, cx| buffer.set_text(text, cx));
            }
        });
    }

    fn start_selected_process(&mut self, kind: ProcessKind, cx: &mut Context<Self>) {
        if self.process_panel.is_active() {
            return;
        }
        let Ok(project) = &self.project else {
            return;
        };
        let Some((command, target)) =
            ros_node_command(project, self.selected_node_id.as_deref(), kind)
        else {
            self.process_panel.label = kind.label().to_owned();
            self.process_panel.state = ProcessState::Failed("select a ROS node first".to_owned());
            cx.notify();
            return;
        };

        self.start_process(command, target, kind, cx);
    }

    fn select_node_for_source(&mut self, source_path: &str) -> bool {
        let Ok(project) = &self.project else {
            return false;
        };
        let Some(node) = project.nodes.iter().find(|node| {
            node.source_locations
                .iter()
                .any(|location| location.path == source_path)
        }) else {
            return false;
        };
        self.selected_node_id = Some(node.id.as_str().to_owned());
        self.inspector_view = InspectorView::Selection;
        true
    }

    fn start_source_file_process(
        &mut self,
        source_path: &str,
        kind: ProcessKind,
        cx: &mut Context<Self>,
    ) {
        if self.process_panel.is_active() {
            return;
        }
        let project = self.project.as_ref().ok();
        let Some((command, target)) = source_file_command(project, source_path, kind) else {
            self.process_panel.label = kind.label().to_owned();
            self.process_panel.state =
                ProcessState::Failed("the active file cannot be built or run".to_owned());
            cx.notify();
            return;
        };

        self.start_process(command, target, kind, cx);
    }

    fn start_process(
        &mut self,
        command: Vec<String>,
        target: String,
        kind: ProcessKind,
        cx: &mut Context<Self>,
    ) {
        self.start_runtime_discovery(cx);
        let Some(sender) = &self.runtime_commands else {
            self.process_panel.label = format!("{} {target}", kind.label());
            self.process_panel.state = ProcessState::Failed("daemon unavailable".to_owned());
            cx.notify();
            return;
        };
        match sender.try_send(runtime_client::RuntimeCommand::Launch {
            command,
            env: BTreeMap::new(),
        }) {
            Ok(()) => {
                self.process_panel = ProcessPanel {
                    label: format!("{} {target}", kind.label()),
                    state: ProcessState::Starting,
                    output: Vec::new(),
                };
                self.sync_process_log_editor(cx);
                if kind == ProcessKind::Run && self.auto_follow_process {
                    self.set_graph_mode(GraphMode::Live, cx);
                }
            }
            Err(error) => {
                self.process_panel.label = format!("{} {target}", kind.label());
                self.process_panel.state = ProcessState::Failed(error.to_string());
            }
        }
        cx.notify();
    }

    fn stop_selected_process(&mut self, cx: &mut Context<Self>) {
        let process_id = match &self.process_panel.state {
            ProcessState::Running(id) => id.clone(),
            _ => return,
        };
        let Some(sender) = &self.runtime_commands else {
            self.process_panel.state = ProcessState::Failed("daemon unavailable".to_owned());
            cx.notify();
            return;
        };
        match sender.try_send(runtime_client::RuntimeCommand::StopProcess {
            id: process_id.clone(),
        }) {
            Ok(()) => {
                self.process_panel.state = ProcessState::Stopping(process_id);
                if self.auto_follow_process {
                    self.set_graph_mode(GraphMode::Design, cx);
                }
            }
            Err(error) => self.process_panel.state = ProcessState::Failed(error.to_string()),
        }
        cx.notify();
    }

    fn show_canvas_context_menu(
        &mut self,
        position: Point<Pixels>,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let context_menu = ContextMenu::build(window, cx, |menu, _, _| {
            menu.context(self.focus_handle.clone())
                .action("New ROS Node", Box::new(CreateNode))
        });
        window.focus(&context_menu.focus_handle(cx), cx);
        let subscription = cx.subscribe(&context_menu, |this, _, _: &gpui::DismissEvent, cx| {
            this.context_menu = None;
            cx.notify();
        });
        self.context_menu = Some((context_menu, position, subscription));
        cx.notify();
    }

    fn visible_graph_center(&self) -> (f32, f32) {
        let Some(canvas_bounds) = self.canvas_bounds else {
            return (DEFAULT_VIEWPORT_CENTER_X, DEFAULT_VIEWPORT_CENTER_Y);
        };
        let zoom_scale = f32::from(self.zoom_percent) / 100.0;
        (
            (canvas_bounds.size.width.as_f32() / 2.0 - self.camera_offset_x) / zoom_scale,
            (canvas_bounds.size.height.as_f32() / 2.0 - self.camera_offset_y) / zoom_scale,
        )
    }

    fn apply_drag_position(&mut self, mouse_x: f32, mouse_y: f32, zoom_scale: f32) -> bool {
        if let Some(node_drag) = self.node_drag.clone() {
            let horizontal_delta = (mouse_x - node_drag.mouse_start_x) / zoom_scale;
            let vertical_delta = (mouse_y - node_drag.mouse_start_y) / zoom_scale;
            let Some(layout) = self
                .node_layouts
                .iter_mut()
                .find(|layout| layout.node_id == node_drag.node_id)
            else {
                return false;
            };

            layout.x = node_drag.node_start_x + horizontal_delta;
            layout.y = node_drag.node_start_y + vertical_delta;
            true
        } else if let Some(canvas_pan) = self.canvas_pan {
            let horizontal_delta = mouse_x - canvas_pan.mouse_start_x;
            let vertical_delta = mouse_y - canvas_pan.mouse_start_y;
            self.camera_offset_x = canvas_pan.camera_start_x + horizontal_delta;
            self.camera_offset_y = canvas_pan.camera_start_y + vertical_delta;
            true
        } else {
            false
        }
    }

    fn set_zoom_percent(
        &mut self,
        zoom_percent: u16,
        zoom_center: Option<gpui::Point<Pixels>>,
    ) -> bool {
        let zoom_percent = zoom_percent.clamp(MINIMUM_ZOOM_PERCENT, MAXIMUM_ZOOM_PERCENT);
        if zoom_percent == self.zoom_percent {
            return false;
        }

        if let Some((zoom_center, canvas_bounds)) = zoom_center.zip(self.canvas_bounds) {
            let old_zoom_scale = f32::from(self.zoom_percent) / 100.0;
            let new_zoom_scale = f32::from(zoom_percent) / 100.0;
            let local_center_x = (zoom_center.x - canvas_bounds.origin.x).as_f32();
            let local_center_y = (zoom_center.y - canvas_bounds.origin.y).as_f32();
            let graph_center_x = (local_center_x - self.camera_offset_x) / old_zoom_scale;
            let graph_center_y = (local_center_y - self.camera_offset_y) / old_zoom_scale;

            self.camera_offset_x = local_center_x - graph_center_x * new_zoom_scale;
            self.camera_offset_y = local_center_y - graph_center_y * new_zoom_scale;
        }

        self.zoom_percent = zoom_percent;
        true
    }

    fn handle_scroll_wheel(
        &mut self,
        event: &ScrollWheelEvent,
        _window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let vertical_delta = match event.delta {
            ScrollDelta::Pixels(delta) => delta.y.as_f32(),
            ScrollDelta::Lines(delta) => delta.y,
        };
        if vertical_delta == 0.0 {
            return;
        }

        let step = if event.delta.precise() {
            (vertical_delta.abs() * 0.2)
                .round()
                .clamp(1.0, f32::from(WHEEL_ZOOM_STEP_PERCENT)) as u16
        } else {
            WHEEL_ZOOM_STEP_PERCENT
        };
        let zoom_percent = if vertical_delta > 0.0 {
            self.zoom_percent.saturating_add(step)
        } else {
            self.zoom_percent.saturating_sub(step)
        };

        if self.set_zoom_percent(zoom_percent, Some(event.position)) {
            cx.notify();
        }
        cx.stop_propagation();
    }

    fn open_source_location(
        &self,
        source_location: SourceLocation,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let Some(workspace_root) = self
            .project
            .as_ref()
            .ok()
            .map(|project| PathBuf::from(&project.root_path))
        else {
            return;
        };
        let source_path = workspace_root.join(&source_location.path);
        let workspace = self.workspace.clone();
        let point = TextPoint::new(source_location.line, source_location.column);

        window
            .spawn(cx, async move |cx| {
                let item = workspace
                    .update_in(cx, |workspace, window, cx| {
                        workspace.open_abs_path(
                            source_path,
                            workspace::OpenOptions {
                                focus: Some(true),
                                ..Default::default()
                            },
                            window,
                            cx,
                        )
                    })?
                    .await?;
                let Some(editor) = item.downcast::<Editor>() else {
                    return anyhow::Ok(());
                };

                editor.update_in(cx, |editor, window, cx| {
                    editor.change_selections(
                        SelectionEffects::scroll(Autoscroll::center()),
                        window,
                        cx,
                        |selections| selections.select_ranges([point..point]),
                    );
                })?;

                anyhow::Ok(())
            })
            .detach_and_log_err(cx);
    }
}

#[cfg(test)]
fn load_fixture_project() -> Result<Project, serde_json::Error> {
    serde_json::from_str(FIXTURE_GRAPH)
}

fn merge_graph_layouts(
    existing_layouts: &[GraphNodeLayout],
    project: &Project,
    edges: &[GraphEdge],
    viewport_center: (f32, f32),
) -> Vec<GraphNodeLayout> {
    let generated_layouts = graph_layout(project, edges);
    if existing_layouts.is_empty() {
        return center_layouts(generated_layouts, viewport_center);
    }

    let current_node_ids = project
        .nodes
        .iter()
        .map(|node| node.id.as_str())
        .collect::<BTreeSet<_>>();
    let mut layouts = existing_layouts
        .iter()
        .filter(|layout| current_node_ids.contains(layout.node_id.as_str()))
        .cloned()
        .collect::<Vec<_>>();
    let new_node_ids = generated_layouts
        .iter()
        .filter(|layout| {
            !layouts
                .iter()
                .any(|existing_layout| existing_layout.node_id == layout.node_id)
        })
        .map(|layout| layout.node_id.clone())
        .collect::<Vec<_>>();
    let unconnected_node_ids = new_node_ids
        .iter()
        .filter(|node_id| connected_candidate(node_id, edges, &layouts).is_none())
        .cloned()
        .collect::<Vec<_>>();
    let unconnected_count = unconnected_node_ids.len();
    let mut unconnected_index = 0_usize;

    for node_id in new_node_ids {
        let preferred_position =
            connected_candidate(&node_id, edges, &layouts).unwrap_or_else(|| {
                let x = viewport_center.0 - NODE_CARD_WIDTH / 2.0
                    + centered_slot_offset(unconnected_index, unconnected_count)
                        * (NODE_CARD_WIDTH + NEW_NODE_HORIZONTAL_GAP);
                unconnected_index += 1;
                (x, viewport_center.1 - NODE_CARD_HEIGHT / 2.0)
            });
        let (x, y) = nearest_available_position(preferred_position, &layouts);
        layouts.push(GraphNodeLayout { node_id, x, y });
    }

    layouts
}

fn center_layouts(
    mut layouts: Vec<GraphNodeLayout>,
    viewport_center: (f32, f32),
) -> Vec<GraphNodeLayout> {
    let Some(first_layout) = layouts.first() else {
        return layouts;
    };
    let mut minimum_x = first_layout.x;
    let mut minimum_y = first_layout.y;
    let mut maximum_x = first_layout.x + NODE_CARD_WIDTH;
    let mut maximum_y = first_layout.y + NODE_CARD_HEIGHT;

    for layout in layouts.iter().skip(1) {
        minimum_x = minimum_x.min(layout.x);
        minimum_y = minimum_y.min(layout.y);
        maximum_x = maximum_x.max(layout.x + NODE_CARD_WIDTH);
        maximum_y = maximum_y.max(layout.y + NODE_CARD_HEIGHT);
    }

    let horizontal_offset = viewport_center.0 - (minimum_x + maximum_x) / 2.0;
    let vertical_offset = viewport_center.1 - (minimum_y + maximum_y) / 2.0;
    for layout in &mut layouts {
        layout.x += horizontal_offset;
        layout.y += vertical_offset;
    }
    layouts
}

fn connected_candidate(
    node_id: &str,
    edges: &[GraphEdge],
    layouts: &[GraphNodeLayout],
) -> Option<(f32, f32)> {
    let mut candidates = Vec::new();

    for edge in edges {
        if edge.publisher_node_id == node_id {
            if let Some(subscriber_layout) = layouts
                .iter()
                .find(|layout| layout.node_id == edge.subscriber_node_id)
            {
                candidates.push((
                    subscriber_layout.x - NODE_CARD_WIDTH - NEW_NODE_HORIZONTAL_GAP,
                    subscriber_layout.y,
                ));
            }
        } else if edge.subscriber_node_id == node_id
            && let Some(publisher_layout) = layouts
                .iter()
                .find(|layout| layout.node_id == edge.publisher_node_id)
        {
            candidates.push((
                publisher_layout.x + NODE_CARD_WIDTH + NEW_NODE_HORIZONTAL_GAP,
                publisher_layout.y,
            ));
        }
    }

    (!candidates.is_empty()).then(|| {
        let candidate_count = candidates.len() as f32;
        let (total_x, total_y) = candidates
            .into_iter()
            .fold((0.0, 0.0), |(total_x, total_y), (x, y)| {
                (total_x + x, total_y + y)
            });
        (total_x / candidate_count, total_y / candidate_count)
    })
}

fn centered_slot_offset(index: usize, count: usize) -> f32 {
    index as f32 - count.saturating_sub(1) as f32 / 2.0
}

fn nearest_available_position(
    preferred_position: (f32, f32),
    layouts: &[GraphNodeLayout],
) -> (f32, f32) {
    let horizontal_step = NODE_CARD_WIDTH + NEW_NODE_HORIZONTAL_GAP;
    let vertical_step = NODE_CARD_HEIGHT + NEW_NODE_VERTICAL_GAP;

    for row in 0..8 {
        let y = preferred_position.1 + row as f32 * vertical_step;
        for column in 0..8 {
            let offsets = if column == 0 {
                [0.0, 0.0]
            } else {
                [column as f32, -(column as f32)]
            };
            for offset in offsets {
                let candidate = (preferred_position.0 + offset * horizontal_step, y);
                if layouts
                    .iter()
                    .all(|layout| !positions_overlap(candidate, layout))
                {
                    return candidate;
                }
            }
        }
    }

    preferred_position
}

fn positions_overlap(position: (f32, f32), layout: &GraphNodeLayout) -> bool {
    (position.0 - layout.x).abs() < NODE_CARD_WIDTH + NEW_NODE_HORIZONTAL_GAP / 2.0
        && (position.1 - layout.y).abs() < NODE_CARD_HEIGHT + NEW_NODE_VERTICAL_GAP / 2.0
}

fn graph_edges(project: &Project) -> Vec<GraphEdge> {
    let mut edges = Vec::new();

    for publisher_node in &project.nodes {
        for publisher in publisher_node
            .endpoints
            .iter()
            .filter(|endpoint| endpoint.kind == EndpointKind::Publisher)
        {
            for subscriber_node in &project.nodes {
                for subscriber in subscriber_node.endpoints.iter().filter(|endpoint| {
                    endpoint.kind == EndpointKind::Subscription
                        && endpoint.name == publisher.name
                        && canonical_ros_type_name(&endpoint.type_name)
                            == canonical_ros_type_name(&publisher.type_name)
                }) {
                    edges.push(GraphEdge {
                        publisher_node_id: publisher_node.id.as_str().to_owned(),
                        subscriber_node_id: subscriber_node.id.as_str().to_owned(),
                        topic: subscriber.name.clone(),
                        type_name: subscriber.type_name.clone(),
                    });
                }
            }
        }
    }

    edges.sort();
    edges
}

fn topic_count(project: &Project) -> usize {
    project
        .nodes
        .iter()
        .flat_map(|node| &node.endpoints)
        .filter(|endpoint| {
            matches!(
                endpoint.kind,
                EndpointKind::Publisher | EndpointKind::Subscription
            )
        })
        .map(|endpoint| endpoint.name.as_str())
        .collect::<BTreeSet<_>>()
        .len()
}

fn project_topics(project: &Project) -> Vec<(String, String)> {
    project
        .nodes
        .iter()
        .flat_map(|node| &node.endpoints)
        .filter(|endpoint| {
            matches!(
                endpoint.kind,
                EndpointKind::Publisher | EndpointKind::Subscription
            )
        })
        .map(|endpoint| (endpoint.name.clone(), endpoint.type_name.clone()))
        .collect::<BTreeMap<_, _>>()
        .into_iter()
        .collect()
}

fn graph_layout(project: &Project, edges: &[GraphEdge]) -> Vec<GraphNodeLayout> {
    let mut incoming_edge_counts = project
        .nodes
        .iter()
        .map(|node| (node.id.as_str().to_owned(), 0_usize))
        .collect::<BTreeMap<_, _>>();
    let mut outgoing_nodes = BTreeMap::<String, Vec<String>>::new();

    for edge in edges {
        if edge.publisher_node_id == edge.subscriber_node_id {
            continue;
        }

        if let Some(incoming_edge_count) = incoming_edge_counts.get_mut(&edge.subscriber_node_id) {
            *incoming_edge_count += 1;
        }
        outgoing_nodes
            .entry(edge.publisher_node_id.clone())
            .or_default()
            .push(edge.subscriber_node_id.clone());
    }

    let names_by_id = project
        .nodes
        .iter()
        .map(|node| (node.id.as_str().to_owned(), node.logical_name.clone()))
        .collect::<BTreeMap<_, _>>();
    let mut ready_nodes = incoming_edge_counts
        .iter()
        .filter(|(_, incoming_edge_count)| **incoming_edge_count == 0)
        .filter_map(|(node_id, _)| {
            names_by_id
                .get(node_id)
                .map(|name| (name.clone(), node_id.clone()))
        })
        .collect::<BTreeSet<_>>();
    let mut ordered_node_ids = Vec::with_capacity(project.nodes.len());

    while let Some((_, node_id)) = ready_nodes.pop_first() {
        ordered_node_ids.push(node_id.clone());

        if let Some(subscriber_node_ids) = outgoing_nodes.get(&node_id) {
            for subscriber_node_id in subscriber_node_ids {
                let Some(incoming_edge_count) = incoming_edge_counts.get_mut(subscriber_node_id)
                else {
                    continue;
                };

                *incoming_edge_count = incoming_edge_count.saturating_sub(1);
                if *incoming_edge_count == 0
                    && let Some(name) = names_by_id.get(subscriber_node_id)
                {
                    ready_nodes.insert((name.clone(), subscriber_node_id.clone()));
                }
            }
        }
    }

    let ordered_node_id_set = ordered_node_ids
        .iter()
        .map(String::as_str)
        .collect::<BTreeSet<_>>();
    let mut remaining_nodes = project
        .nodes
        .iter()
        .filter(|node| !ordered_node_id_set.contains(node.id.as_str()))
        .map(|node| (node.logical_name.clone(), node.id.as_str().to_owned()))
        .collect::<Vec<_>>();
    remaining_nodes.sort();
    ordered_node_ids.extend(
        remaining_nodes
            .into_iter()
            .map(|(_, remaining_node_id)| remaining_node_id),
    );

    ordered_node_ids
        .into_iter()
        .enumerate()
        .map(|(index, node_id)| {
            let row = index / 2;
            let slot = index % 2;
            let left_x = LEFT_COLUMN_X + row as f32 * LEFT_COLUMN_OFFSET_X;
            let right_x = RIGHT_COLUMN_X + row as f32 * RIGHT_COLUMN_OFFSET_X;
            let x = if row.is_multiple_of(2) {
                if slot == 0 { left_x } else { right_x }
            } else if slot == 0 {
                right_x
            } else {
                left_x
            };

            GraphNodeLayout {
                node_id,
                x,
                y: GRAPH_TOP_PADDING + row as f32 * (NODE_CARD_HEIGHT + GRAPH_ROW_GAP),
            }
        })
        .collect()
}

#[cfg(test)]
fn graph_height(node_count: usize) -> f32 {
    let row_count = node_count.max(1).div_ceil(2);
    (GRAPH_TOP_PADDING
        + row_count as f32 * NODE_CARD_HEIGHT
        + row_count.saturating_sub(1) as f32 * GRAPH_ROW_GAP
        + GRAPH_BOTTOM_PADDING)
        .max(GRAPH_MINIMUM_HEIGHT)
}

fn edge_geometry(edge: &GraphEdge, node_layouts: &[GraphNodeLayout]) -> Option<GraphEdgeGeometry> {
    let publisher_layout = node_layouts
        .iter()
        .find(|layout| layout.node_id == edge.publisher_node_id)?;
    let subscriber_layout = node_layouts
        .iter()
        .find(|layout| layout.node_id == edge.subscriber_node_id)?;
    let publisher_center_x = publisher_layout.x + NODE_CARD_WIDTH / 2.0;
    let publisher_center_y = publisher_layout.y + NODE_CARD_HEIGHT / 2.0;
    let subscriber_center_x = subscriber_layout.x + NODE_CARD_WIDTH / 2.0;
    let subscriber_center_y = subscriber_layout.y + NODE_CARD_HEIGHT / 2.0;
    let horizontal_distance = subscriber_center_x - publisher_center_x;
    let vertical_distance = subscriber_center_y - publisher_center_y;

    let (start_x, start_y, end_x, end_y) = if horizontal_distance.abs() >= vertical_distance.abs() {
        let direction = horizontal_distance.signum();
        (
            publisher_center_x + direction * NODE_CARD_WIDTH / 2.0,
            publisher_center_y,
            subscriber_center_x - direction * NODE_CARD_WIDTH / 2.0,
            subscriber_center_y,
        )
    } else {
        let direction = vertical_distance.signum();
        (
            publisher_center_x,
            publisher_center_y + direction * NODE_CARD_HEIGHT / 2.0,
            subscriber_center_x,
            subscriber_center_y - direction * NODE_CARD_HEIGHT / 2.0,
        )
    };

    let is_horizontal = horizontal_distance.abs() >= vertical_distance.abs();
    let (label_x, label_y) = if is_horizontal {
        (
            (start_x + end_x) / 2.0,
            if end_x >= start_x {
                (start_y + end_y) / 2.0 - EDGE_LABEL_OFFSET
            } else {
                (start_y + end_y) / 2.0 + EDGE_LABEL_OFFSET
            },
        )
    } else {
        (
            (start_x + end_x) / 2.0 + EDGE_LABEL_OFFSET,
            (start_y + end_y) / 2.0,
        )
    };

    Some(GraphEdgeGeometry {
        start_x,
        start_y,
        end_x,
        end_y,
        label_x,
        label_y,
    })
}

fn endpoint_kind_label(kind: EndpointKind) -> &'static str {
    match kind {
        EndpointKind::Publisher => "Publisher",
        EndpointKind::Subscription => "Subscription",
        EndpointKind::ServiceClient => "Service client",
        EndpointKind::ServiceServer => "Service server",
        EndpointKind::ActionClient => "Action client",
        EndpointKind::ActionServer => "Action server",
    }
}

fn confidence_label(confidence: Confidence) -> &'static str {
    match confidence {
        Confidence::ConfirmedSource => "Confirmed by source",
        Confidence::Inferred => "Inferred",
        Confidence::RuntimeConfirmed => "Runtime confirmed",
        Confidence::Conflict => "Conflict",
        Confidence::Unknown => "Unknown",
    }
}

fn runtime_state_label(runtime_state: RuntimeState) -> &'static str {
    match runtime_state {
        RuntimeState::Unknown => "Design only",
        RuntimeState::Online => "Online",
        RuntimeState::Offline => "Not detected in the current ROS graph",
        RuntimeState::RuntimeOnly => "Runtime only",
        RuntimeState::Conflict => "Conflict",
    }
}

fn runtime_state_badge(runtime_state: RuntimeState) -> (&'static str, Color) {
    match runtime_state {
        RuntimeState::Unknown => ("DESIGN", Color::Muted),
        RuntimeState::Online => ("LIVE", Color::Success),
        RuntimeState::Offline => ("NOT SEEN", Color::Warning),
        RuntimeState::RuntimeOnly => ("RUNTIME", Color::Info),
        RuntimeState::Conflict => ("CONFLICT", Color::Conflict),
    }
}

fn runtime_status_label(status: &RuntimeStatus) -> &'static str {
    match status {
        RuntimeStatus::NotStarted => "NOT CONNECTED",
        RuntimeStatus::Connecting => "CONNECTING",
        RuntimeStatus::Ready => "CONNECTED",
        RuntimeStatus::Failed(_) => "UNAVAILABLE",
    }
}

fn source_location_label(source_location: &SourceLocation) -> String {
    format!(
        "{}:{}:{}",
        source_location.path,
        source_location.line.saturating_add(1),
        source_location.column.saturating_add(1),
    )
}

fn node_subtitle(package_name: &str, executable: &str) -> String {
    if executable.is_empty() {
        package_name.to_owned()
    } else {
        format!("{package_name} · {executable}")
    }
}

fn ros_node_command(
    project: &Project,
    selected_node_id: Option<&str>,
    kind: ProcessKind,
) -> Option<(Vec<String>, String)> {
    let node = project
        .nodes
        .iter()
        .find(|node| Some(node.id.as_str()) == selected_node_id)?;
    if node.executable.is_empty() {
        return None;
    }
    let package = project
        .packages
        .iter()
        .find(|package| package.id == node.package_id)?;
    let is_rust_node = node
        .source_locations
        .iter()
        .any(|location| location.path.ends_with(".rs"));
    let command = if is_rust_node {
        let manifest_path = if package.path.is_empty() {
            "Cargo.toml".to_owned()
        } else {
            format!("{}/Cargo.toml", package.path.trim_end_matches('/'))
        };
        let cargo_action = match kind {
            ProcessKind::Build => "check",
            ProcessKind::Run => "run",
        };
        vec![
            "cargo".to_owned(),
            cargo_action.to_owned(),
            "--manifest-path".to_owned(),
            manifest_path,
            "--bin".to_owned(),
            node.executable.clone(),
        ]
    } else {
        match kind {
            ProcessKind::Build => vec![
                "colcon".to_owned(),
                "build".to_owned(),
                "--packages-select".to_owned(),
                package.name.clone(),
            ],
            ProcessKind::Run => vec![
                "bash".to_owned(),
                "-lc".to_owned(),
                "colcon build --packages-select \"$1\" && . install/setup.bash && exec ros2 run \"$1\" \"$2\""
                    .to_owned(),
                "ros-studio".to_owned(),
                package.name.clone(),
                node.executable.clone(),
            ],
        }
    };

    Some((command, node.logical_name.clone()))
}

fn source_file_command(
    project: Option<&Project>,
    source_path: &str,
    kind: ProcessKind,
) -> Option<(Vec<String>, String)> {
    let path = std::path::Path::new(source_path);
    let extension = path.extension()?.to_str()?;
    let file_name = path.file_name()?.to_str()?.to_owned();
    let file_stem = path.file_stem()?.to_str()?;
    let output_name = file_stem
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || character == '_' {
                character
            } else {
                '_'
            }
        })
        .collect::<String>();
    let output_path = format!("/tmp/ros-studio-{output_name}");

    let command = match extension {
        "py" => match kind {
            ProcessKind::Build => vec![
                "python3".to_owned(),
                "-m".to_owned(),
                "py_compile".to_owned(),
                source_path.to_owned(),
            ],
            ProcessKind::Run => vec!["python3".to_owned(), source_path.to_owned()],
        },
        "c" | "cc" | "cpp" | "cxx" => {
            let compiler = if extension == "c" { "cc" } else { "c++" };
            match kind {
                ProcessKind::Build => vec![
                    compiler.to_owned(),
                    "-fsyntax-only".to_owned(),
                    source_path.to_owned(),
                ],
                ProcessKind::Run => vec![
                    "bash".to_owned(),
                    "-lc".to_owned(),
                    "\"$1\" \"$2\" -o \"$3\" && exec \"$3\"".to_owned(),
                    "ros-studio".to_owned(),
                    compiler.to_owned(),
                    source_path.to_owned(),
                    output_path,
                ],
            }
        }
        "rs" => {
            let package = project.and_then(|project| {
                project
                    .packages
                    .iter()
                    .filter(|package| {
                        package.path.is_empty()
                            || source_path.starts_with(&format!("{}/", package.path))
                    })
                    .max_by_key(|package| package.path.len())
            });
            if let Some(package) = package {
                let executable = project
                    .and_then(|project| {
                        project.nodes.iter().find(|node| {
                            node.package_id == package.id
                                && node
                                    .source_locations
                                    .iter()
                                    .any(|location| location.path == source_path)
                        })
                    })
                    .map(|node| node.executable.as_str())
                    .filter(|executable| !executable.is_empty())
                    .unwrap_or(file_stem);
                let manifest_path = if package.path.is_empty() {
                    "Cargo.toml".to_owned()
                } else {
                    format!("{}/Cargo.toml", package.path.trim_end_matches('/'))
                };
                vec![
                    "cargo".to_owned(),
                    match kind {
                        ProcessKind::Build => "check",
                        ProcessKind::Run => "run",
                    }
                    .to_owned(),
                    "--manifest-path".to_owned(),
                    manifest_path,
                    "--bin".to_owned(),
                    executable.to_owned(),
                ]
            } else {
                match kind {
                    ProcessKind::Build => vec![
                        "rustc".to_owned(),
                        "--emit=metadata".to_owned(),
                        source_path.to_owned(),
                        "-o".to_owned(),
                        output_path,
                    ],
                    ProcessKind::Run => vec![
                        "bash".to_owned(),
                        "-lc".to_owned(),
                        "rustc \"$1\" -o \"$2\" && exec \"$2\"".to_owned(),
                        "ros-studio".to_owned(),
                        source_path.to_owned(),
                        output_path,
                    ],
                }
            }
        }
        _ => return None,
    };

    Some((command, file_name))
}

fn render_ros_inspector(
    project: &Project,
    selected_node_id: Option<&str>,
    mode: GraphMode,
    inspector_view: InspectorView,
    cx: &mut Context<RosGraph>,
) -> Div {
    let panel = v_flex()
        .h_full()
        .w(px(320.0))
        .flex_none()
        .p_3()
        .gap_3()
        .overflow_hidden()
        .border_1()
        .border_color(cx.theme().colors().border_variant)
        .rounded_md()
        .bg(cx.theme().colors().surface_background)
        .child(
            h_flex()
                .justify_between()
                .child(Headline::new("Inspector").size(HeadlineSize::Small))
                .child(
                    Label::new(mode.label())
                        .size(LabelSize::XSmall)
                        .color(Color::Accent),
                ),
        );

    match inspector_view {
        InspectorView::Packages => {
            return panel
                .child(
                    Label::new(format!("{} packages", project.packages.len()))
                        .size(LabelSize::Small),
                )
                .children(project.packages.iter().map(|package| {
                    v_flex()
                        .gap_1()
                        .p_2()
                        .border_1()
                        .border_color(cx.theme().colors().border_variant)
                        .rounded_sm()
                        .child(Label::new(package.name.clone()).size(LabelSize::Small))
                        .child(
                            Label::new(package.path.clone())
                                .size(LabelSize::XSmall)
                                .color(Color::Muted),
                        )
                }));
        }
        InspectorView::Nodes => {
            return panel
                .child(Label::new(format!("{} nodes", project.nodes.len())).size(LabelSize::Small))
                .children(project.nodes.iter().enumerate().map(|(index, node)| {
                    let node_id = node.id.as_str().to_owned();
                    Button::new(
                        ("ros-inspector-node-list-entry", index),
                        node.logical_name.clone(),
                    )
                    .full_width()
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.selected_node_id = Some(node_id.clone());
                        this.inspector_view = InspectorView::Selection;
                        cx.notify();
                    }))
                }));
        }
        InspectorView::Topics => {
            let topics = project_topics(project);
            return panel
                .child(Label::new(format!("{} topics", topics.len())).size(LabelSize::Small))
                .children(topics.into_iter().map(|(name, type_name)| {
                    v_flex()
                        .gap_1()
                        .p_2()
                        .border_1()
                        .border_color(cx.theme().colors().border_variant)
                        .rounded_sm()
                        .child(Label::new(name).size(LabelSize::Small))
                        .child(
                            Label::new(type_name)
                                .size(LabelSize::XSmall)
                                .color(Color::Muted)
                                .truncate_middle(),
                        )
                }));
        }
        InspectorView::Selection => {}
    }

    let Some(node) = selected_node_id.and_then(|node_id| {
        project
            .nodes
            .iter()
            .find(|node| node.id.as_str() == node_id)
    }) else {
        return panel
            .child(
                Label::new("No node selected")
                    .size(LabelSize::Small)
                    .color(Color::Muted),
            )
            .child(
                Label::new(format!(
                    "{} packages · {} nodes",
                    project.packages.len(),
                    project.nodes.len()
                ))
                .size(LabelSize::XSmall)
                .color(Color::Muted),
            )
            .child(
                Label::new("Select a node to inspect its interfaces and source locations.")
                    .size(LabelSize::Small)
                    .color(Color::Muted),
            );
    };

    let package_name = project
        .packages
        .iter()
        .find(|package| package.id == node.package_id)
        .map(|package| package.name.as_str())
        .unwrap_or(node.package_id.as_str());
    let mut panel = panel
        .child(
            v_flex()
                .gap_1()
                .child(Headline::new(node.logical_name.clone()).size(HeadlineSize::Small))
                .child(
                    Label::new(node_subtitle(package_name, &node.executable))
                        .size(LabelSize::XSmall)
                        .color(Color::Muted),
                )
                .child(
                    Label::new(runtime_state_label(node.runtime_state))
                        .size(LabelSize::XSmall)
                        .color(runtime_state_badge(node.runtime_state).1),
                ),
        )
        .child(
            v_flex()
                .gap_1()
                .border_t_1()
                .border_color(cx.theme().colors().border_variant)
                .pt_3()
                .child(
                    Label::new("SOURCE")
                        .size(LabelSize::XSmall)
                        .color(Color::Muted),
                ),
        );

    if let Some(source_location) = node.source_locations.first().cloned() {
        let source_label = source_location_label(&source_location);
        panel = panel.child(
            Button::new("ros-inspector-node-source", source_label)
                .full_width()
                .label_size(LabelSize::XSmall)
                .on_click(cx.listener(move |this, _: &ClickEvent, window, cx| {
                    this.open_source_location(source_location.clone(), window, cx);
                })),
        );
    } else {
        panel = panel.child(
            Label::new("No source location")
                .size(LabelSize::XSmall)
                .color(Color::Muted),
        );
    }

    panel = panel.child(
        h_flex()
            .justify_between()
            .border_t_1()
            .border_color(cx.theme().colors().border_variant)
            .pt_3()
            .child(
                Label::new("INTERFACES")
                    .size(LabelSize::XSmall)
                    .color(Color::Muted),
            )
            .child(
                Label::new(node.endpoints.len().to_string())
                    .size(LabelSize::XSmall)
                    .color(Color::Muted),
            ),
    );

    for (index, endpoint) in node.endpoints.iter().enumerate() {
        let source_location = endpoint.source_location.clone();
        let mut endpoint_card = v_flex()
            .id(("ros-inspector-endpoint", index))
            .gap_1()
            .p_2()
            .border_1()
            .border_color(cx.theme().colors().border_variant)
            .rounded_sm()
            .bg(cx.theme().colors().editor_background)
            .child(
                h_flex()
                    .justify_between()
                    .child(Label::new(endpoint.name.clone()).size(LabelSize::Small))
                    .child(
                        Label::new(endpoint_kind_label(endpoint.kind))
                            .size(LabelSize::XSmall)
                            .color(Color::Accent),
                    ),
            )
            .child(
                Label::new(endpoint.type_name.clone())
                    .size(LabelSize::XSmall)
                    .color(Color::Muted)
                    .truncate_middle(),
            )
            .child(
                Label::new(confidence_label(endpoint.confidence))
                    .size(LabelSize::XSmall)
                    .color(Color::Muted),
            );

        if let Some(source_location) = source_location {
            let source_label = source_location_label(&source_location);
            endpoint_card = endpoint_card.child(
                Button::new(("ros-inspector-endpoint-source", index), source_label)
                    .full_width()
                    .label_size(LabelSize::XSmall)
                    .on_click(cx.listener(move |this, _: &ClickEvent, window, cx| {
                        this.open_source_location(source_location.clone(), window, cx);
                    })),
            );
        }

        panel = panel.child(endpoint_card);
    }

    panel
}

impl EventEmitter<ItemEvent> for RosGraph {}

impl Focusable for RosGraph {
    fn focus_handle(&self, _: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl Item for RosGraph {
    type Event = ItemEvent;

    fn tab_content_text(&self, _detail: usize, _cx: &App) -> SharedString {
        "ROS Graph".into()
    }

    fn telemetry_event_text(&self) -> Option<&'static str> {
        Some("ROS Graph Opened")
    }

    fn show_toolbar(&self) -> bool {
        false
    }

    fn to_item_events(event: &Self::Event, emit: &mut dyn FnMut(ItemEvent)) {
        emit(*event);
    }

    fn deactivated(&mut self, _window: &mut Window, cx: &mut Context<Self>) {
        self.node_drag = None;
        self.canvas_pan = None;
        self.context_menu = None;
        cx.notify();
    }
}

impl Render for RosGraph {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let content = v_flex()
            .size_full()
            .track_focus(&self.focus_handle)
            .id("ros-graph-content")
            .p_4()
            .gap_3()
            .bg(cx.theme().colors().editor_background)
            .children(self.context_menu.as_ref().map(|(menu, position, _)| {
                deferred(
                    anchored()
                        .position(*position)
                        .anchor(gpui::Anchor::TopLeft)
                        .child(menu.clone()),
                )
                .with_priority(3)
            }));

        match &self.displayed_project {
            Ok(project) => {
                let selected_node_id = self.selected_node_id.clone();
                let graph_mode = self.graph_mode;
                let auto_follow_process = self.auto_follow_process;
                let runtime_hint = if graph_mode == GraphMode::Design {
                    None
                } else {
                    match &self.runtime_status {
                        RuntimeStatus::Failed(error) => Some(format!("Live unavailable: {error}")),
                        _ => self.runtime_diagnostic.clone(),
                    }
                };
                let zoom_percent = self.zoom_percent;
                let zoom_scale = f32::from(zoom_percent) / 100.0;
                let edges = graph_edges(project);
                let node_layouts = self.node_layouts.clone();
                let edge_geometries = edges
                    .iter()
                    .filter_map(|edge| {
                        edge_geometry(edge, &node_layouts).map(|geometry| (edge.clone(), geometry))
                    })
                    .collect::<Vec<_>>();
                let node_width = px(NODE_CARD_WIDTH * zoom_scale);
                let node_height = px(NODE_CARD_HEIGHT * zoom_scale);
                let camera_offset_x = self.camera_offset_x;
                let camera_offset_y = self.camera_offset_y;
                let show_ports = zoom_percent >= 100;
                let show_edge_types = zoom_percent >= 100;
                let edge_label_width = if show_edge_types {
                    EDGE_LABEL_WIDTH
                } else {
                    160.0
                };
                let edge_color = cx.theme().colors().text_accent.opacity(0.8);
                let grid_dot_color = cx.theme().colors().text_muted.opacity(0.35);
                let edge_label_background = cx.theme().colors().surface_background;
                let edge_label_border = cx.theme().colors().border_variant;
                let node_background = cx.theme().colors().surface_background;
                let node_hover_background = cx.theme().colors().element_hover.opacity(0.55);
                let port_color = cx.theme().colors().icon_accent;
                let graph_entity = cx.entity();
                let selected_rust_node =
                    ros_node_command(project, selected_node_id.as_deref(), ProcessKind::Build)
                        .is_some();
                let process_is_active = self.process_panel.is_active();
                let process_can_stop =
                    matches!(&self.process_panel.state, ProcessState::Running(_));
                let process_status = self.process_panel.status_label();

                content
                    .child(
                        h_flex()
                            .flex_none()
                            .justify_between()
                            .child(
                                v_flex()
                                    .gap_1()
                                    .child(
                                        h_flex()
                                            .gap_2()
                                            .child(
                                                Headline::new("ROS Graph")
                                                    .size(HeadlineSize::Large),
                                            )
                                            .child(
                                                Label::new(graph_mode.label())
                                                    .size(LabelSize::XSmall)
                                                    .color(Color::Accent),
                                            )
                                            .when(graph_mode != GraphMode::Design, |header| {
                                                header.child(
                                                    Label::new(runtime_status_label(
                                                        &self.runtime_status,
                                                    ))
                                                    .size(LabelSize::XSmall)
                                                    .color(Color::Muted),
                                                )
                                            })
                                            .when(self.is_scanning, |header| {
                                                header.child(
                                                    Label::new("SCANNING")
                                                        .size(LabelSize::XSmall)
                                                        .color(Color::Muted),
                                                )
                                            }),
                                    )
                                    .child(
                                        h_flex()
                                            .gap_1()
                                            .child(
                                                Label::new(project.name.clone())
                                                    .size(LabelSize::Small)
                                                    .color(Color::Muted),
                                            )
                                            .child(
                                                Label::new("·")
                                                    .size(LabelSize::Small)
                                                    .color(Color::Muted),
                                            )
                                            .child(
                                                Button::new(
                                                    "ros-graph-show-packages",
                                                    format!("{} packages", project.packages.len()),
                                                )
                                                .label_size(LabelSize::Small)
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.inspector_view = InspectorView::Packages;
                                                    cx.notify();
                                                })),
                                            )
                                            .child(
                                                Label::new("·")
                                                    .size(LabelSize::Small)
                                                    .color(Color::Muted),
                                            )
                                            .child(
                                                Button::new(
                                                    "ros-graph-show-nodes",
                                                    format!("{} nodes", project.nodes.len()),
                                                )
                                                .label_size(LabelSize::Small)
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.inspector_view = InspectorView::Nodes;
                                                    cx.notify();
                                                })),
                                            )
                                            .child(
                                                Label::new("·")
                                                    .size(LabelSize::Small)
                                                    .color(Color::Muted),
                                            )
                                            .child(
                                                Button::new(
                                                    "ros-graph-show-topics",
                                                    format!("{} topics", topic_count(project)),
                                                )
                                                .label_size(LabelSize::Small)
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.inspector_view = InspectorView::Topics;
                                                    cx.notify();
                                                })),
                                            ),
                                    )
                                    .when_some(runtime_hint, |header, hint| {
                                        header.child(
                                            Label::new(hint)
                                                .size(LabelSize::XSmall)
                                                .color(Color::Warning)
                                                .truncate_middle(),
                                        )
                                    }),
                            )
                            .child(
                                h_flex()
                                    .gap_3()
                                    .child(
                                        Button::new("ros-graph-create-node", "Create node")
                                            .on_click(cx.listener(|_, _, window, cx| {
                                                window.dispatch_action(Box::new(CreateNode), cx);
                                            })),
                                    )
                                    .child(
                                        h_flex()
                                            .gap_1()
                                            .child(
                                                Button::new("ros-graph-build-node", "Build")
                                                    .disabled(
                                                        !selected_rust_node || process_is_active,
                                                    )
                                                    .on_click(|_, window, cx| {
                                                        window.dispatch_action(
                                                            Box::new(BuildNode),
                                                            cx,
                                                        );
                                                    }),
                                            )
                                            .child(
                                                Button::new("ros-graph-run-node", "Run")
                                                    .disabled(
                                                        !selected_rust_node || process_is_active,
                                                    )
                                                    .on_click(|_, window, cx| {
                                                        window.dispatch_action(Box::new(RunNode), cx);
                                                    }),
                                            )
                                            .child(
                                                Button::new("ros-graph-stop-node", "Stop")
                                                    .disabled(!process_can_stop)
                                                    .on_click(|_, window, cx| {
                                                        window.dispatch_action(
                                                            Box::new(StopNode),
                                                            cx,
                                                        );
                                                    }),
                                            ),
                                    )
                                    .child(
                                        Button::new("ros-graph-toggle-logs", "Logs")
                                            .disabled(process_status.is_none())
                                            .on_click(|_, window, cx| {
                                                window.dispatch_action(
                                                    Box::new(ToggleProcessPanel),
                                                    cx,
                                                );
                                            }),
                                    )
                                    .child(
                                        h_flex()
                                            .gap_1()
                                            .child(
                                                Button::new("ros-graph-mode-design", "Design")
                                                    .toggle_state(graph_mode == GraphMode::Design)
                                                    .on_click(cx.listener(|this, _, _, cx| {
                                                        this.set_graph_mode(GraphMode::Design, cx);
                                                    })),
                                            )
                                            .child(
                                                Button::new("ros-graph-mode-live", "Live")
                                                    .toggle_state(graph_mode == GraphMode::Live)
                                                    .on_click(cx.listener(|this, _, _, cx| {
                                                        this.set_graph_mode(GraphMode::Live, cx);
                                                    })),
                                            )
                                            .child(
                                                Button::new("ros-graph-mode-both", "Both")
                                                    .toggle_state(graph_mode == GraphMode::Both)
                                                    .on_click(cx.listener(|this, _, _, cx| {
                                                        this.set_graph_mode(GraphMode::Both, cx);
                                                    })),
                                            ),
                                    )
                                    .child(
                                        Button::new("ros-graph-auto-follow-process", "Auto mode")
                                            .toggle_state(auto_follow_process)
                                            .tooltip(Tooltip::text(
                                                "Switch to Live on Run and Design on Stop",
                                            ))
                                            .on_click(cx.listener(|this, _, _, cx| {
                                                this.auto_follow_process =
                                                    !this.auto_follow_process;
                                                cx.notify();
                                            })),
                                    )
                                    .child(
                                        h_flex()
                                            .gap_1()
                                            .child(
                                                Button::new("ros-graph-zoom-out", "−")
                                                    .disabled(zoom_percent <= MINIMUM_ZOOM_PERCENT)
                                                    .on_click(cx.listener(|this, _, _, cx| {
                                                        this.zoom_percent = this
                                                            .zoom_percent
                                                            .saturating_sub(ZOOM_STEP_PERCENT)
                                                            .max(MINIMUM_ZOOM_PERCENT);
                                                        cx.notify();
                                                    })),
                                            )
                                            .child(
                                                Button::new(
                                                    "ros-graph-reset-zoom",
                                                    format!("{zoom_percent}%"),
                                                )
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.zoom_percent = 100;
                                                    this.camera_offset_x = 0.0;
                                                    this.camera_offset_y = 0.0;
                                                    cx.notify();
                                                })),
                                            )
                                            .child(
                                                Button::new("ros-graph-zoom-in", "+")
                                                    .disabled(zoom_percent >= MAXIMUM_ZOOM_PERCENT)
                                                    .on_click(cx.listener(|this, _, _, cx| {
                                                        this.zoom_percent = this
                                                            .zoom_percent
                                                            .saturating_add(ZOOM_STEP_PERCENT)
                                                            .min(MAXIMUM_ZOOM_PERCENT);
                                                        cx.notify();
                                                    })),
                                            ),
                                    ),
                            ),
                    )
                    .child(
                        h_flex()
                            .flex_1()
                            .min_h(px(420.0))
                            .gap_3()
                            .items_stretch()
                            .child(
                                div()
                                    .id("ros-graph-viewport")
                                    .relative()
                                    .flex_1()
                                    .overflow_hidden()
                                    .border_color(cx.theme().colors().border_variant)
                                    .border_1()
                                    .rounded_md()
                                    .child(
                                        div()
                                    .id("ros-graph-canvas")
                                    .relative()
                                    .size_full()
                                    .bg(cx.theme().colors().editor_background)
                                    .cursor_default()
                                    .when(self.canvas_pan.is_some(), |canvas| {
                                        canvas.cursor_grabbing()
                                    })
                                    .on_scroll_wheel(cx.listener(Self::handle_scroll_wheel))
                                    .on_mouse_down(
                                        MouseButton::Right,
                                        cx.listener(|this, event: &MouseDownEvent, window, cx| {
                                            cx.stop_propagation();
                                            this.show_canvas_context_menu(event.position, window, cx);
                                        }),
                                    )
                                    .on_mouse_down(
                                        MouseButton::Left,
                                        cx.listener(|this, event: &MouseDownEvent, _, cx| {
                                            this.context_menu = None;
                                            this.canvas_pan = Some(GraphCanvasPan {
                                                mouse_start_x: event.position.x.as_f32(),
                                                mouse_start_y: event.position.y.as_f32(),
                                                camera_start_x: this.camera_offset_x,
                                                camera_start_y: this.camera_offset_y,
                                            });
                                            this.node_drag = None;
                                            this.selected_node_id = None;
                                            cx.notify();
                                        }),
                                    )
                                    .on_mouse_move(cx.listener(
                                        move |this, event: &MouseMoveEvent, _, cx| {
                                            if !event.dragging() {
                                                return;
                                            }

                                            if this.apply_drag_position(
                                                event.position.x.as_f32(),
                                                event.position.y.as_f32(),
                                                zoom_scale,
                                            ) {
                                                cx.notify();
                                            }
                                        },
                                    ))
                                    .on_mouse_up(
                                        MouseButton::Left,
                                        cx.listener(move |this, event: &MouseUpEvent, _, cx| {
                                            this.apply_drag_position(
                                                event.position.x.as_f32(),
                                                event.position.y.as_f32(),
                                                zoom_scale,
                                            );
                                            this.node_drag = None;
                                            this.canvas_pan = None;
                                            cx.notify();
                                        }),
                                    )
                                    .on_mouse_up_out(
                                        MouseButton::Left,
                                        cx.listener(move |this, event: &MouseUpEvent, _, cx| {
                                            this.apply_drag_position(
                                                event.position.x.as_f32(),
                                                event.position.y.as_f32(),
                                                zoom_scale,
                                            );
                                            this.node_drag = None;
                                            this.canvas_pan = None;
                                            cx.notify();
                                        }),
                                    )
                                    .on_click(cx.listener(|this, _, _, cx| {
                                        this.selected_node_id = None;
                                        cx.notify();
                                    }))
                                    .child(
                                        canvas(
                                            move |bounds, _, cx| {
                                                graph_entity.update(cx, |this, _| {
                                                    this.canvas_bounds = Some(bounds);
                                                });
                                            },
                                            {
                                                let edge_geometries = edge_geometries.clone();
                                                move |bounds, _, window, _| {
                                                    let dot_spacing =
                                                        GRAPH_DOT_SPACING * zoom_scale;
                                                    let dot_size = px(1.0);
                                                    let mut grid_builder =
                                                        PathBuilder::stroke(px(2.0));
                                                    let mut dot_y =
                                                        camera_offset_y.rem_euclid(dot_spacing);
                                                    while dot_y < bounds.size.height.as_f32() {
                                                        let mut dot_x = camera_offset_x
                                                            .rem_euclid(dot_spacing);
                                                        while dot_x < bounds.size.width.as_f32() {
                                                            let dot_origin = point(
                                                                bounds.origin.x + px(dot_x),
                                                                bounds.origin.y + px(dot_y),
                                                            );
                                                            grid_builder.move_to(dot_origin);
                                                            grid_builder.line_to(point(
                                                                dot_origin.x + dot_size,
                                                                dot_origin.y,
                                                            ));
                                                            dot_x += dot_spacing;
                                                        }
                                                        dot_y += dot_spacing;
                                                    }
                                                    if let Ok(grid) = grid_builder.build() {
                                                        window.paint_path(grid, grid_dot_color);
                                                    }

                                                    for (_, geometry) in &edge_geometries {
                                                        let start_x = geometry.start_x * zoom_scale
                                                            + camera_offset_x;
                                                        let start_y = geometry.start_y * zoom_scale
                                                            + camera_offset_y;
                                                        let end_x = geometry.end_x * zoom_scale
                                                            + camera_offset_x;
                                                        let end_y = geometry.end_y * zoom_scale
                                                            + camera_offset_y;
                                                        let start = point(
                                                            bounds.origin.x + px(start_x),
                                                            bounds.origin.y + px(start_y),
                                                        );
                                                        let end = point(
                                                            bounds.origin.x + px(end_x),
                                                            bounds.origin.y + px(end_y),
                                                        );
                                                        let mut line_builder =
                                                            PathBuilder::stroke(px(2.0));
                                                        line_builder.move_to(start);
                                                        line_builder.line_to(end);
                                                        if let Ok(line) = line_builder.build() {
                                                            window.paint_path(line, edge_color);
                                                        }

                                                        let angle =
                                                            (end_y - start_y).atan2(end_x - start_x);
                                                        let arrow_length = 10.0 * zoom_scale;
                                                        let arrow_half_width = 5.0 * zoom_scale;
                                                        let arrow_base_x =
                                                            end_x - arrow_length * angle.cos();
                                                        let arrow_base_y =
                                                            end_y - arrow_length * angle.sin();
                                                        let perpendicular_x =
                                                            arrow_half_width * angle.sin();
                                                        let perpendicular_y =
                                                            arrow_half_width * angle.cos();
                                                        let mut arrow_builder = PathBuilder::fill();
                                                        arrow_builder.move_to(end);
                                                        arrow_builder.line_to(point(
                                                            bounds.origin.x
                                                                + px(arrow_base_x + perpendicular_x),
                                                            bounds.origin.y
                                                                + px(arrow_base_y - perpendicular_y),
                                                        ));
                                                        arrow_builder.line_to(point(
                                                            bounds.origin.x
                                                                + px(arrow_base_x - perpendicular_x),
                                                            bounds.origin.y
                                                                + px(arrow_base_y + perpendicular_y),
                                                        ));
                                                        arrow_builder.close();
                                                        if let Ok(arrow) = arrow_builder.build() {
                                                            window.paint_path(arrow, edge_color);
                                                        }
                                                    }
                                                }
                                            },
                                        )
                                        .absolute()
                                        .size_full(),
                                    )
                                    .children(edge_geometries.iter().enumerate().map(
                                        |(index, (edge, geometry))| {
                                            v_flex()
                                                .id(("ros-graph-edge-label", index))
                                                .absolute()
                                                .left(px(
                                                    geometry.label_x * zoom_scale
                                                        + camera_offset_x
                                                        - edge_label_width / 2.0,
                                                ))
                                                .top(px(
                                                    geometry.label_y * zoom_scale
                                                        + camera_offset_y
                                                        - 22.0,
                                                ))
                                                .w(px(edge_label_width))
                                                .items_center()
                                                .px_2()
                                                .py_1()
                                                .gap_0p5()
                                                .border_1()
                                                .border_color(edge_label_border)
                                                .rounded_sm()
                                                .bg(edge_label_background)
                                                .child(
                                                    Label::new(edge.topic.clone())
                                                        .size(LabelSize::Small)
                                                        .color(Color::Accent),
                                                )
                                                .when(show_edge_types, |label| {
                                                    label.child(
                                                        Label::new(edge.type_name.clone())
                                                            .size(LabelSize::XSmall)
                                                            .color(Color::Muted)
                                                            .truncate_middle(),
                                                    )
                                                })
                                        },
                                    ))
                                    .children(node_layouts.iter().enumerate().filter_map(
                                        |(index, layout)| {
                                            let node = project.nodes.iter().find(|node| {
                                                node.id.as_str() == layout.node_id
                                            })?;
                                            let node_id = node.id.as_str().to_owned();
                                            let primary_source_location =
                                                node.source_locations.first().cloned();
                                            let selected =
                                                selected_node_id.as_deref() == Some(&node_id);
                                            let package_name = project
                                                .packages
                                                .iter()
                                                .find(|package| package.id == node.package_id)
                                                .map(|package| package.name.as_str())
                                                .unwrap_or(node.package_id.as_str());
                                            let input_endpoints = node.endpoints.iter().filter(
                                                |endpoint| {
                                                    matches!(
                                                        endpoint.kind,
                                                        EndpointKind::Subscription
                                                            | EndpointKind::ServiceServer
                                                            | EndpointKind::ActionServer
                                                    )
                                                },
                                            );
                                            let output_endpoints = node.endpoints.iter().filter(
                                                |endpoint| {
                                                    matches!(
                                                        endpoint.kind,
                                                        EndpointKind::Publisher
                                                            | EndpointKind::ServiceClient
                                                            | EndpointKind::ActionClient
                                                    )
                                                },
                                            );

                                            Some(
                                                v_flex()
                                                    .id(("ros-graph-node", index))
                                                    .absolute()
                                                    .left(px(
                                                        layout.x * zoom_scale + camera_offset_x,
                                                    ))
                                                    .top(px(
                                                        layout.y * zoom_scale + camera_offset_y,
                                                    ))
                                                    .w(node_width)
                                                    .h(node_height)
                                                    .p_3()
                                                    .gap_2()
                                                    .overflow_hidden()
                                                    .border_1()
                                                    .border_color(cx.theme().colors().border)
                                                    .rounded_md()
                                                    .bg(node_background)
                                                    .cursor_default()
                                                    .hover(|style| {
                                                        style.bg(node_hover_background)
                                                    })
                                                    .when(selected, |card| {
                                                        card.border_color(
                                                            cx.theme().colors().border_selected,
                                                        )
                                                        .bg(cx.theme().colors().element_selected)
                                                    })
                                                    .on_click(cx.listener(
                                                        move |this,
                                                              event: &ClickEvent,
                                                              window,
                                                              cx| {
                                                            cx.stop_propagation();
                                                            this.selected_node_id =
                                                                Some(node_id.clone());
                                                            this.inspector_view =
                                                                InspectorView::Selection;
                                                            if event.click_count() >= 2
                                                                && let Some(source_location) =
                                                                    primary_source_location.clone()
                                                            {
                                                                this.open_source_location(
                                                                    source_location,
                                                                    window,
                                                                    cx,
                                                                );
                                                            }
                                                            cx.notify();
                                                        },
                                                    ))
                                                    .on_mouse_down(
                                                        MouseButton::Right,
                                                        cx.listener(|_, _: &MouseDownEvent, _, cx| {
                                                            cx.stop_propagation();
                                                        }),
                                                    )
                                                    .on_mouse_down(
                                                        MouseButton::Left,
                                                        cx.listener({
                                                            let node_id =
                                                                node.id.as_str().to_owned();
                                                            let node_start_x = layout.x;
                                                            let node_start_y = layout.y;
                                                            move |this,
                                                                  event: &MouseDownEvent,
                                                                  _,
                                                                  cx| {
                                                                cx.stop_propagation();
                                                                this.canvas_pan = None;
                                                                this.selected_node_id =
                                                                    Some(node_id.clone());
                                                                this.inspector_view =
                                                                    InspectorView::Selection;
                                                                this.node_drag =
                                                                    Some(GraphNodeDrag {
                                                                        node_id: node_id.clone(),
                                                                        mouse_start_x: event
                                                                            .position
                                                                            .x
                                                                            .as_f32(),
                                                                        mouse_start_y: event
                                                                            .position
                                                                            .y
                                                                            .as_f32(),
                                                                        node_start_x,
                                                                        node_start_y,
                                                                    });
                                                                cx.notify();
                                                            }
                                                        }),
                                                    )
                                                    .on_mouse_up(
                                                        MouseButton::Left,
                                                        cx.listener(move |this, event: &MouseUpEvent, _, cx| {
                                                            cx.stop_propagation();
                                                            this.apply_drag_position(
                                                                event.position.x.as_f32(),
                                                                event.position.y.as_f32(),
                                                                zoom_scale,
                                                            );
                                                            this.node_drag = None;
                                                            this.canvas_pan = None;
                                                            cx.notify();
                                                        }),
                                                    )
                                                    .on_mouse_up_out(
                                                        MouseButton::Left,
                                                        cx.listener(|this, _: &MouseUpEvent, _, cx| {
                                                            this.node_drag = None;
                                                            this.canvas_pan = None;
                                                            cx.notify();
                                                        }),
                                                    )
                                                    .child(
                                                        h_flex()
                                                            .justify_between()
                                                            .child(
                                                                Headline::new(
                                                                    node.logical_name.clone(),
                                                                )
                                                                .size(HeadlineSize::Small),
                                                            )
                                                            .child({
                                                                let (badge, color) =
                                                                    runtime_state_badge(
                                                                        node.runtime_state,
                                                                    );
                                                                Label::new(badge)
                                                                    .size(LabelSize::XSmall)
                                                                    .color(color)
                                                            }),
                                                    )
                                                    .child(
                                                        Label::new(node_subtitle(
                                                            package_name,
                                                            &node.executable,
                                                        ))
                                                        .size(LabelSize::XSmall)
                                                        .color(Color::Muted),
                                                    )
                                                    .when(show_ports, |card| {
                                                        card.child(
                                                            h_flex()
                                                                .items_start()
                                                                .gap_3()
                                                                .border_t_1()
                                                                .border_color(
                                                                    cx.theme()
                                                                        .colors()
                                                                        .border_variant,
                                                                )
                                                                .pt_2()
                                                                .child(
                                                                    v_flex()
                                                                        .flex_1()
                                                                        .gap_1()
                                                                        .child(
                                                                            Label::new("IN")
                                                                                .size(
                                                                                    LabelSize::XSmall,
                                                                                )
                                                                                .color(
                                                                                    Color::Muted,
                                                                                ),
                                                                        )
                                                                        .children(
                                                                            input_endpoints.map(
                                                                                |endpoint| {
                                                                                    h_flex()
                                                                                        .gap_1()
                                                                                        .child(
                                                                                            div()
                                                                                                .w(px(6.0))
                                                                                                .h(px(6.0))
                                                                                                .flex_none()
                                                                                                .rounded_full()
                                                                                                .bg(port_color),
                                                                                        )
                                                                                        .child(
                                                                                            Label::new(
                                                                                                endpoint
                                                                                                    .name
                                                                                                    .clone(),
                                                                                            )
                                                                                            .size(
                                                                                                LabelSize::XSmall,
                                                                                            ),
                                                                                        )
                                                                                },
                                                                            ),
                                                                        ),
                                                                )
                                                                .child(
                                                                    v_flex()
                                                                        .flex_1()
                                                                        .items_end()
                                                                        .gap_1()
                                                                        .child(
                                                                            Label::new("OUT")
                                                                                .size(
                                                                                    LabelSize::XSmall,
                                                                                )
                                                                                .color(
                                                                                    Color::Muted,
                                                                                ),
                                                                        )
                                                                        .children(
                                                                            output_endpoints.map(
                                                                                |endpoint| {
                                                                                    h_flex()
                                                                                        .gap_1()
                                                                                        .child(
                                                                                            Label::new(
                                                                                                endpoint
                                                                                                    .name
                                                                                                    .clone(),
                                                                                            )
                                                                                            .size(
                                                                                                LabelSize::XSmall,
                                                                                            ),
                                                                                        )
                                                                                        .child(
                                                                                            div()
                                                                                                .w(px(6.0))
                                                                                                .h(px(6.0))
                                                                                                .flex_none()
                                                                                                .rounded_full()
                                                                                                .bg(port_color),
                                                                                        )
                                                                                },
                                                                            ),
                                                                        ),
                                                                ),
                                                        )
                                                    }),
                                            )
                                        },
                                    )),
                                    ),
                            )
                            .child(render_ros_inspector(
                                project,
                                selected_node_id.as_deref(),
                                graph_mode,
                                self.inspector_view,
                                cx,
                            )),
                    )
            }
            Err(error) => content
                .justify_center()
                .items_center()
                .child(
                    Headline::new(if self.is_scanning {
                        "Scanning ROS workspace"
                    } else {
                        "Unable to load ROS Graph"
                    })
                    .size(HeadlineSize::Small),
                )
                .child(
                    Label::new(error.clone())
                        .size(LabelSize::Small)
                        .color(Color::Error),
                ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_fixture_project() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;

        assert_eq!(project.id.as_str(), "project:drone_demo_ws");
        assert_eq!(project.packages.len(), 4);
        assert_eq!(project.nodes.len(), 4);

        Ok(())
    }

    #[test]
    fn derives_fixture_graph_edges() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;

        assert_eq!(
            graph_edges(&project),
            [
                GraphEdge {
                    publisher_node_id: "node:camera:src/camera/src/camera.rs:node".to_owned(),
                    subscriber_node_id: "node:detector:src/detector/src/detector.rs:node"
                        .to_owned(),
                    topic: "/camera/image".to_owned(),
                    type_name: "sensor_msgs::msg::Image".to_owned(),
                },
                GraphEdge {
                    publisher_node_id: "node:detector:src/detector/src/detector.rs:node".to_owned(),
                    subscriber_node_id: "node:navigation:src/navigation/src/navigation.rs:node"
                        .to_owned(),
                    topic: "/detections".to_owned(),
                    type_name: "vision_msgs::msg::Detection2DArray".to_owned(),
                },
                GraphEdge {
                    publisher_node_id: "node:navigation:src/navigation/src/navigation.rs:node"
                        .to_owned(),
                    subscriber_node_id:
                        "node:autopilot_bridge:src/autopilot_bridge/src/autopilot_bridge.rs:node"
                            .to_owned(),
                    topic: "/cmd_vel".to_owned(),
                    type_name: "geometry_msgs::msg::Twist".to_owned(),
                },
            ]
        );

        Ok(())
    }

    #[test]
    fn lists_fixture_topics_once() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;

        assert_eq!(
            project_topics(&project)
                .into_iter()
                .map(|(name, _)| name)
                .collect::<Vec<_>>(),
            ["/camera/image", "/cmd_vel", "/detections"]
        );

        Ok(())
    }

    #[test]
    fn builds_commands_for_rust_and_non_rust_nodes() -> anyhow::Result<()> {
        let mut project = load_fixture_project()?;
        let camera_id = "node:camera:src/camera/src/camera.rs:node";

        assert_eq!(
            ros_node_command(&project, Some(camera_id), ProcessKind::Build),
            Some((
                vec![
                    "cargo".to_owned(),
                    "check".to_owned(),
                    "--manifest-path".to_owned(),
                    "src/camera/Cargo.toml".to_owned(),
                    "--bin".to_owned(),
                    "camera".to_owned(),
                ],
                "camera".to_owned(),
            ))
        );
        assert_eq!(
            ros_node_command(&project, Some(camera_id), ProcessKind::Run),
            Some((
                vec![
                    "cargo".to_owned(),
                    "run".to_owned(),
                    "--manifest-path".to_owned(),
                    "src/camera/Cargo.toml".to_owned(),
                    "--bin".to_owned(),
                    "camera".to_owned(),
                ],
                "camera".to_owned(),
            ))
        );

        let camera = project
            .nodes
            .iter_mut()
            .find(|node| node.id.as_str() == camera_id)
            .context("camera fixture node is missing")?;
        camera.source_locations[0].path = "src/camera.py".to_owned();
        assert_eq!(
            ros_node_command(&project, Some(camera_id), ProcessKind::Run),
            Some((
                vec![
                    "bash".to_owned(),
                    "-lc".to_owned(),
                    "colcon build --packages-select \"$1\" && . install/setup.bash && exec ros2 run \"$1\" \"$2\""
                        .to_owned(),
                    "ros-studio".to_owned(),
                    "camera".to_owned(),
                    "camera".to_owned(),
                ],
                "camera".to_owned(),
            ))
        );

        Ok(())
    }

    #[test]
    fn builds_commands_for_active_source_files() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;

        assert_eq!(
            source_file_command(Some(&project), "src/camera/src/camera.rs", ProcessKind::Run,),
            Some((
                vec![
                    "cargo".to_owned(),
                    "run".to_owned(),
                    "--manifest-path".to_owned(),
                    "src/camera/Cargo.toml".to_owned(),
                    "--bin".to_owned(),
                    "camera".to_owned(),
                ],
                "camera.rs".to_owned(),
            ))
        );
        assert_eq!(
            source_file_command(None, "scripts/teleop.py", ProcessKind::Build),
            Some((
                vec![
                    "python3".to_owned(),
                    "-m".to_owned(),
                    "py_compile".to_owned(),
                    "scripts/teleop.py".to_owned(),
                ],
                "teleop.py".to_owned(),
            ))
        );

        Ok(())
    }

    #[test]
    fn lays_out_fixture_in_flow_order() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;
        let edges = graph_edges(&project);
        let layouts = graph_layout(&project, &edges);

        assert_eq!(
            layouts,
            [
                GraphNodeLayout {
                    node_id: "node:camera:src/camera/src/camera.rs:node".to_owned(),
                    x: 48.0,
                    y: 64.0,
                },
                GraphNodeLayout {
                    node_id: "node:detector:src/detector/src/detector.rs:node".to_owned(),
                    x: 500.0,
                    y: 64.0,
                },
                GraphNodeLayout {
                    node_id: "node:navigation:src/navigation/src/navigation.rs:node".to_owned(),
                    x: 548.0,
                    y: 360.0,
                },
                GraphNodeLayout {
                    node_id:
                        "node:autopilot_bridge:src/autopilot_bridge/src/autopilot_bridge.rs:node"
                            .to_owned(),
                    x: 100.0,
                    y: 360.0,
                },
            ]
        );
        assert_eq!(graph_height(project.nodes.len()), 720.0);

        Ok(())
    }

    #[test]
    fn places_connected_new_node_next_to_its_family() -> anyhow::Result<()> {
        let project = load_fixture_project()?;
        let camera_id = "node:camera:src/camera/src/camera.rs:node";
        let detector_id = "node:detector:src/detector/src/detector.rs:node";
        let mut previous_project = project.clone();
        previous_project
            .nodes
            .retain(|node| node.id.as_str() != camera_id);
        let mut previous_layouts = graph_layout(&previous_project, &graph_edges(&previous_project));
        let Some(detector_layout) = previous_layouts
            .iter_mut()
            .find(|layout| layout.node_id == detector_id)
        else {
            anyhow::bail!("detector fixture layout should exist");
        };
        detector_layout.x = 600.0;
        detector_layout.y = 1_000.0;

        let layouts = merge_graph_layouts(
            &previous_layouts,
            &project,
            &graph_edges(&project),
            (600.0, 360.0),
        );
        let Some(camera_layout) = layouts.iter().find(|layout| layout.node_id == camera_id) else {
            anyhow::bail!("camera fixture layout should exist");
        };

        assert_eq!(camera_layout.x, 280.0);
        assert_eq!(camera_layout.y, 1_000.0);

        Ok(())
    }

    #[test]
    fn centers_initial_graph_in_visible_viewport() -> anyhow::Result<()> {
        let project = load_fixture_project()?;
        let layouts = merge_graph_layouts(&[], &project, &graph_edges(&project), (800.0, 500.0));
        let minimum_x = layouts
            .iter()
            .map(|layout| layout.x)
            .reduce(f32::min)
            .context("fixture layouts should not be empty")?;
        let maximum_x = layouts
            .iter()
            .map(|layout| layout.x + NODE_CARD_WIDTH)
            .reduce(f32::max)
            .context("fixture layouts should not be empty")?;
        let minimum_y = layouts
            .iter()
            .map(|layout| layout.y)
            .reduce(f32::min)
            .context("fixture layouts should not be empty")?;
        let maximum_y = layouts
            .iter()
            .map(|layout| layout.y + NODE_CARD_HEIGHT)
            .reduce(f32::max)
            .context("fixture layouts should not be empty")?;

        assert_eq!((minimum_x + maximum_x) / 2.0, 800.0);
        assert_eq!((minimum_y + maximum_y) / 2.0, 500.0);

        Ok(())
    }

    #[test]
    fn anchors_fixture_edges_at_node_borders() -> Result<(), serde_json::Error> {
        let project = load_fixture_project()?;
        let edges = graph_edges(&project);
        let layouts = graph_layout(&project, &edges);
        let geometries = edges
            .iter()
            .filter_map(|edge| edge_geometry(edge, &layouts))
            .collect::<Vec<_>>();

        assert_eq!(
            geometries,
            [
                GraphEdgeGeometry {
                    start_x: 288.0,
                    start_y: 152.0,
                    end_x: 500.0,
                    end_y: 152.0,
                    label_x: 394.0,
                    label_y: 132.0,
                },
                GraphEdgeGeometry {
                    start_x: 620.0,
                    start_y: 240.0,
                    end_x: 668.0,
                    end_y: 360.0,
                    label_x: 664.0,
                    label_y: 300.0,
                },
                GraphEdgeGeometry {
                    start_x: 548.0,
                    start_y: 448.0,
                    end_x: 340.0,
                    end_y: 448.0,
                    label_x: 444.0,
                    label_y: 468.0,
                },
            ]
        );

        Ok(())
    }

    #[test]
    fn applies_runtime_patches_without_changing_design_node_identity() -> anyhow::Result<()> {
        let design = load_fixture_project()?;
        let mut overlay = RuntimeOverlay::default();
        assert!(overlay.snapshot(&design).is_none());

        let mut live_camera = design
            .nodes
            .iter()
            .find(|node| node.logical_name == "camera")
            .context("fixture should contain a camera")?
            .clone();
        live_camera.id = EntityId::new("node:runtime:/camera");
        live_camera.logical_name = "/camera".to_owned();
        live_camera.package_id = EntityId::new("package:runtime");
        live_camera.source_locations.clear();
        live_camera.runtime_state = RuntimeState::Online;

        assert!(!overlay.apply_patch(
            GraphPatch {
                project_id: EntityId::new("project:other"),
                upsert_packages: Vec::new(),
                removed_package_ids: Vec::new(),
                upsert_nodes: vec![live_camera.clone()],
                removed_node_ids: Vec::new(),
            },
            &design.id,
        ));
        assert!(overlay.snapshot(&design).is_none());

        assert!(overlay.apply_patch(
            GraphPatch {
                project_id: design.id.clone(),
                upsert_packages: vec![Package {
                    id: EntityId::new("package:runtime"),
                    name: "Runtime".to_owned(),
                    path: String::new(),
                }],
                removed_package_ids: Vec::new(),
                upsert_nodes: vec![live_camera.clone()],
                removed_node_ids: Vec::new(),
            },
            &design.id,
        ));

        let snapshot = overlay
            .snapshot(&design)
            .context("runtime snapshot should exist")?;
        let merged = reconcile_project(&design, Some(&snapshot));
        let camera = merged
            .nodes
            .iter()
            .find(|node| node.logical_name == "camera")
            .context("merged camera should retain its design name")?;
        assert_eq!(camera.runtime_state, RuntimeState::Online);
        assert_eq!(
            camera.id.as_str(),
            "node:camera:src/camera/src/camera.rs:node"
        );
        assert_eq!(
            project_for_mode(&design, &merged, GraphMode::Design),
            design
        );
        assert_eq!(
            project_for_mode(&design, &merged, GraphMode::Live)
                .nodes
                .len(),
            1
        );
        assert_eq!(
            project_for_mode(&design, &merged, GraphMode::Both)
                .nodes
                .len(),
            4
        );

        assert!(overlay.apply_patch(
            GraphPatch {
                project_id: design.id.clone(),
                upsert_packages: Vec::new(),
                removed_package_ids: Vec::new(),
                upsert_nodes: Vec::new(),
                removed_node_ids: vec![live_camera.id],
            },
            &design.id,
        ));
        let snapshot = overlay
            .snapshot(&design)
            .context("empty runtime snapshot should exist")?;
        let merged = reconcile_project(&design, Some(&snapshot));
        assert!(
            merged
                .nodes
                .iter()
                .all(|node| node.runtime_state == RuntimeState::Offline)
        );
        assert!(
            project_for_mode(&design, &merged, GraphMode::Live)
                .nodes
                .is_empty()
        );

        Ok(())
    }

    #[test]
    fn graph_edges_match_equivalent_ros_type_spellings() -> anyhow::Result<()> {
        let mut project = load_fixture_project()?;
        let detector = project
            .nodes
            .iter_mut()
            .find(|node| node.logical_name == "detector")
            .context("fixture should contain a detector")?;
        let subscription = detector
            .endpoints
            .iter_mut()
            .find(|endpoint| endpoint.kind == EndpointKind::Subscription)
            .context("detector should subscribe to the camera image")?;
        subscription.type_name = "sensor_msgs/msg/Image".to_owned();

        assert!(graph_edges(&project).iter().any(|edge| {
            edge.topic == "/camera/image"
                && edge.publisher_node_id.contains("camera")
                && edge.subscriber_node_id.contains("detector")
        }));

        Ok(())
    }

    #[test]
    fn counts_topics_even_without_a_subscriber() -> anyhow::Result<()> {
        let mut project = load_fixture_project()?;
        assert_eq!(topic_count(&project), 3);

        let camera = project
            .nodes
            .iter_mut()
            .find(|node| node.logical_name == "camera")
            .context("fixture should contain a camera")?;
        let mut publisher = camera
            .endpoints
            .first()
            .context("camera should publish an image")?
            .clone();
        publisher.name = "/ros_studio_smoke".to_owned();
        camera.endpoints.push(publisher);

        assert_eq!(topic_count(&project), 4);
        assert_eq!(node_subtitle("Runtime", ""), "Runtime");
        assert_eq!(
            node_subtitle("camera", "camera_node"),
            "camera · camera_node"
        );

        Ok(())
    }
}
