#![forbid(unsafe_code)]

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use editor::{Editor, SelectionEffects, scroll::Autoscroll};
use gpui::{
    App, Bounds, ClickEvent, Context, Div, EventEmitter, FocusHandle, Focusable,
    InteractiveElement, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent, PathBuilder,
    Pixels, Render, ScrollDelta, ScrollWheelEvent, SharedString, TaskExt, WeakEntity, Window,
    actions, canvas, point,
};
use rope::Point as TextPoint;
use ros_studio_model::{Confidence, EndpointKind, Project, RuntimeState, SourceLocation};
use ui::{Button, Headline, HeadlineSize, Label, LabelSize, prelude::*};
use workspace::{
    Workspace,
    item::{Item, ItemEvent},
};

const FIXTURE_GRAPH: &str = include_str!("../../../examples/drone_demo_ws/expected_graph.json");
const FIXTURE_WORKSPACE_DIRECTORY: &str = "examples/drone_demo_ws";
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
const EDGE_LABEL_WIDTH: f32 = 200.0;
const EDGE_LABEL_OFFSET: f32 = 20.0;
const LEFT_COLUMN_X: f32 = 48.0;
const RIGHT_COLUMN_X: f32 = 500.0;
const LEFT_COLUMN_OFFSET_X: f32 = 52.0;
const RIGHT_COLUMN_OFFSET_X: f32 = 48.0;

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

actions!(
    ros_studio,
    [
        /// Opens the ROS 2 Studio graph.
        OpenGraph
    ]
);

pub fn init(cx: &mut App) {
    cx.observe_new(|workspace: &mut Workspace, _, _| {
        workspace.register_action(|workspace, _: &OpenGraph, window, cx| {
            let existing = workspace
                .active_pane()
                .read(cx)
                .items()
                .find_map(|item| item.downcast::<RosGraph>());

            if let Some(existing) = existing {
                workspace.activate_item(&existing, true, true, window, cx);
            } else {
                let workspace_handle = cx.weak_entity();
                let graph = cx.new(|cx| RosGraph::new(workspace_handle, cx));
                workspace.add_item_to_active_pane(Box::new(graph), None, true, window, cx);
            }
        });
    })
    .detach();
}

pub struct RosGraph {
    workspace: WeakEntity<Workspace>,
    project: Result<Project, SharedString>,
    node_layouts: Vec<GraphNodeLayout>,
    selected_node_id: Option<String>,
    node_drag: Option<GraphNodeDrag>,
    canvas_pan: Option<GraphCanvasPan>,
    camera_offset_x: f32,
    camera_offset_y: f32,
    canvas_bounds: Option<Bounds<Pixels>>,
    zoom_percent: u16,
    focus_handle: FocusHandle,
}

impl RosGraph {
    fn new(workspace: WeakEntity<Workspace>, cx: &mut Context<Self>) -> Self {
        let project = load_fixture_project().map_err(|error| {
            SharedString::from(format!("Failed to load ROS graph fixture: {error}"))
        });
        let node_layouts = project
            .as_ref()
            .map(|project| graph_layout(project, &graph_edges(project)))
            .unwrap_or_default();

        Self {
            workspace,
            project,
            node_layouts,
            selected_node_id: None,
            node_drag: None,
            canvas_pan: None,
            camera_offset_x: 0.0,
            camera_offset_y: 0.0,
            canvas_bounds: None,
            zoom_percent: 100,
            focus_handle: cx.focus_handle(),
        }
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
        let Some(source_path) = fixture_source_path(&source_location) else {
            return;
        };
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

fn fixture_source_path(source_location: &SourceLocation) -> Option<PathBuf> {
    let repository_root = Path::new(env!("CARGO_MANIFEST_DIR")).parent()?.parent()?;
    Some(
        repository_root
            .join(FIXTURE_WORKSPACE_DIRECTORY)
            .join(&source_location.path),
    )
}

fn load_fixture_project() -> Result<Project, serde_json::Error> {
    serde_json::from_str(FIXTURE_GRAPH)
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
                        && endpoint.type_name == publisher.type_name
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
        RuntimeState::Offline => "Offline",
        RuntimeState::RuntimeOnly => "Runtime only",
        RuntimeState::Conflict => "Conflict",
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

fn render_ros_inspector(
    project: &Project,
    selected_node_id: Option<&str>,
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
                    Label::new("DESIGN")
                        .size(LabelSize::XSmall)
                        .color(Color::Accent),
                ),
        );

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
                    Label::new(format!("{package_name} · {}", node.executable))
                        .size(LabelSize::XSmall)
                        .color(Color::Muted),
                )
                .child(
                    Label::new(runtime_state_label(node.runtime_state))
                        .size(LabelSize::XSmall)
                        .color(Color::Accent),
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
}

impl Render for RosGraph {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let content = v_flex()
            .size_full()
            .track_focus(&self.focus_handle)
            .id("ros-graph-content")
            .p_4()
            .gap_3()
            .bg(cx.theme().colors().editor_background);

        match &self.project {
            Ok(project) => {
                let selected_node_id = self.selected_node_id.clone();
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
                                                Label::new("DESIGN")
                                                    .size(LabelSize::XSmall)
                                                    .color(Color::Accent),
                                            ),
                                    )
                                    .child(
                                        Label::new(format!(
                                            "{} · {} packages · {} nodes · {} topics",
                                            project.name,
                                            project.packages.len(),
                                            project.nodes.len(),
                                            edges.len(),
                                        ))
                                        .size(LabelSize::Small)
                                        .color(Color::Muted),
                                    ),
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
                                        .on_click(
                                            cx.listener(|this, _, _, cx| {
                                                this.zoom_percent = 100;
                                                this.camera_offset_x = 0.0;
                                                this.camera_offset_y = 0.0;
                                                cx.notify();
                                            }),
                                        ),
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
                                        MouseButton::Left,
                                        cx.listener(|this, event: &MouseDownEvent, _, cx| {
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
                                                    .child(
                                                        h_flex()
                                                            .justify_between()
                                                            .child(
                                                                Headline::new(
                                                                    node.logical_name.clone(),
                                                                )
                                                                .size(HeadlineSize::Small),
                                                            )
                                                            .child(
                                                                Label::new("DESIGN")
                                                                    .size(LabelSize::XSmall)
                                                                    .color(Color::Muted),
                                                            ),
                                                    )
                                                    .child(
                                                        Label::new(format!(
                                                            "{package_name} · {}",
                                                            node.executable,
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
                                cx,
                            )),
                    )
            }
            Err(error) => content
                .justify_center()
                .items_center()
                .child(Headline::new("Unable to load ROS Graph").size(HeadlineSize::Small))
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
                    publisher_node_id: "node:camera:camera".to_owned(),
                    subscriber_node_id: "node:detector:detector".to_owned(),
                    topic: "/camera/image".to_owned(),
                    type_name: "sensor_msgs::msg::Image".to_owned(),
                },
                GraphEdge {
                    publisher_node_id: "node:detector:detector".to_owned(),
                    subscriber_node_id: "node:navigation:navigation".to_owned(),
                    topic: "/detections".to_owned(),
                    type_name: "vision_msgs::msg::Detection2DArray".to_owned(),
                },
                GraphEdge {
                    publisher_node_id: "node:navigation:navigation".to_owned(),
                    subscriber_node_id: "node:autopilot_bridge:autopilot_bridge".to_owned(),
                    topic: "/cmd_vel".to_owned(),
                    type_name: "geometry_msgs::msg::Twist".to_owned(),
                },
            ]
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
                    node_id: "node:camera:camera".to_owned(),
                    x: 48.0,
                    y: 64.0,
                },
                GraphNodeLayout {
                    node_id: "node:detector:detector".to_owned(),
                    x: 500.0,
                    y: 64.0,
                },
                GraphNodeLayout {
                    node_id: "node:navigation:navigation".to_owned(),
                    x: 548.0,
                    y: 360.0,
                },
                GraphNodeLayout {
                    node_id: "node:autopilot_bridge:autopilot_bridge".to_owned(),
                    x: 100.0,
                    y: 360.0,
                },
            ]
        );
        assert_eq!(graph_height(project.nodes.len()), 720.0);

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
}
