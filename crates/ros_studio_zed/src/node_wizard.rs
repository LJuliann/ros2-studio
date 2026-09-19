use std::{collections::BTreeSet, path::PathBuf};

use editor::{Editor, EditorEvent};
use gpui::{
    App, Context, DismissEvent, Entity, EventEmitter, FocusHandle, Focusable, Render, ScrollHandle,
    Subscription, Task, TaskExt, WeakEntity, Window,
};
use multi_buffer::MultiBufferOffset;
use ros_studio_codegen::{
    ApiDetection, ApiSelection, NodeInterface, NodePreview, RclrsApi, available_topics,
    detect_rclrs_api, preview_node_with_interfaces, write_node,
};
use ros_studio_model::{EndpointKind, EntityId, Package, Project};
use ui::{Button, Color, Headline, HeadlineSize, Label, LabelSize, prelude::*};
use workspace::{ModalView, Workspace};

use super::{graph_item, open_graph};

pub struct NodeWizard {
    workspace: WeakEntity<Workspace>,
    project: Option<Project>,
    selected_package_id: Option<EntityId>,
    api_selection: ApiSelection,
    api_description: String,
    node_name: Entity<Editor>,
    topic_filter: Entity<Editor>,
    selected_interfaces: BTreeSet<NodeInterface>,
    preview: Option<NodePreview>,
    error: Option<String>,
    scroll_handle: ScrollHandle,
    _scan_task: Task<()>,
    _subscriptions: Vec<Subscription>,
}

impl NodeWizard {
    pub fn new(
        workspace: WeakEntity<Workspace>,
        workspace_root: PathBuf,
        initial_project: Option<Project>,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> Self {
        let node_name = cx.new(|cx| {
            let mut editor = Editor::single_line(window, cx);
            editor.set_placeholder_text("e.g. camera_backup", window, cx);
            editor
        });
        let topic_filter = cx.new(|cx| {
            let mut editor = Editor::single_line(window, cx);
            editor.set_placeholder_text("Filter project topics by name or type", window, cx);
            editor
        });
        let name_subscription = cx.subscribe_in(
            &node_name,
            window,
            |_, _, event: &EditorEvent, window, cx| {
                if matches!(event, EditorEvent::BufferEdited) {
                    cx.defer_in(window, |this, window, cx| {
                        this.normalize_node_name(window, cx);
                    });
                }
            },
        );
        let topic_filter_subscription =
            cx.subscribe_in(&topic_filter, window, |_, _, event: &EditorEvent, _, cx| {
                if matches!(event, EditorEvent::BufferEdited) {
                    cx.notify();
                }
            });
        let selected_package_id = initial_project
            .as_ref()
            .and_then(|project| rust_packages(project).next())
            .map(|package| package.id.clone());
        let mut wizard = Self {
            workspace,
            project: initial_project,
            selected_package_id,
            api_selection: ApiSelection::Auto,
            api_description: String::new(),
            node_name,
            topic_filter,
            selected_interfaces: BTreeSet::new(),
            preview: None,
            error: None,
            scroll_handle: ScrollHandle::new(),
            _scan_task: Task::ready(()),
            _subscriptions: vec![name_subscription, topic_filter_subscription],
        };
        wizard.refresh_api_description();

        if wizard.project.is_none() {
            let project_name = workspace_root
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("ros_workspace")
                .to_owned();
            wizard._scan_task = cx.spawn(async move |this, cx| {
                let result = cx
                    .background_spawn(async move {
                        ros_studio_scan::scan_project(&workspace_root, &project_name)
                    })
                    .await;
                if let Some(this) = this.upgrade() {
                    this.update(cx, |this, cx| {
                        match result {
                            Ok(project) => {
                                this.selected_package_id = rust_packages(&project)
                                    .next()
                                    .map(|package| package.id.clone());
                                this.project = Some(project);
                                this.refresh_api_description();
                            }
                            Err(error) => {
                                this.error = Some(format!("Workspace scan failed: {error:#}"))
                            }
                        }
                        cx.notify();
                    });
                }
            });
        }

        wizard
    }

    fn normalize_node_name(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let (raw_name, cursor_offset) = self.node_name.update(cx, |editor, cx| {
            let selection = editor
                .selections
                .newest::<MultiBufferOffset>(&editor.display_snapshot(cx));
            (editor.text(cx), selection.head().0)
        });
        let normalized_name = normalize_name_input(&raw_name);
        if normalized_name == raw_name {
            return;
        }

        let prefix = raw_name.get(..cursor_offset).unwrap_or(raw_name.as_str());
        let normalized_cursor_offset = normalize_name_input(prefix).len();
        self.node_name.update(cx, |editor, cx| {
            editor.set_text(normalized_name, window, cx);
            let cursor = MultiBufferOffset(normalized_cursor_offset);
            editor.change_selections(Default::default(), window, cx, |selections| {
                selections.select_ranges([cursor..cursor]);
            });
        });
        self.error = None;
        cx.notify();
    }

    fn refresh_api_description(&mut self) {
        self.api_description = match self
            .project
            .as_ref()
            .zip(self.selected_package_id.as_ref())
            .map(|(project, package_id)| detect_rclrs_api(project, package_id))
        {
            Some(Ok(ApiDetection::Missing)) => {
                "No rclrs dependency found · Auto adds rclrs 0.7".to_owned()
            }
            Some(Ok(ApiDetection::Supported(api))) => {
                format!("Detected rclrs {} in Cargo.toml", api.version())
            }
            Some(Ok(ApiDetection::Unknown(reason))) => {
                format!("Could not detect API: {reason} · select a version manually")
            }
            Some(Ok(ApiDetection::Unsupported(reason))) => {
                format!("Cannot generate a node: {reason}")
            }
            Some(Err(error)) => format!("Could not read Cargo.toml: {error:#}"),
            None => "Select a package to detect its rclrs API".to_owned(),
        };
    }

    fn show_preview(&mut self, cx: &mut Context<Self>) {
        self.refresh_api_description();
        let interfaces = self.selected_interfaces.iter().cloned().collect::<Vec<_>>();
        let result = (|| {
            let project = self
                .project
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("Workspace scan is not complete"))?;
            let package_id = self
                .selected_package_id
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("Select a Rust package"))?;
            let node_name = self.node_name.read(cx).text(cx);
            preview_node_with_interfaces(
                project,
                package_id,
                node_name.trim(),
                self.api_selection,
                &interfaces,
            )
        })();
        match result {
            Ok(preview) => {
                self.preview = Some(preview);
                self.error = None;
            }
            Err(error) => self.error = Some(format!("{error:#}")),
        }
        cx.notify();
    }

    fn create_node(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(preview) = self.preview.as_ref() else {
            return;
        };
        match write_node(preview) {
            Ok(source_path) => {
                let workspace = self.workspace.clone();
                cx.emit(DismissEvent);
                window
                    .spawn(cx, async move |cx| {
                        workspace
                            .update_in(cx, |workspace, window, cx| {
                                if let Some((_, graph)) = graph_item(workspace, cx) {
                                    graph.update(cx, |graph, cx| {
                                        graph.schedule_rescan(std::time::Duration::ZERO, cx);
                                    });
                                } else {
                                    open_graph(workspace, window, cx);
                                }
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
                        anyhow::Ok(())
                    })
                    .detach_and_log_err(cx);
            }
            Err(error) => {
                self.preview = None;
                self.error = Some(format!("Node creation failed: {error:#}"));
                cx.notify();
            }
        }
    }
}

fn rust_packages(project: &Project) -> impl Iterator<Item = &Package> {
    project.packages.iter().filter(|package| {
        PathBuf::from(&project.root_path)
            .join(&package.path)
            .join("Cargo.toml")
            .is_file()
    })
}

fn normalize_name_input(name: &str) -> String {
    let mut normalized = String::with_capacity(name.len());
    for character in name.chars() {
        let character = if character.is_whitespace() {
            '_'
        } else {
            character
        };
        let allowed = if normalized.is_empty() {
            character.is_ascii_alphabetic() || character == '_'
        } else {
            character.is_ascii_alphanumeric() || character == '_'
        };
        if allowed {
            normalized.push(character);
        }
    }
    normalized
}

fn topic_matches_filter(name: &str, type_name: &str, filter: &str) -> bool {
    let name = name.to_lowercase();
    let type_name = type_name.to_lowercase();
    filter
        .to_lowercase()
        .split_whitespace()
        .all(|term| fuzzy_subsequence(term, &name) || fuzzy_subsequence(term, &type_name))
}

fn fuzzy_subsequence(needle: &str, haystack: &str) -> bool {
    let mut characters = haystack.chars();
    needle
        .chars()
        .all(|character| characters.by_ref().any(|candidate| candidate == character))
}

impl EventEmitter<DismissEvent> for NodeWizard {}
impl ModalView for NodeWizard {}

impl Focusable for NodeWizard {
    fn focus_handle(&self, cx: &App) -> FocusHandle {
        self.node_name.focus_handle(cx)
    }
}

impl Render for NodeWizard {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let packages = self
            .project
            .as_ref()
            .map(|project| rust_packages(project).cloned().collect::<Vec<_>>())
            .unwrap_or_default();
        let selected_package_id = self.selected_package_id.clone();
        let selected_interfaces = self.selected_interfaces.clone();
        let filter = self.topic_filter.read(cx).text(cx);
        let topics = self
            .project
            .as_ref()
            .map(available_topics)
            .unwrap_or_default();
        let visible_topics = topics
            .iter()
            .filter(|topic| topic_matches_filter(&topic.name, &topic.type_name, &filter))
            .cloned()
            .collect::<Vec<_>>();
        let has_selected_rust_package = selected_package_id
            .as_ref()
            .is_some_and(|selected_id| packages.iter().any(|package| &package.id == selected_id));
        let is_filtering_topics = !filter.trim().is_empty();
        let topic_count_label = if is_filtering_topics {
            format!("{} of {} topics", visible_topics.len(), topics.len())
        } else {
            format!("{} topics", topics.len())
        };
        let is_preview = self.preview.is_some();
        let node_name_focus = self.node_name.focus_handle(cx);
        let topic_filter_focus = self.topic_filter.focus_handle(cx);

        v_flex()
            .id("ros-node-wizard")
            .w(rems(42.0))
            .max_h(px(650.0))
            .overflow_hidden()
            .elevation_3(cx)
            .bg(cx.theme().colors().elevated_surface_background)
            .child(
                v_flex()
                    .p_4()
                    .gap_1()
                    .border_b_1()
                    .border_color(cx.theme().colors().border_variant)
                    .child(Headline::new("New ROS Node").size(HeadlineSize::Large))
                    .child(
                        Label::new("Creates a Rust binary in the selected package")
                            .size(LabelSize::Small)
                            .color(Color::Muted),
                    ),
            )
            .child(
                v_flex()
                    .id("ros-node-wizard-body")
                    .p_4()
                    .gap_3()
                    .overflow_y_scroll()
                    .track_scroll(&self.scroll_handle)
                    .when(!is_preview, |body| {
                        body.child(
                            v_flex()
                                .gap_2()
                                .p_3()
                                .rounded_md()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(Label::new("Node name").size(LabelSize::Large))
                                .child(
                                    h_flex()
                                        .id("ros-node-name-input")
                                        .track_focus(&node_name_focus)
                                        .min_h_8()
                                        .w_full()
                                        .px_2()
                                        .py_1p5()
                                        .rounded_md()
                                        .bg(cx.theme().colors().editor_background)
                                        .border_1()
                                        .border_color(if node_name_focus.contains_focused(window, cx) {
                                            cx.theme().colors().border_focused
                                        } else {
                                            cx.theme().colors().border_variant
                                        })
                                        .on_click(cx.listener(|this, _, window, cx| {
                                            this.node_name.focus_handle(cx).focus(window, cx);
                                        }))
                                        .child(self.node_name.clone()),
                                )
                                .child(
                                    Label::new("Letters, digits, _ only · spaces become _ · other characters ignored")
                                        .size(LabelSize::Small)
                                        .color(Color::Muted),
                                ),
                        )
                        .child(
                            v_flex()
                                .gap_2()
                                .p_3()
                                .rounded_md()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(Label::new("Package").size(LabelSize::Large))
                                .child(
                                    Label::new("Choose where the new Rust binary will be created")
                                        .size(LabelSize::Small)
                                        .color(Color::Muted),
                                )
                                .when(self.project.is_none() && self.error.is_none(), |section| {
                                    section.child(Label::new("Scanning workspace…").color(Color::Muted))
                                })
                                .when(self.project.is_some() && packages.is_empty(), |section| {
                                    section.child(Label::new("No Rust ROS package found in this workspace").color(Color::Warning))
                                })
                                .child(
                                    h_flex()
                                        .gap_2()
                                        .flex_wrap()
                                        .children(packages.into_iter().enumerate().map(|(index, package)| {
                                            let package_id = package.id;
                                            Button::new(format!("ros-node-package-{index}"), package.name)
                                                .toggle_state(selected_package_id.as_ref() == Some(&package_id))
                                                .on_click(cx.listener(move |this, _, _, cx| {
                                                    this.selected_package_id = Some(package_id.clone());
                                                    this.refresh_api_description();
                                                    this.error = None;
                                                    cx.notify();
                                                }))
                                        })),
                                ),
                        )
                        .when(has_selected_rust_package, |body| body.child(
                            v_flex()
                                .gap_2()
                                .p_3()
                                .rounded_md()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(Label::new("rclrs API").size(LabelSize::Large))
                                .child(Label::new(self.api_description.clone()).size(LabelSize::Small).color(Color::Muted))
                                .child(
                                    h_flex()
                                        .gap_2()
                                        .child(
                                            Button::new("ros-node-api-auto", "Auto")
                                                .toggle_state(self.api_selection == ApiSelection::Auto)
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.api_selection = ApiSelection::Auto;
                                                    this.error = None;
                                                    cx.notify();
                                                })),
                                        )
                                        .child(
                                            Button::new("ros-node-api-06", "rclrs 0.6")
                                                .toggle_state(self.api_selection == ApiSelection::Manual(RclrsApi::V06))
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.api_selection = ApiSelection::Manual(RclrsApi::V06);
                                                    this.error = None;
                                                    cx.notify();
                                                })),
                                        )
                                        .child(
                                            Button::new("ros-node-api-07", "rclrs 0.7")
                                                .toggle_state(self.api_selection == ApiSelection::Manual(RclrsApi::V07))
                                                .on_click(cx.listener(|this, _, _, cx| {
                                                    this.api_selection = ApiSelection::Manual(RclrsApi::V07);
                                                    this.error = None;
                                                    cx.notify();
                                                })),
                                        ),
                                ),
                        ))
                        .child(
                            v_flex()
                                .gap_2()
                                .p_3()
                                .rounded_md()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(Label::new("Interfaces from this project").size(LabelSize::Large))
                                .child(Label::new("Only typed topics detected in this project · choose Subscribe or Publish").size(LabelSize::Small).color(Color::Muted))
                                .child(
                                    h_flex()
                                        .w_full()
                                        .justify_between()
                                        .child(Label::new("Detected topics"))
                                        .child(Label::new(topic_count_label).size(LabelSize::Small).color(Color::Muted)),
                                )
                                .child(
                                    h_flex()
                                        .id("ros-topic-filter-input")
                                        .track_focus(&topic_filter_focus)
                                        .min_h_8()
                                        .w_full()
                                        .px_2()
                                        .py_1p5()
                                        .rounded_md()
                                        .bg(cx.theme().colors().editor_background)
                                        .border_1()
                                        .border_color(if topic_filter_focus.contains_focused(window, cx) {
                                            cx.theme().colors().border_focused
                                        } else {
                                            cx.theme().colors().border_variant
                                        })
                                        .on_click(cx.listener(|this, _, window, cx| {
                                            this.topic_filter.focus_handle(cx).focus(window, cx);
                                        }))
                                        .child(self.topic_filter.clone()),
                                )
                                .when(is_filtering_topics, |section| {
                                    section.child(Button::new("ros-topic-clear-filter", "Clear filter").on_click(
                                        cx.listener(|this, _, window, cx| {
                                            this.topic_filter.update(cx, |editor, cx| {
                                                editor.set_text("", window, cx);
                                                editor.focus_handle(cx).focus(window, cx);
                                            });
                                        }),
                                    ))
                                })
                                .child(
                                    v_flex()
                                        .id("ros-topic-results")
                                        .gap_2()
                                        .max_h(px(240.0))
                                        .overflow_y_scroll()
                                        .when(topics.is_empty(), |list| {
                                            list.child(Label::new("No typed ROS topics detected yet; create a node without interfaces").size(LabelSize::Small).color(Color::Muted))
                                        })
                                        .when(!topics.is_empty() && visible_topics.is_empty(), |list| {
                                            list.child(Label::new("No topics match this filter").size(LabelSize::Small).color(Color::Muted))
                                        })
                                        .children(visible_topics.into_iter().map(|topic| {
                                let subscription = NodeInterface {
                                    kind: EndpointKind::Subscription,
                                    name: topic.name.clone(),
                                    type_name: topic.type_name.clone(),
                                };
                                let publisher = NodeInterface {
                                    kind: EndpointKind::Publisher,
                                    name: topic.name.clone(),
                                    type_name: topic.type_name.clone(),
                                };
                                let producers = if topic.publishers.is_empty() {
                                    "No known publisher".to_owned()
                                } else {
                                    format!("Published by {}", topic.publishers.join(", "))
                                };
                                v_flex()
                                    .gap_1()
                                    .p_2()
                                    .border_1()
                                    .border_color(cx.theme().colors().border_variant)
                                    .child(Label::new(format!("{} · {}", topic.name, topic.type_name)).size(LabelSize::Small))
                                    .child(Label::new(producers).size(LabelSize::XSmall).color(Color::Muted))
                                    .child(
                                        h_flex()
                                            .gap_2()
                                            .child(
                                                Button::new(format!("ros-topic-subscribe-{}-{}", topic.name, topic.type_name), "Subscribe")
                                                    .toggle_state(selected_interfaces.contains(&subscription))
                                                    .on_click(cx.listener(move |this, _, _, cx| {
                                                        if !this.selected_interfaces.insert(subscription.clone()) {
                                                            this.selected_interfaces.remove(&subscription);
                                                        }
                                                        this.error = None;
                                                        cx.notify();
                                                    })),
                                            )
                                            .child(
                                                Button::new(format!("ros-topic-publish-{}-{}", topic.name, topic.type_name), "Publish")
                                                    .toggle_state(selected_interfaces.contains(&publisher))
                                                    .on_click(cx.listener(move |this, _, _, cx| {
                                                        if !this.selected_interfaces.insert(publisher.clone()) {
                                                            this.selected_interfaces.remove(&publisher);
                                                        }
                                                        this.error = None;
                                                        cx.notify();
                                                    })),
                                            ),
                                    )
                                        })),
                                ),
                        )
                    })
                    .when_some(self.preview.as_ref(), |body, preview| {
                        body.child(
                            Label::new(preview.api_note.clone())
                                .size(LabelSize::Small)
                                .color(Color::Muted),
                        )
                        .child(
                            Label::new(format!(
                                "Create {}",
                                preview.source_relative_path.display()
                            ))
                            .size(LabelSize::Small),
                        )
                        .child(
                            div()
                                .id("ros-node-source-preview")
                                .p_2()
                                .flex()
                                .min_w_0()
                                .overflow_x_scroll()
                                .restrict_scroll_to_axis()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(
                                    div()
                                        .flex_none()
                                        .whitespace_nowrap()
                                        .child(preview.source.clone()),
                                ),
                        )
                        .when(preview.changes_manifest(), |body| {
                            body.child(
                                Label::new(format!(
                                    "Update {} (add missing dependencies)",
                                    preview.manifest_relative_path.display()
                                ))
                                .size(LabelSize::Small),
                            )
                            .child(
                                div()
                                    .id("ros-node-manifest-preview")
                                    .p_2()
                                    .flex()
                                    .min_w_0()
                                    .overflow_x_scroll()
                                    .restrict_scroll_to_axis()
                                    .border_1()
                                    .border_color(cx.theme().colors().border_variant)
                                    .child(
                                        div()
                                            .flex_none()
                                            .whitespace_nowrap()
                                            .child(preview.manifest_after.clone()),
                                    ),
                            )
                        })
                    })
                    .when_some(self.error.as_ref(), |body, error| {
                        body.child(
                            Label::new(error.clone())
                                .size(LabelSize::Small)
                                .color(Color::Error),
                        )
                    }),
            )
            .child(
                h_flex()
                    .p_3()
                    .gap_2()
                    .justify_end()
                    .border_t_1()
                    .border_color(cx.theme().colors().border_variant)
                    .child(
                        Button::new("ros-node-cancel", "Cancel")
                            .on_click(cx.listener(|_, _, _, cx| cx.emit(DismissEvent))),
                    )
                    .when(is_preview, |footer| {
                        footer
                            .child(Button::new("ros-node-back", "Back").on_click(cx.listener(
                                |this, _, _, cx| {
                                    this.preview = None;
                                    this.error = None;
                                    cx.notify();
                                },
                            )))
                            .child(Button::new("ros-node-create", "Create node").on_click(
                                cx.listener(|this, _, window, cx| {
                                    this.create_node(window, cx);
                                }),
                            ))
                    })
                    .when(!is_preview, |footer| {
                        footer.child(
                            Button::new("ros-node-preview", "Preview changes")
                                .disabled(self.project.is_none() || selected_package_id.is_none())
                                .on_click(cx.listener(|this, _, _, cx| this.show_preview(cx))),
                        )
                    }),
            )
    }
}

#[cfg(test)]
mod tests {
    use super::{normalize_name_input, topic_matches_filter};

    #[test]
    fn normalizes_node_names_while_typing() {
        assert_eq!(normalize_name_input("camera backup"), "camera_backup");
        assert_eq!(normalize_name_input("camera-backup"), "camerabackup");
        assert_eq!(normalize_name_input("camera - backup"), "camera__backup");
        assert_eq!(normalize_name_input("camera\tbackup"), "camera_backup");
        assert_eq!(normalize_name_input("camera@backup!"), "camerabackup");
        assert_eq!(normalize_name_input("3camera"), "camera");
        assert_eq!(normalize_name_input("_camera2"), "_camera2");
        assert_eq!(normalize_name_input("caméra"), "camra");
    }

    #[test]
    fn searches_topics_by_name_or_type() {
        assert!(topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "cam img"
        ));
        assert!(topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "Sensor Image"
        ));
        assert!(!topic_matches_filter(
            "/camera/image",
            "sensor_msgs::msg::Image",
            "velocity"
        ));
    }
}
