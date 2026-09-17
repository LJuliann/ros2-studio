use std::path::PathBuf;

use editor::{Editor, EditorEvent};
use gpui::{
    App, Context, DismissEvent, Entity, EventEmitter, FocusHandle, Focusable, Render, ScrollHandle,
    Subscription, Task, TaskExt, WeakEntity, Window,
};
use multi_buffer::MultiBufferOffset;
use ros_studio_codegen::{
    ApiDetection, ApiSelection, NodePreview, RclrsApi, detect_rclrs_api, preview_node_with_api,
    write_node,
};
use ros_studio_model::{EntityId, Package, Project};
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
            preview: None,
            error: None,
            scroll_handle: ScrollHandle::new(),
            _scan_task: Task::ready(()),
            _subscriptions: vec![name_subscription],
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
            preview_node_with_api(project, package_id, node_name.trim(), self.api_selection)
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

impl EventEmitter<DismissEvent> for NodeWizard {}
impl ModalView for NodeWizard {}

impl Focusable for NodeWizard {
    fn focus_handle(&self, cx: &App) -> FocusHandle {
        self.node_name.focus_handle(cx)
    }
}

impl Render for NodeWizard {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let packages = self
            .project
            .as_ref()
            .map(|project| rust_packages(project).cloned().collect::<Vec<_>>())
            .unwrap_or_default();
        let selected_package_id = self.selected_package_id.clone();
        let is_preview = self.preview.is_some();

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
                        Label::new("Rust · rclrs · creates a new binary in the selected package")
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
                        body.child(Label::new("Node name").size(LabelSize::Small))
                            .child(self.node_name.clone())
                            .child(
                                Label::new(
                                    "Letters, digits, _ only · spaces become _ · other characters ignored",
                                )
                                    .size(LabelSize::XSmall)
                                    .color(Color::Muted),
                            )
                            .child(Label::new("Package").size(LabelSize::Small))
                            .when(self.project.is_none() && self.error.is_none(), |body| {
                                body.child(
                                    Label::new("Scanning workspace…")
                                        .size(LabelSize::Small)
                                        .color(Color::Muted),
                                )
                            })
                            .when(self.project.is_some() && packages.is_empty(), |body| {
                                body.child(
                                    Label::new("No Rust ROS package found in this workspace")
                                        .size(LabelSize::Small)
                                        .color(Color::Warning),
                                )
                            })
                            .children(packages.into_iter().enumerate().map(|(index, package)| {
                                let package_id = package.id;
                                Button::new(
                                    format!("ros-node-package-{index}"),
                                    package.name,
                                )
                                .toggle_state(selected_package_id.as_ref() == Some(&package_id))
                                .on_click(cx.listener(
                                    move |this, _, _, cx| {
                                        this.selected_package_id = Some(package_id.clone());
                                        this.refresh_api_description();
                                        this.error = None;
                                        cx.notify();
                                    },
                                ))
                            }))
                            .child(Label::new("rclrs API").size(LabelSize::Small))
                            .child(
                                Label::new(self.api_description.clone())
                                    .size(LabelSize::XSmall)
                                    .color(Color::Muted),
                            )
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
                                            .toggle_state(
                                                self.api_selection
                                                    == ApiSelection::Manual(RclrsApi::V06),
                                            )
                                            .on_click(cx.listener(|this, _, _, cx| {
                                                this.api_selection =
                                                    ApiSelection::Manual(RclrsApi::V06);
                                                this.error = None;
                                                cx.notify();
                                            })),
                                    )
                                    .child(
                                        Button::new("ros-node-api-07", "rclrs 0.7")
                                            .toggle_state(
                                                self.api_selection
                                                    == ApiSelection::Manual(RclrsApi::V07),
                                            )
                                            .on_click(cx.listener(|this, _, _, cx| {
                                                this.api_selection =
                                                    ApiSelection::Manual(RclrsApi::V07);
                                                this.error = None;
                                                cx.notify();
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
                                .p_2()
                                .border_1()
                                .border_color(cx.theme().colors().border_variant)
                                .child(preview.source.clone()),
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
                                    .p_2()
                                    .border_1()
                                    .border_color(cx.theme().colors().border_variant)
                                    .child(preview.manifest_after.clone()),
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
    use super::normalize_name_input;

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
}
