use std::{path::PathBuf, sync::Arc};

use anyhow::Result;
use gpui::{
    AppContext as _, Context, Entity, EventEmitter, FocusHandle, Focusable, Render, ScrollHandle,
    Subscription, Task, Window, div, prelude::*, px, rgb,
};
use ros_studio_app::{
    GraphSnapshot,
    wizard::{NodeDraft, rust_packages, topic_matches_filter},
};
use ros_studio_codegen::{
    ApiDetection, ApiSelection, NodeInterface, NodePreview, RclrsApi, TopicOption,
    available_topics, detect_rclrs_api, write_node,
};
use ros_studio_model::{EndpointKind, Project, SourceLocation};

use crate::{
    graph_view::button,
    text_input::{Changed, TextInput},
    theme,
};

pub enum WizardEvent {
    Cancelled,
    Created(SourceLocation),
}

impl EventEmitter<WizardEvent> for NodeWizard {}

pub struct NodeWizard {
    project: Project,
    draft: NodeDraft,
    topics: Vec<TopicOption>,
    node_name: Entity<TextInput>,
    topic_filter: Entity<TextInput>,
    focus: FocusHandle,
    api_note: String,
    preview: Option<Arc<NodePreview>>,
    error: Option<String>,
    busy: bool,
    scroll: ScrollHandle,
    task: Option<Task<()>>,
    _subscriptions: Vec<Subscription>,
}

impl NodeWizard {
    pub fn new(project: Project, window: &mut Window, cx: &mut Context<Self>) -> Self {
        let node_name = cx.new(|cx| TextInput::new("e.g. ground_station_bridge", true, cx));
        let topic_filter =
            cx.new(|cx| TextInput::new("Search topics or message types…", false, cx));
        window.focus(&node_name.focus_handle(cx), cx);
        let draft = NodeDraft {
            package_id: rust_packages(&project)
                .next()
                .map(|package| package.id.clone()),
            ..Default::default()
        };
        let subscriptions = vec![
            cx.subscribe(&node_name, |this, input, _: &Changed, cx| {
                this.draft.name = input.read(cx).text().to_owned();
                this.preview = None;
                this.error = None;
                cx.notify();
            }),
            cx.subscribe(&topic_filter, |_, _, _: &Changed, cx| cx.notify()),
        ];
        let mut wizard = Self {
            topics: available_topics(&project),
            project,
            draft,
            node_name,
            topic_filter,
            focus: cx.focus_handle(),
            api_note: String::new(),
            preview: None,
            error: None,
            busy: false,
            scroll: ScrollHandle::new(),
            task: None,
            _subscriptions: subscriptions,
        };
        wizard.update_api_note();
        wizard
    }

    fn update_api_note(&mut self) {
        self.api_note = match self
            .draft
            .package_id
            .as_ref()
            .map(|id| detect_rclrs_api(&self.project, id))
        {
            Some(Ok(ApiDetection::Supported(api))) => {
                format!("Detected rclrs {} in Cargo.toml", api.version())
            }
            Some(Ok(ApiDetection::Missing)) => "No rclrs dependency · Auto adds rclrs 0.7".into(),
            Some(Ok(ApiDetection::Unknown(version))) => {
                format!("Cannot infer API from {version} · choose a version manually")
            }
            Some(Ok(ApiDetection::Unsupported(version))) => {
                format!("Unsupported rclrs version: {version}")
            }
            Some(Err(error)) => format!("Cannot read API: {error:#}"),
            None => "No Rust package found. Python/C++ generation is not available yet.".into(),
        };
    }

    fn preview(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.busy {
            return;
        }
        self.draft.name = self.node_name.read(cx).text().to_owned();
        let draft = self.draft.clone();
        let root = PathBuf::from(&self.project.root_path);
        self.busy = true;
        self.error = None;
        window.focus(&self.focus, cx);
        let preview = cx.background_spawn(async move {
            // The code may have changed in the external editor since this dialog opened.
            let project = GraphSnapshot::scan(&root)?.project;
            let preview = draft.preview(&project);
            anyhow::Ok((project, preview))
        });
        self.task = Some(cx.spawn(async move |this, cx| {
            let result = preview.await;
            if let Err(error) = this.update(cx, |this, cx| {
                this.busy = false;
                match result {
                    Ok((project, preview)) => {
                        this.topics = available_topics(&project);
                        this.project = project;
                        this.update_api_note();
                        match preview {
                            Ok(preview) => this.preview = Some(Arc::new(preview)),
                            Err(error) => this.error = Some(format!("{error:#}")),
                        }
                        this.scroll.set_offset(gpui::point(px(0.0), px(0.0)));
                    }
                    Err(error) => this.error = Some(format!("{error:#}")),
                }
                cx.notify();
            }) {
                eprintln!("Cannot update node preview: {error:#}");
            }
        }));
        cx.notify();
    }

    fn create(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.busy {
            return;
        }
        let Some(preview) = self.preview.clone() else {
            return;
        };
        self.busy = true;
        self.error = None;
        window.focus(&self.focus, cx);
        let create = cx.background_spawn(async move {
            write_node(&preview)?;
            Ok::<_, anyhow::Error>(SourceLocation {
                path: preview
                    .source_relative_path
                    .to_string_lossy()
                    .replace('\\', "/"),
                line: 0,
                column: 0,
            })
        });
        self.task = Some(cx.spawn(async move |this, cx| {
            let result: Result<SourceLocation> = create.await;
            if let Err(error) = this.update(cx, |this, cx| {
                this.busy = false;
                match result {
                    Ok(source) => cx.emit(WizardEvent::Created(source)),
                    Err(error) => {
                        this.error = Some(format!("{error:#}. Go back and preview again."));
                    }
                }
                cx.notify();
            }) {
                eprintln!("Cannot report generated node: {error:#}");
            }
        }));
        cx.notify();
    }

    fn cancel(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if !self.busy {
            window.blur(cx);
            cx.emit(WizardEvent::Cancelled);
        }
    }

    fn form(&self, cx: &mut Context<Self>) -> gpui::Div {
        let packages = rust_packages(&self.project).cloned().collect::<Vec<_>>();
        let has_packages = !packages.is_empty();
        let filter = self.topic_filter.read(cx).text();
        let topics = self
            .topics
            .iter()
            .filter(|topic| topic_matches_filter(&topic.name, &topic.type_name, filter))
            .cloned()
            .collect::<Vec<_>>();
        let package_section = section("Rust package")
            .when(!has_packages, |section| {
                section.child(hint(
                    "No package with Cargo.toml found. Only Rust generation is supported.",
                ))
            })
            .children(packages.into_iter().enumerate().map(|(index, package)| {
                let selected = self.draft.package_id.as_ref() == Some(&package.id);
                button(
                    ("package", index),
                    format!("{} · {}", package.name, package.path),
                )
                .when(selected, |button| button.bg(rgb(theme::SELECTED)))
                .on_click(cx.listener(move |this, _, _, cx| {
                    this.draft.package_id = Some(package.id.clone());
                    this.draft.api = ApiSelection::Auto;
                    this.update_api_note();
                    this.error = None;
                    cx.notify();
                }))
            }));
        let topic_section = section("Interfaces from this project")
            .child(hint(format!(
                "Optional · {} selected · typed interfaces require rclrs 0.7",
                self.draft.interfaces.len()
            )))
            .child(self.topic_filter.clone())
            .child(hint(format!(
                "{} of {} topics",
                topics.len(),
                self.topics.len()
            )))
            .when(topics.is_empty(), |section| {
                section.child(hint(
                    "No matching typed topics. You can still create a node without interfaces.",
                ))
            })
            .children(
                topics
                    .into_iter()
                    .enumerate()
                    .map(|(index, topic)| self.topic_row(index, topic, cx)),
            );
        div()
            .flex()
            .flex_col()
            .gap_4()
            .child(
                section("Node name")
                    .child(self.node_name.clone())
                    .child(hint(
                        "Letters, digits, _ only · spaces become _ · other characters ignored",
                    )),
            )
            .child(package_section)
            .when(has_packages, |form| form.child(self.api_selector(cx)))
            .child(topic_section)
    }

    fn api_selector(&self, cx: &mut Context<Self>) -> gpui::Div {
        let choices = [
            ("Auto", ApiSelection::Auto),
            ("rclrs 0.6", ApiSelection::Manual(RclrsApi::V06)),
            ("rclrs 0.7", ApiSelection::Manual(RclrsApi::V07)),
        ];
        section("rclrs API")
            .child(hint(self.api_note.clone()))
            .child(
                div()
                    .flex()
                    .gap_2()
                    .children(
                        choices
                            .into_iter()
                            .enumerate()
                            .map(|(index, (label, api))| {
                                button(("api", index), label)
                                    .when(self.draft.api == api, |button| {
                                        button.bg(rgb(theme::SELECTED))
                                    })
                                    .on_click(cx.listener(move |this, _, _, cx| {
                                        this.draft.api = api;
                                        this.error = None;
                                        cx.notify();
                                    }))
                            }),
                    ),
            )
    }

    fn topic_row(&self, index: usize, topic: TopicOption, cx: &mut Context<Self>) -> gpui::Div {
        let publishers = if topic.publishers.is_empty() {
            "—".into()
        } else {
            topic.publishers.join(", ")
        };
        let subscribers = if topic.subscribers.is_empty() {
            "—".into()
        } else {
            topic.subscribers.join(", ")
        };
        let choices = [
            (EndpointKind::Subscription, "Subscribe"),
            (EndpointKind::Publisher, "Publish"),
        ];
        div()
            .flex()
            .flex_col()
            .gap_2()
            .p_3()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_md()
            .child(format!("{} · {}", topic.name, topic.type_name))
            .child(hint(format!(
                "Published by: {publishers} · Received by: {subscribers}"
            )))
            .child(
                div()
                    .flex()
                    .gap_2()
                    .children(choices.into_iter().enumerate().map(
                        |(kind_index, (kind, label))| {
                            let interface = NodeInterface {
                                kind,
                                name: topic.name.clone(),
                                type_name: topic.type_name.clone(),
                            };
                            let selected = self.draft.interfaces.contains(&interface);
                            button(
                                ("interface", index * 2 + kind_index),
                                format!("{} {label}", if selected { "✓" } else { "+" }),
                            )
                            .when(selected, |button| button.bg(rgb(theme::SELECTED)))
                            .on_click(cx.listener(
                                move |this, _, _, cx| {
                                    if !this.draft.interfaces.remove(&interface) {
                                        this.draft.interfaces.insert(interface.clone());
                                    }
                                    this.error = None;
                                    cx.notify();
                                },
                            ))
                        },
                    )),
            )
    }

    fn footer(&self, cx: &mut Context<Self>) -> gpui::Div {
        let previewing = self.preview.is_some();
        let label = if self.busy {
            "Working…"
        } else if previewing {
            "Create and open source"
        } else {
            "Preview changes"
        };
        div()
            .flex()
            .justify_end()
            .gap_2()
            .p_4()
            .border_t_1()
            .border_color(rgb(theme::BORDER))
            .child(
                button("wizard-cancel", "Cancel")
                    .when(self.busy, |button| button.opacity(0.5))
                    .on_click(cx.listener(|this, _, window, cx| this.cancel(window, cx))),
            )
            .when(previewing, |footer| {
                footer.child(button("wizard-back", "Back").on_click(cx.listener(
                    |this, _, window, cx| {
                        if !this.busy {
                            this.preview = None;
                            this.error = None;
                            this.scroll.set_offset(gpui::point(px(0.0), px(0.0)));
                            window.focus(&this.node_name.focus_handle(cx), cx);
                            cx.notify();
                        }
                    },
                )))
            })
            .child(
                button("wizard-confirm", label)
                    .bg(rgb(theme::PRIMARY))
                    .on_click(cx.listener(move |this, _, window, cx| {
                        if previewing {
                            this.create(window, cx);
                        } else {
                            this.preview(window, cx);
                        }
                    })),
            )
    }
}

impl Render for NodeWizard {
    fn render(&mut self, _: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let previewing = self.preview.is_some();
        let body = div()
            .id("wizard-scroll")
            .flex_1()
            .min_h_0()
            .overflow_y_scroll()
            .track_scroll(&self.scroll)
            .p_4()
            .when(!self.busy && !previewing, |body| body.child(self.form(cx)))
            .when_some(self.preview.clone(), |body, preview| {
                body.child(preview_files(&preview))
            })
            .when(self.busy, |body| {
                body.child(hint(if previewing {
                    "Creating node…"
                } else {
                    "Scanning and preparing preview…"
                }))
            });
        div()
            .id("node-wizard")
            .track_focus(&self.focus)
            .flex()
            .flex_col()
            .w_full()
            .max_w(px(850.0))
            .h_full()
            .bg(rgb(theme::SURFACE))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_lg()
            .overflow_hidden()
            .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
                match event.keystroke.key.as_str() {
                    "escape" => this.cancel(window, cx),
                    "tab" if !this.busy && this.preview.is_none() => {
                        let name = this.node_name.focus_handle(cx);
                        let filter = this.topic_filter.focus_handle(cx);
                        window.focus(
                            if name.is_focused(window) {
                                &filter
                            } else {
                                &name
                            },
                            cx,
                        );
                    }
                    _ => return,
                }
                cx.stop_propagation();
            }))
            .child(
                div()
                    .p_4()
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(div().text_xl().child(if previewing {
                        "Preview ROS node"
                    } else {
                        "New ROS node"
                    }))
                    .child(hint("Rust · creates a new binary in an existing package")),
            )
            .child(body)
            .when_some(self.error.clone(), |body, error| {
                body.child(
                    div()
                        .p_3()
                        .bg(rgb(theme::ERROR_BACKGROUND))
                        .text_sm()
                        .child(error),
                )
            })
            .child(self.footer(cx))
    }
}

fn section(title: &'static str) -> gpui::Div {
    div()
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
        .rounded_md()
        .bg(rgb(theme::ELEVATED))
        .child(div().text_lg().child(title))
}

fn hint(text: impl Into<gpui::SharedString>) -> gpui::Div {
    div()
        .text_sm()
        .text_color(rgb(theme::MUTED))
        .child(text.into())
}

fn code_preview(title: &'static str, path: &std::path::Path, source: &str) -> gpui::Div {
    section(title)
        .child(hint(path.display().to_string()))
        .child(
            div()
                .id(title)
                .overflow_x_scroll()
                .bg(rgb(theme::INPUT))
                .p_3()
                .text_sm()
                .font_family("monospace")
                .children(source.lines().map(|line| {
                    div()
                        .whitespace_nowrap()
                        .min_h(px(20.0))
                        .child(line.to_owned())
                })),
        )
}

fn preview_files(preview: &NodePreview) -> gpui::Div {
    div()
        .flex()
        .flex_col()
        .gap_4()
        .child(hint(preview.api_note.clone()))
        .child(code_preview(
            "New source file",
            &preview.source_relative_path,
            &preview.source,
        ))
        .when(preview.changes_manifest(), |body| {
            body.child(code_preview(
                "Cargo.toml · before",
                &preview.manifest_relative_path,
                &preview.manifest_before,
            ))
            .child(code_preview(
                "Cargo.toml · after",
                &preview.manifest_relative_path,
                &preview.manifest_after,
            ))
        })
        .child(hint(
            "Creating writes only these files. Existing sources are never overwritten.",
        ))
}
