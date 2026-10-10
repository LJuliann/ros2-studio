//! Monitoring UI, separate from graph rendering and process execution.

use gpui::{Context, Div, Focusable as _, div, prelude::*, px, rgb};
use ros_studio_app::{health::absolute_topic, runtime::GraphMode};
use ros_studio_model::{EndpointKind, RuntimeState, design_live::canonical_ros_type_name};
use ros_studio_protocol::health::{TopicHealthState, TopicWatch, validate_watches};
use std::{collections::BTreeSet, time::Instant};

use super::{GraphView, InspectorView, button};
use crate::theme;
use ros_studio_app::runtime_client::ClientCommand;

pub(super) fn state_color(state: TopicHealthState) -> u32 {
    match state {
        TopicHealthState::Healthy => theme::SUCCESS,
        TopicHealthState::Degraded => theme::WARNING,
        TopicHealthState::Missing => theme::ERROR,
        TopicHealthState::Waiting | TopicHealthState::Unavailable => theme::MUTED,
    }
}

impl GraphView {
    pub(super) fn health_watches(&self) -> &[TopicWatch] {
        self.design_project
            .as_ref()
            .and_then(|project| self.preferences.health_topics.get(&project.root_path))
            .map_or(&[], Vec::as_slice)
    }

    pub(super) fn sync_health(&mut self, cx: &mut Context<Self>) {
        self.health_snapshot.reset(Instant::now());
        self.health_error = None;
        if self.ready && self.discovery {
            let topics = self.health_watches().to_vec();
            if let Some(connection) = &self.connection
                && let Err(error) = connection.send(ClientCommand::ConfigureHealth(topics))
            {
                self.health_error = Some(error.to_string());
            }
        }
        cx.notify();
    }

    fn choose_health_topic(&mut self, topic: String, type_name: String, cx: &mut Context<Self>) {
        self.health_settings_error = None;
        let current = self
            .health_watches()
            .iter()
            .find(|watch| watch.topic == topic);
        let hz = current.map_or_else(String::new, |watch| watch.expected_hz.to_string());
        let timeout = current.map_or_else(
            || "3".to_owned(),
            |watch| (watch.timeout_ms as f64 / 1000.0).to_string(),
        );
        self.health_hz
            .update(cx, |input, cx| input.set_text(hz, cx));
        self.health_timeout
            .update(cx, |input, cx| input.set_text(timeout, cx));
        self.health_topic = Some((topic, type_name));
        self.inspector_view = InspectorView::Health;
        cx.notify();
    }

    fn save_health_watch(&mut self, cx: &mut Context<Self>) {
        self.health_settings_error = None;
        let result = (|| -> anyhow::Result<()> {
            let (topic, type_name) = self
                .health_topic
                .clone()
                .ok_or_else(|| anyhow::anyhow!("Choose a topic"))?;
            let expected_hz: f64 = self.health_hz.read(cx).text().trim().parse()?;
            let seconds: f64 = self.health_timeout.read(cx).text().trim().parse()?;
            anyhow::ensure!(
                seconds.is_finite() && (0.1..=600.0).contains(&seconds),
                "Timeout must be between 0.1 and 600 seconds"
            );
            let watch = TopicWatch {
                topic: topic.clone(),
                type_name,
                expected_hz,
                tolerance: 0.2,
                timeout_ms: (seconds * 1000.0).round() as u64,
            };
            let mut topics = self.health_watches().to_vec();
            topics.retain(|watch| watch.topic != topic);
            topics.push(watch);
            validate_watches(&topics).map_err(anyhow::Error::msg)?;
            let project = self
                .design_project
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("Open a project first"))?;
            self.preferences
                .health_topics
                .insert(project.root_path.clone(), topics);
            self.save_preferences(cx);
            self.sync_health(cx);
            if self.connection.is_none() {
                self.connect_runtime(cx);
            }
            Ok(())
        })();
        if let Err(error) = result {
            self.health_settings_error = Some(format!("Monitoring settings: {error:#}"));
        }
        cx.notify();
    }

    fn remove_health_watch(&mut self, topic: &str, cx: &mut Context<Self>) {
        if let Some(project) = &self.design_project
            && let Some(topics) = self.preferences.health_topics.get_mut(&project.root_path)
        {
            topics.retain(|watch| watch.topic != topic);
        }
        if self
            .health_topic
            .as_ref()
            .is_some_and(|(selected, _)| selected == topic)
        {
            self.health_topic = None;
        }
        self.save_preferences(cx);
        self.sync_health(cx);
    }

    pub(super) fn health_panel(&self, cx: &mut Context<Self>) -> Div {
        let now = Instant::now();
        let project = self
            .design_project
            .as_ref()
            .map(|design| self.overlay.project(design, GraphMode::Both));
        let watches = self.health_watches();
        let topics = project
            .as_ref()
            .map(|project| {
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
                    .map(|endpoint| {
                        (
                            absolute_topic(&endpoint.name),
                            canonical_ros_type_name(&endpoint.type_name),
                        )
                    })
                    .filter(|(topic, type_name)| {
                        validate_watches(&[TopicWatch {
                            topic: topic.clone(),
                            type_name: type_name.clone(),
                            expected_hz: 1.0,
                            tolerance: 0.2,
                            timeout_ms: 3000,
                        }])
                        .is_ok()
                    })
                    .collect::<BTreeSet<_>>()
            })
            .unwrap_or_default();
        let mut content = div().id("health-scroll").flex().flex_col().flex_1().min_h_0().overflow_y_scroll().gap_3().p_3()
            .child(div().text_xs().text_color(rgb(theme::MUTED)).child("Passive monitoring · received Hz, not algorithm correctness. No commands or test messages are sent."));
        if !self.discovery {
            content = content.child(div().text_sm().text_color(rgb(theme::WARNING)).child("Connect to the ROS runtime to measure live data. Saved settings remain available offline."));
        }
        if let Some(error) = &self.health_error {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::WARNING))
                    .child(error.clone()),
            );
        }
        if let Some(error) = &self.health_settings_error {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::ERROR))
                    .child(error.clone()),
            );
        }
        content = content.child(div().text_sm().child("Important topics"));
        if watches.is_empty() {
            content =
                content.child(div().text_xs().text_color(rgb(theme::MUTED)).child(
                    "Choose a topic below, then enter its expected Hz and silence timeout.",
                ));
        }
        for (index, watch) in watches.iter().enumerate() {
            let report = self
                .discovery
                .then(|| self.health_snapshot.get(&watch.topic, now))
                .flatten();
            let state = report.map(|report| report.state);
            let hz = report
                .and_then(|report| report.observed_hz)
                .map_or_else(|| "—".into(), |hz| format!("{hz:.1}"));
            let topic = watch.topic.clone();
            let type_name = watch.type_name.clone();
            let remove = topic.clone();
            content =
                content.child(
                    div()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .pb_2()
                        .border_b_1()
                        .border_color(rgb(theme::BORDER))
                        .child(button(("health-topic", index), topic.clone()).on_click(
                            cx.listener(move |this, _, window, cx| {
                                this.choose_health_topic(topic.clone(), type_name.clone(), cx);
                                window.focus(&this.health_hz.read(cx).focus_handle(cx), cx);
                            }),
                        ))
                        .child(
                            div()
                                .text_sm()
                                .text_color(rgb(state.map_or(theme::MUTED, state_color)))
                                .child(format!(
                                    "{} · {hz} / {} Hz",
                                    state.map_or("UNKNOWN", TopicHealthState::label),
                                    watch.expected_hz
                                )),
                        )
                        .when_some(report, |row, report| {
                            row.child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(theme::MUTED))
                                    .child(report.detail.clone()),
                            )
                            .when_some(report.age_ms, |row, age| {
                                row.child(div().text_xs().child(format!(
                                    "Last message: {:.1}s ago",
                                    age as f64 / 1000.0
                                )))
                            })
                        })
                        .child(button(("health-remove", index), "Stop watching").on_click(
                            cx.listener(move |this, _, _, cx| {
                                this.remove_health_watch(&remove, cx)
                            }),
                        )),
                );
        }
        if let Some((topic, type_name)) = &self.health_topic {
            content = content.child(div().flex().flex_col().gap_2().p_2().rounded_md().bg(rgb(theme::INPUT))
                .child(div().text_sm().child(format!("Configure {topic}")))
                    .child(div().text_xs().text_color(rgb(theme::MUTED)).child(type_name.clone()))
                .child(div().text_xs().child("Expected rate (Hz)"))
                .child(self.health_hz.clone())
                .child(div().text_xs().child("Silence timeout (seconds)"))
                .child(self.health_timeout.clone())
                .child(div().text_xs().text_color(rgb(theme::MUTED)).child("Tolerance: ±20% · warm-up: 5s · choose a timeout longer than the normal interval."))
                .child(button("health-save", "Save and monitor").bg(rgb(theme::PRIMARY)).on_click(cx.listener(|this, _, _, cx| this.save_health_watch(cx)))));
        }
        content = content.child(div().text_sm().child("Available topics"));
        for (index, (topic, type_name)) in topics.into_iter().enumerate() {
            if watches.iter().any(|watch| watch.topic == topic) {
                continue;
            }
            let message_type = type_name.clone();
            content = content.child(
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .child(button(("health-available", index), topic.clone()).on_click(
                        cx.listener(move |this, _, window, cx| {
                            this.choose_health_topic(topic.clone(), type_name.clone(), cx);
                            window.focus(&this.health_hz.read(cx).focus_handle(cx), cx);
                        }),
                    ))
                    .child(
                        div()
                            .text_xs()
                            .text_color(rgb(theme::MUTED))
                            .child(message_type),
                    ),
            );
        }
        content = content.child(div().text_sm().child("Node presence"));
        if let Some(project) = project {
            for (index, node) in project.nodes.iter().enumerate() {
                let state = if self.discovery {
                    match node.runtime_state {
                        RuntimeState::Online | RuntimeState::RuntimeOnly => "PRESENT",
                        RuntimeState::Offline => "ABSENT",
                        RuntimeState::Conflict => "CONFLICT",
                        RuntimeState::Unknown => "UNKNOWN",
                    }
                } else {
                    "UNKNOWN"
                };
                let health = self
                    .discovery
                    .then(|| {
                        self.health_snapshot
                            .node_state(&project, node, watches, now)
                    })
                    .flatten();
                let label = format!(
                    "{} · {state}{}",
                    node.logical_name,
                    health.map_or_else(String::new, |state| format!(" · {}", state.label()))
                );
                let id = node.id.as_str().to_owned();
                content = content.child(
                    button(("health-node", index), label)
                        .text_xs()
                        .text_color(rgb(health.map_or(
                            if state == "ABSENT" {
                                theme::ERROR
                            } else {
                                theme::MUTED
                            },
                            state_color,
                        )))
                        .on_click(cx.listener(move |this, _, _, cx| {
                            this.selected_node = Some(id.clone());
                            this.inspector_view = InspectorView::Selection;
                            cx.notify();
                        })),
                );
            }
        }
        content = content.child(div().text_xs().text_color(rgb(theme::MUTED)).child("Multiple publishers: Hz are aggregated, not attributed to a node. QoS, CPU/network load or missing type support can affect observations. Close only hides this panel."));
        div()
            .flex()
            .flex_col()
            .min_h_0()
            .w(px(320.0))
            .flex_shrink_0()
            .bg(rgb(theme::SURFACE))
            .border_l_1()
            .border_color(rgb(theme::BORDER))
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .p_3()
                    .child(div().flex_1().text_lg().child("Health"))
                    .child(button("health-close", "Close").on_click(cx.listener(
                        |this, _, _, cx| {
                            this.inspector_view = InspectorView::Selection;
                            cx.notify();
                        },
                    ))),
            )
            .child(content)
    }
}
