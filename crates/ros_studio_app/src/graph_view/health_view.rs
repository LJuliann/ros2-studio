//! Monitoring UI, separate from graph rendering and process execution.

use gpui::{Context, Div, Focusable as _, div, prelude::*, px, rgb};
use ros_studio_app::{
    health::{absolute_topic, automatic_watches, publisher_count},
    runtime::GraphMode,
};
use ros_studio_model::{EndpointKind, RuntimeState};
use ros_studio_protocol::health::{
    MAX_TOPIC_WATCHES, TopicHealthState, TopicWatch, validate_watches,
};
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
        if self.discovery {
            &self.health_topics
        } else {
            self.saved_health_watches()
        }
    }

    fn saved_health_watches(&self) -> &[TopicWatch] {
        self.design_project
            .as_ref()
            .and_then(|project| self.preferences.health_topics.get(&project.root_path))
            .map_or(&[], Vec::as_slice)
    }

    fn next_health_watches(&self) -> Vec<TopicWatch> {
        self.design_project
            .as_ref()
            .map(|design| {
                automatic_watches(
                    &self.overlay.project(design, GraphMode::Both),
                    self.saved_health_watches(),
                    &self.health_topics,
                )
            })
            .unwrap_or_default()
    }

    pub(super) fn refresh_health_watches(&mut self, cx: &mut Context<Self>) {
        if self.ready && self.discovery && self.next_health_watches() != self.health_topics {
            self.sync_health(cx);
        }
    }

    pub(super) fn sync_health(&mut self, cx: &mut Context<Self>) {
        if self.ready && self.discovery {
            self.health_topics = self.next_health_watches();
            self.health_snapshot.reset(Instant::now());
            self.health_error = None;
            let topics = self.health_topics.clone();
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
        let hz = current
            .and_then(|watch| watch.expected_hz)
            .map_or_else(String::new, |hz| hz.to_string());
        let timeout = current.map_or_else(
            || "5".to_owned(),
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
            let hz = self.health_hz.read(cx).text();
            let expected_hz = if hz.trim().is_empty() {
                None
            } else {
                Some(hz.trim().parse::<f64>()?)
            };
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
            let mut topics = self.saved_health_watches().to_vec();
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
        let mut content = div().id("health-scroll").flex().flex_col().flex_1()
            .min_h_0().overflow_y_scroll().gap_3().p_3()
            .child(div().text_sm().child("Topic frequencies"))
            .child(div().text_xs().text_color(rgb(theme::MUTED))
                .child("Topics are monitored automatically while connected. Click a topic to set an optional expected rate."));
        if !self.discovery {
            content = content.child(
                button("monitor-connect", "Connect to measure live frequencies")
                    .bg(rgb(theme::PRIMARY))
                    .on_click(cx.listener(|this, _, _, cx| this.connect_runtime(cx))),
            );
        }
        for error in [&self.health_error, &self.health_settings_error]
            .into_iter()
            .flatten()
        {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::WARNING))
                    .child(error.clone()),
            );
        }
        if watches.is_empty() {
            content = content.child(div().text_xs().text_color(rgb(theme::MUTED)).child(
                if self.discovery {
                    "Waiting for published topics to appear..."
                } else {
                    "Frequencies appear here after connecting."
                },
            ));
        }
        if watches.len() >= MAX_TOPIC_WATCHES {
            content = content.child(div().text_xs().text_color(rgb(theme::WARNING))
                .child(format!("Monitoring limit: {MAX_TOPIC_WATCHES} topics. Saved thresholds have priority.")));
        }
        for (index, watch) in watches.iter().enumerate() {
            let report = self
                .discovery
                .then(|| self.health_snapshot.get(&watch.topic, now))
                .flatten();
            let state = report.map(|report| report.state);
            let color = state.map_or(theme::MUTED, state_color);
            let hz = report
                .and_then(|report| report.observed_hz)
                .map_or_else(|| "— Hz".to_owned(), |hz| format!("{hz:.1} Hz"));
            let topic = watch.topic.clone();
            let type_name = watch.type_name.clone();
            let mut row = div()
                .flex()
                .flex_col()
                .gap_1()
                .pb_2()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .child(
                    div()
                        .flex()
                        .items_center()
                        .gap_2()
                        .child(
                            button(("health-topic", index), topic.clone())
                                .flex_1()
                                .min_w_0()
                                .truncate()
                                .on_click(cx.listener(move |this, _, window, cx| {
                                    this.choose_health_topic(topic.clone(), type_name.clone(), cx);
                                    window.focus(&this.health_hz.read(cx).focus_handle(cx), cx);
                                })),
                        )
                        .child(div().text_sm().text_color(rgb(color)).child(hz))
                        .child(div().size(px(9.0)).rounded_full().bg(rgb(color))),
                )
                .child(div().text_xs().text_color(rgb(color)).child(state.map_or(
                    if self.discovery {
                        "MEASURING"
                    } else {
                        "DISCONNECTED"
                    },
                    TopicHealthState::label,
                )));
            if let Some(expected) = watch.expected_hz {
                row = row.child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::MUTED))
                        .child(format!("Expected: {expected} Hz · ±20%")),
                );
            }
            if let Some(report) = report {
                if report.state == TopicHealthState::Unavailable {
                    row = row.child(
                        div()
                            .text_xs()
                            .text_color(rgb(theme::MUTED))
                            .child(report.detail.clone()),
                    );
                }
                if report.state == TopicHealthState::Missing
                    && let Some(age) = report.age_ms
                {
                    row = row.child(
                        div()
                            .text_xs()
                            .text_color(rgb(theme::MUTED))
                            .child(format!("Last message: {:.1}s ago", age as f64 / 1000.0)),
                    );
                }
            }
            content = content.child(row);
        }
        if let Some((topic, _)) = &self.health_topic {
            let reset = topic.clone();
            content = content.child(div().flex().flex_col().gap_2().p_2().rounded_md().bg(rgb(theme::INPUT))
                .child(div().text_sm().child(format!("Thresholds · {topic}")))
                .child(div().text_xs().child("Expected Hz (optional)"))
                .child(self.health_hz.clone())
                .child(div().text_xs().child("Silence timeout (seconds)"))
                .child(self.health_timeout.clone())
                .child(div().text_xs().text_color(rgb(theme::MUTED))
                    .child("Leave Hz empty to measure activity without a target. Automatic silence detection adapts to slow topics."))
                .child(div().flex().gap_2()
                    .child(button("health-save", "Save thresholds").bg(rgb(theme::PRIMARY))
                        .on_click(cx.listener(|this, _, _, cx| this.save_health_watch(cx))))
                    .child(button("health-reset", "Reset to automatic")
                        .on_click(cx.listener(move |this, _, _, cx| this.remove_health_watch(&reset, cx))))));
        }
        content = content.child(div().text_sm().child("Nodes"));
        if let Some(project) = project {
            for (index, node) in project.nodes.iter().enumerate() {
                let (presence, color) = if self.discovery && !self.health_snapshot.is_stale(now) {
                    match node.runtime_state {
                        RuntimeState::Online | RuntimeState::RuntimeOnly => {
                            ("ONLINE", theme::SUCCESS)
                        }
                        RuntimeState::Offline => ("OFFLINE", theme::ERROR),
                        RuntimeState::Conflict => ("CONFLICT", theme::WARNING),
                        RuntimeState::Unknown => ("UNKNOWN", theme::MUTED),
                    }
                } else if self.discovery {
                    ("UNKNOWN", theme::MUTED)
                } else {
                    ("DISCONNECTED", theme::MUTED)
                };
                let id = node.id.as_str().to_owned();
                let mut row = div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .pb_2()
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(
                        button(("health-node", index), node.logical_name.clone())
                            .text_xs()
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.selected_node = Some(id.clone());
                                this.inspector_view = InspectorView::Selection;
                                cx.notify();
                            })),
                    )
                    .child(div().text_xs().text_color(rgb(color)).child(presence));
                let published: BTreeSet<_> = node
                    .endpoints
                    .iter()
                    .filter(|endpoint| endpoint.kind == EndpointKind::Publisher)
                    .map(|endpoint| absolute_topic(&endpoint.name))
                    .filter(|topic| watches.iter().any(|watch| &watch.topic == topic))
                    .collect();
                for topic in &published {
                    let report = self
                        .discovery
                        .then(|| self.health_snapshot.get(topic, now))
                        .flatten();
                    let shared = publisher_count(&project, topic) > 1;
                    let hz = report
                        .and_then(|report| report.observed_hz)
                        .map_or_else(|| "—".to_owned(), |hz| format!("{hz:.1}"));
                    row = row.child(
                        div()
                            .text_xs()
                            .text_color(rgb(if shared {
                                theme::MUTED
                            } else {
                                report.map_or(theme::MUTED, |report| state_color(report.state))
                            }))
                            .child(format!(
                                "{topic} · {hz} Hz{}",
                                if shared { " (shared topic)" } else { "" }
                            )),
                    );
                }
                if published.is_empty() {
                    row = row.child(
                        div()
                            .text_xs()
                            .text_color(rgb(theme::MUTED))
                            .child("No measured output topic"),
                    );
                }
                content = content.child(row);
            }
        }
        content = content.child(div().text_xs().text_color(rgb(theme::MUTED))
            .child("Hz measure received topic messages. A node can publish several topics at different rates. Shared-topic rates combine publishers. Silence does not prove a node crashed."));
        div()
            .flex()
            .flex_col()
            .min_h_0()
            .w(px(360.0))
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
                    .child(div().flex_1().text_lg().child("Monitor"))
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
