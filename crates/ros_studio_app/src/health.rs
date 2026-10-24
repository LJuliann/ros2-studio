//! Honest display of monitoring evidence: never attribute an aggregated topic's
//! rate to one of several publishers, or keep a green badge after reports stop.

use ros_studio_model::{
    EndpointKind, EntityId, Node, Project, RuntimeState, design_live::canonical_ros_type_name,
};
use ros_studio_protocol::health::{
    HealthReport, MAX_TOPIC_WATCHES, TopicHealth, TopicHealthState, TopicWatch, validate_watches,
};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

#[derive(Default)]
pub struct HealthSnapshot {
    updated: Option<Instant>,
    topics: BTreeMap<String, TopicHealth>,
}

pub fn absolute_topic(topic: &str) -> String {
    format!("/{}", topic.trim_start_matches('/'))
}

/// Keep discovered topics for the session, so disappearing publishers remain
/// visible as silent. User thresholds take priority over automatic settings.
pub fn automatic_watches(
    project: &Project,
    saved: &[TopicWatch],
    previous: &[TopicWatch],
) -> Vec<TopicWatch> {
    let mut discovered = BTreeMap::new();
    for watch in previous {
        discovered.insert(
            watch.topic.clone(),
            TopicWatch {
                expected_hz: None,
                tolerance: 0.2,
                timeout_ms: 5000,
                ..watch.clone()
            },
        );
    }
    for endpoint in project
        .nodes
        .iter()
        .filter(|node| {
            matches!(
                node.runtime_state,
                RuntimeState::Online | RuntimeState::RuntimeOnly | RuntimeState::Conflict
            )
        })
        .flat_map(|node| &node.endpoints)
        .filter(|endpoint| endpoint.kind == EndpointKind::Publisher)
    {
        let topic = absolute_topic(&endpoint.name);
        if matches!(topic.as_str(), "/rosout" | "/parameter_events") {
            continue;
        }
        let watch = TopicWatch {
            topic: topic.clone(),
            type_name: canonical_ros_type_name(&endpoint.type_name),
            expected_hz: None,
            tolerance: 0.2,
            timeout_ms: 5000,
        };
        if validate_watches(std::slice::from_ref(&watch)).is_ok() {
            discovered.insert(topic, watch);
        }
    }
    let mut watches: BTreeMap<_, _> = saved
        .iter()
        .filter(|watch| validate_watches(std::slice::from_ref(*watch)).is_ok())
        .take(MAX_TOPIC_WATCHES)
        .map(|watch| (watch.topic.clone(), watch.clone()))
        .collect();
    for (topic, watch) in discovered {
        if watches.len() >= MAX_TOPIC_WATCHES {
            break;
        }
        watches.entry(topic).or_insert(watch);
    }
    watches.into_values().collect()
}

pub fn publisher_count(project: &Project, topic: &str) -> usize {
    project
        .nodes
        .iter()
        .filter(|node| {
            matches!(
                node.runtime_state,
                RuntimeState::Online | RuntimeState::RuntimeOnly | RuntimeState::Conflict
            ) && node.endpoints.iter().any(|endpoint| {
                endpoint.kind == EndpointKind::Publisher
                    && absolute_topic(&endpoint.name) == absolute_topic(topic)
            })
        })
        .count()
}

impl HealthSnapshot {
    pub fn reset(&mut self, now: Instant) {
        self.topics.clear();
        self.updated = Some(now);
    }

    pub fn is_stale(&self, now: Instant) -> bool {
        self.updated
            .is_some_and(|updated| now.saturating_duration_since(updated) > Duration::from_secs(3))
    }
    pub fn apply(
        &mut self,
        report: HealthReport,
        expected: &EntityId,
        watches: &[TopicWatch],
        now: Instant,
    ) -> bool {
        if &report.project_id != expected {
            return false;
        }
        self.topics = report
            .topics
            .into_iter()
            .filter(|topic| {
                watches
                    .iter()
                    .any(|watch| watch == &topic.settings && watch.topic == topic.topic)
            })
            .filter(|topic| {
                topic
                    .observed_hz
                    .is_none_or(|hz| hz.is_finite() && hz >= 0.0)
            })
            .map(|topic| (topic.topic.clone(), topic))
            .collect();
        self.updated = Some(now);
        true
    }

    pub fn get(&self, topic: &str, now: Instant) -> Option<&TopicHealth> {
        self.updated
            .filter(|updated| now.saturating_duration_since(*updated) <= Duration::from_secs(3))?;
        self.topics.get(&absolute_topic(topic))
    }

    pub fn node_state(
        &self,
        project: &Project,
        node: &Node,
        watches: &[TopicWatch],
        now: Instant,
    ) -> Option<TopicHealthState> {
        if self.is_stale(now)
            || !matches!(
                node.runtime_state,
                RuntimeState::Online | RuntimeState::RuntimeOnly
            )
        {
            return None;
        }
        node.endpoints
            .iter()
            .filter(|endpoint| endpoint.kind == EndpointKind::Publisher)
            .filter(|endpoint| {
                watches
                    .iter()
                    .any(|watch| watch.topic == absolute_topic(&endpoint.name))
            })
            .map(|endpoint| {
                let topic = absolute_topic(&endpoint.name);
                let publishers = publisher_count(project, &topic);
                if publishers == 1 {
                    self.get(&topic, now)
                        .map_or(TopicHealthState::Waiting, |report| report.state)
                } else {
                    TopicHealthState::Unavailable
                }
            })
            .max_by_key(|state| match state {
                TopicHealthState::Healthy => 0,
                TopicHealthState::Waiting => 1,
                TopicHealthState::Unavailable => 2,
                TopicHealthState::Degraded => 3,
                TopicHealthState::Missing => 4,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn discovered_project() -> Project {
        use ros_studio_model::{Confidence, Endpoint};
        Project {
            id: EntityId::new("project:auto"),
            name: "auto".into(),
            root_path: "/auto".into(),
            packages: vec![],
            nodes: vec![Node {
                id: EntityId::new("node:camera"),
                logical_name: "camera".into(),
                package_id: EntityId::new("package:runtime"),
                executable: String::new(),
                source_locations: vec![],
                evidence: vec![],
                runtime_state: RuntimeState::RuntimeOnly,
                endpoints: ["/image", "/rosout", "/parameter_events"]
                    .into_iter()
                    .map(|topic| Endpoint {
                        id: EntityId::new(topic),
                        kind: EndpointKind::Publisher,
                        name: topic.into(),
                        type_name: "sensor_msgs/msg/Image".into(),
                        source_location: None,
                        confidence: Confidence::RuntimeConfirmed,
                        evidence: vec![],
                    })
                    .collect(),
            }],
        }
    }

    #[test]
    fn discovers_topics_without_settings_and_keeps_disappeared_publishers_visible() {
        let mut project = discovered_project();
        let watches = automatic_watches(&project, &[], &[]);
        assert_eq!(watches.len(), 1);
        assert_eq!(watches[0].topic, "/image");
        assert_eq!(watches[0].expected_hz, None);
        assert!(validate_watches(&watches).is_ok());
        project.nodes.clear();
        assert_eq!(automatic_watches(&project, &[], &watches), watches);
        assert!(automatic_watches(&project, &[], &[]).is_empty());
    }

    #[test]
    fn saved_thresholds_override_automatic_rates_and_reset_without_removing_the_topic() {
        let mut project = discovered_project();
        let auto = automatic_watches(&project, &[], &[]);
        let manual = TopicWatch {
            expected_hz: Some(30.0),
            ..auto[0].clone()
        };
        let watches = automatic_watches(&project, std::slice::from_ref(&manual), &auto);
        assert_eq!(watches, vec![manual]);
        assert_eq!(automatic_watches(&project, &[], &watches), auto);
        project.nodes[0].endpoints[0].kind = EndpointKind::Subscription;
        assert!(automatic_watches(&project, &[], &[]).is_empty());
        project.nodes[0].endpoints[0].kind = EndpointKind::Publisher;
        project.nodes[0].runtime_state = RuntimeState::Offline;
        assert!(automatic_watches(&project, &[], &[]).is_empty());
    }

    #[test]
    fn bounds_automatic_subscriptions_and_prioritizes_saved_topics() {
        let mut project = discovered_project();
        let endpoint = project.nodes[0].endpoints[0].clone();
        project.nodes[0].endpoints = (0..=MAX_TOPIC_WATCHES)
            .map(|i| ros_studio_model::Endpoint {
                name: format!("/topic_{i}"),
                ..endpoint.clone()
            })
            .collect();
        let saved = TopicWatch {
            topic: "/zzz_important".into(),
            type_name: endpoint.type_name,
            expected_hz: Some(30.0),
            tolerance: 0.2,
            timeout_ms: 5000,
        };
        let watches = automatic_watches(&project, std::slice::from_ref(&saved), &[]);
        assert_eq!(watches.len(), MAX_TOPIC_WATCHES);
        assert!(watches.contains(&saved));
        assert!(validate_watches(&watches).is_ok());
    }

    #[test]
    fn only_attributes_unique_publisher_evidence_and_never_subscriber_processing() {
        use ros_studio_model::{Confidence, Endpoint};
        let now = Instant::now();
        let watch = TopicWatch {
            topic: "/imu".into(),
            type_name: "sensor_msgs/msg/Imu".into(),
            expected_hz: Some(200.0),
            tolerance: 0.2,
            timeout_ms: 3000,
        };
        let publisher = Node {
            id: EntityId::new("node:publisher"),
            logical_name: "publisher".into(),
            package_id: EntityId::new("package:test"),
            executable: "publisher".into(),
            source_locations: vec![],
            evidence: vec![],
            runtime_state: RuntimeState::Online,
            endpoints: vec![Endpoint {
                id: EntityId::new("endpoint:imu"),
                kind: EndpointKind::Publisher,
                name: "/imu".into(),
                type_name: watch.type_name.clone(),
                source_location: None,
                confidence: Confidence::RuntimeConfirmed,
                evidence: vec![],
            }],
        };
        let mut project = Project {
            id: EntityId::new("project:test"),
            name: "test".into(),
            root_path: "/test".into(),
            packages: vec![],
            nodes: vec![publisher.clone()],
        };
        let report = TopicHealth {
            topic: watch.topic.clone(),
            settings: watch.clone(),
            observed_hz: Some(200.0),
            age_ms: Some(0),
            state: TopicHealthState::Healthy,
            detail: "received".into(),
        };
        let mut snapshot = HealthSnapshot::default();
        snapshot.apply(
            HealthReport {
                project_id: project.id.clone(),
                topics: vec![report.clone()],
            },
            &project.id,
            std::slice::from_ref(&watch),
            now,
        );
        assert_eq!(
            snapshot.node_state(&project, &publisher, std::slice::from_ref(&watch), now),
            Some(TopicHealthState::Healthy)
        );
        let mut subscriber = publisher.clone();
        subscriber.id = EntityId::new("node:subscriber");
        subscriber.endpoints[0].kind = EndpointKind::Subscription;
        project.nodes.push(subscriber.clone());
        assert_eq!(
            snapshot.node_state(&project, &subscriber, std::slice::from_ref(&watch), now),
            None
        );
        let mut second = publisher.clone();
        second.id = EntityId::new("node:second-publisher");
        project.nodes.push(second);
        assert_eq!(
            snapshot.node_state(&project, &publisher, std::slice::from_ref(&watch), now),
            Some(TopicHealthState::Unavailable)
        );
        project.nodes.last_mut().unwrap().runtime_state = RuntimeState::Conflict;
        assert_eq!(
            snapshot.node_state(&project, &publisher, std::slice::from_ref(&watch), now),
            Some(TopicHealthState::Unavailable)
        );
        project.nodes.pop();
        assert_eq!(
            snapshot.node_state(
                &project,
                &publisher,
                std::slice::from_ref(&watch),
                now + Duration::from_secs(4)
            ),
            None
        );
        let changed = TopicWatch {
            expected_hz: Some(100.0),
            ..watch
        };
        snapshot.apply(
            HealthReport {
                project_id: project.id.clone(),
                topics: vec![report],
            },
            &project.id,
            &[changed],
            now,
        );
        assert!(snapshot.get("/imu", now).is_none());
    }

    #[test]
    fn ignores_other_projects_unknown_topics_and_stale_reports() {
        let project = EntityId::new("project:test");
        let now = Instant::now();
        let watch = TopicWatch {
            topic: "/imu".into(),
            type_name: "sensor_msgs/msg/Imu".into(),
            expected_hz: Some(200.0),
            tolerance: 0.2,
            timeout_ms: 3000,
        };
        let topic = TopicHealth {
            topic: "/imu".into(),
            settings: watch.clone(),
            observed_hz: Some(200.0),
            age_ms: Some(0),
            state: TopicHealthState::Healthy,
            detail: "received".into(),
        };
        let mut snapshot = HealthSnapshot::default();
        assert!(!snapshot.apply(
            HealthReport {
                project_id: EntityId::new("project:other"),
                topics: vec![topic.clone()]
            },
            &project,
            std::slice::from_ref(&watch),
            now
        ));
        assert!(snapshot.get("/imu", now).is_none());
        assert!(snapshot.apply(
            HealthReport {
                project_id: project.clone(),
                topics: vec![topic]
            },
            &project,
            std::slice::from_ref(&watch),
            now
        ));
        assert_eq!(
            snapshot.get("imu", now).map(|report| report.state),
            Some(TopicHealthState::Healthy)
        );
        assert!(snapshot.get("/imu", now + Duration::from_secs(4)).is_none());
        snapshot.apply(
            HealthReport {
                project_id: project.clone(),
                topics: vec![],
            },
            &project,
            &[watch],
            now,
        );
        assert!(snapshot.get("/imu", now).is_none());
    }
}
