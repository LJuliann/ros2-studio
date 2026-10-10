//! Honest display of monitoring evidence: never attribute an aggregated topic's
//! rate to one of several publishers, or keep a green badge after reports stop.

use ros_studio_model::{EndpointKind, EntityId, Node, Project, RuntimeState};
use ros_studio_protocol::health::{HealthReport, TopicHealth, TopicHealthState, TopicWatch};
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
                let publishers = project
                    .nodes
                    .iter()
                    .filter(|candidate| {
                        matches!(
                            candidate.runtime_state,
                            RuntimeState::Online
                                | RuntimeState::RuntimeOnly
                                | RuntimeState::Conflict
                        ) && candidate.endpoints.iter().any(|endpoint| {
                            endpoint.kind == EndpointKind::Publisher
                                && absolute_topic(&endpoint.name) == topic
                        })
                    })
                    .count();
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

    #[test]
    fn only_attributes_unique_publisher_evidence_and_never_subscriber_processing() {
        use ros_studio_model::{Confidence, Endpoint};
        let now = Instant::now();
        let watch = TopicWatch {
            topic: "/imu".into(),
            type_name: "sensor_msgs/msg/Imu".into(),
            expected_hz: 200.0,
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
            expected_hz: 100.0,
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
            expected_hz: 200.0,
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
