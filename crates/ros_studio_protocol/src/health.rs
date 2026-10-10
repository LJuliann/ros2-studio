//! Configuration and evidence for passive topic monitoring. A topic's received
//! rate is not proof that any particular publisher, subscriber or algorithm is healthy.

use std::collections::BTreeSet;

use ros_studio_model::EntityId;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TopicWatch {
    pub topic: String,
    pub type_name: String,
    pub expected_hz: f64,
    pub tolerance: f64,
    pub timeout_ms: u64,
}

pub fn validate_watches(watches: &[TopicWatch]) -> Result<(), String> {
    if watches.len() > 32 {
        return Err("Monitor at most 32 important topics".into());
    }
    let mut names = BTreeSet::new();
    for watch in watches {
        let components = watch.topic.split('/').skip(1).collect::<Vec<_>>();
        let valid_component = |value: &str| {
            !value.is_empty()
                && value
                    .bytes()
                    .next()
                    .is_some_and(|c| c.is_ascii_alphabetic() || c == b'_')
                && value
                    .bytes()
                    .all(|c| c.is_ascii_alphanumeric() || c == b'_')
        };
        let parts = watch.type_name.split('/').collect::<Vec<_>>();
        if !watch.topic.starts_with('/')
            || watch.topic.len() > 256
            || !components.iter().all(|c| valid_component(c))
            || !names.insert(&watch.topic)
            || parts.len() != 3
            || parts[1] != "msg"
            || !valid_component(parts[0])
            || !valid_component(parts[2])
            || !watch.expected_hz.is_finite()
            || !(0.0 < watch.expected_hz && watch.expected_hz <= 100_000.0)
            || !watch.tolerance.is_finite()
            || !(0.0..1.0).contains(&watch.tolerance)
            || !(100..=600_000).contains(&watch.timeout_ms)
        {
            return Err(format!("Invalid monitoring settings for {}", watch.topic));
        }
        if watch.timeout_ms as f64 <= 1000.0 / watch.expected_hz {
            return Err(format!(
                "Silence timeout for {} must exceed its normal message interval",
                watch.topic
            ));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TopicHealthState {
    Waiting,
    Healthy,
    Degraded,
    Missing,
    Unavailable,
}

impl TopicHealthState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Waiting => "WAITING",
            Self::Healthy => "RATE OK",
            Self::Degraded => "RATE WARNING",
            Self::Missing => "NO DATA",
            Self::Unavailable => "UNAVAILABLE",
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct TopicHealth {
    pub topic: String,
    /// Tie evidence to its thresholds/type; clients must not display an old
    /// green result under newly edited settings while reconfiguration is queued.
    pub settings: TopicWatch,
    pub observed_hz: Option<f64>,
    pub age_ms: Option<u64>,
    pub state: TopicHealthState,
    pub detail: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct HealthReport {
    pub project_id: EntityId,
    pub topics: Vec<TopicHealth>,
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn watch() -> TopicWatch {
        TopicWatch {
            topic: "/camera/image".into(),
            type_name: "sensor_msgs/msg/Image".into(),
            expected_hz: 30.0,
            tolerance: 0.2,
            timeout_ms: 3000,
        }
    }

    #[test]
    fn validates_limits_names_types_and_numeric_thresholds() {
        assert!(validate_watches(&[]).is_ok());
        assert!(validate_watches(&[watch()]).is_ok());
        assert!(validate_watches(&[watch(), watch()]).is_err());
        for topic in [
            "camera",
            "/",
            "/bad//name",
            "/1name",
            "/camera image",
            "/camera/$(shell)",
        ] {
            assert!(
                validate_watches(&[TopicWatch {
                    topic: topic.into(),
                    ..watch()
                }])
                .is_err()
            );
        }
        for hz in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                validate_watches(&[TopicWatch {
                    expected_hz: hz,
                    ..watch()
                }])
                .is_err()
            );
        }
        assert!(
            validate_watches(&[TopicWatch {
                tolerance: 1.0,
                ..watch()
            }])
            .is_err()
        );
        assert!(
            validate_watches(&[TopicWatch {
                type_name: "pkg/srv/Type".into(),
                ..watch()
            }])
            .is_err()
        );
        assert!(
            validate_watches(&[TopicWatch {
                timeout_ms: 0,
                ..watch()
            }])
            .is_err()
        );
        assert!(
            validate_watches(&[TopicWatch {
                expected_hz: 0.1,
                timeout_ms: 3000,
                ..watch()
            }])
            .is_err()
        );
        assert!(
            validate_watches(&[TopicWatch {
                expected_hz: 0.1,
                timeout_ms: 30000,
                ..watch()
            }])
            .is_ok()
        );
        let many = (0..33)
            .map(|i| TopicWatch {
                topic: format!("/topic_{i}"),
                ..watch()
            })
            .collect::<Vec<_>>();
        assert!(validate_watches(&many).is_err());
    }

    #[test]
    fn health_configuration_and_observation_round_trip() {
        use crate::{Event, EventMessage, Request, RequestMessage};
        let request = RequestMessage::new(
            3,
            Request::ConfigureHealth {
                topics: vec![watch()],
            },
        );
        let decoded: RequestMessage =
            serde_json::from_str(&serde_json::to_string(&request).unwrap()).unwrap();
        assert_eq!(decoded, request);
        let event = EventMessage::new(Event::HealthChanged {
            report: HealthReport {
                project_id: EntityId::new("project:test"),
                topics: vec![TopicHealth {
                    topic: watch().topic,
                    settings: watch(),
                    observed_hz: Some(30.0),
                    age_ms: Some(12),
                    state: TopicHealthState::Healthy,
                    detail: "Rate OK".into(),
                }],
            },
        });
        let decoded: EventMessage =
            serde_json::from_str(&serde_json::to_string(&event).unwrap()).unwrap();
        assert_eq!(decoded, event);
    }
}
