//! ROS adapter: subscriptions share the discovery node/executor, so monitoring
//! stays in the same ROS domain/environment on Host and Docker alike.

use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
    time::Instant,
};

use rclrs::{DynamicSubscription, IntoPrimitiveOptions as _, Node};
use ros_studio_model::EntityId;
use ros_studio_protocol::health::{HealthReport, TopicWatch};

use crate::health::TopicMeter;

struct ObservedTopic {
    meter: Arc<Mutex<TopicMeter>>,
    // Keep the subscription alive until removed, reconfigured or disconnected.
    _subscription: Option<DynamicSubscription>,
}

pub(crate) struct RuntimeHealth {
    project_id: EntityId,
    started: Instant,
    topics: BTreeMap<String, ObservedTopic>,
    pub configured: bool,
}

impl RuntimeHealth {
    pub fn new(project_id: EntityId) -> Self {
        Self {
            project_id,
            started: Instant::now(),
            topics: BTreeMap::new(),
            configured: false,
        }
    }

    pub fn configure(&mut self, node: &Node, watches: Vec<TopicWatch>) -> anyhow::Result<()> {
        self.configured = true;
        self.topics.retain(|topic, observed| {
            observed.meter.lock().is_ok_and(|meter| {
                watches
                    .iter()
                    .any(|watch| &watch.topic == topic && watch == &meter.watch)
            })
        });
        for watch in watches {
            if self.topics.contains_key(&watch.topic) {
                continue;
            }
            let topic = watch.topic.clone();
            let meter = Arc::new(Mutex::new(TopicMeter::new(
                watch.clone(),
                self.started.elapsed(),
            )));
            let callback_meter = meter.clone();
            let clock = self.started;
            let subscription = watch
                .type_name
                .as_str()
                .try_into()
                .map_err(anyhow::Error::from)
                .and_then(|message_type| {
                    node.create_dynamic_subscription(
                        message_type,
                        topic.as_str().best_effort().volatile().keep_last(10),
                        move |_message, _info| {
                            if let Ok(mut meter) = callback_meter.lock() {
                                meter.record(clock.elapsed());
                            }
                        },
                    )
                    .map_err(anyhow::Error::from)
                });
            let subscription = match subscription {
                Ok(subscription) => Some(subscription),
                Err(error) => {
                    meter.lock().map_err(|_| anyhow::anyhow!("Health meter poisoned"))?
                        .unavailable(format!("Cannot observe {topic}: {error:#}. Check message type support in this ROS environment."));
                    None
                }
            };
            self.topics.insert(
                topic,
                ObservedTopic {
                    meter,
                    _subscription: subscription,
                },
            );
        }
        Ok(())
    }

    pub fn report(&self) -> anyhow::Result<HealthReport> {
        let mut topics = Vec::new();
        for topic in self.topics.values() {
            topics.push(
                topic
                    .meter
                    .lock()
                    .map_err(|_| anyhow::anyhow!("Health meter poisoned"))?
                    .report(self.started.elapsed()),
            );
        }
        Ok(HealthReport {
            project_id: self.project_id.clone(),
            topics,
        })
    }
}
