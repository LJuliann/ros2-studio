//! Clock-independent topic-rate evaluation. The ROS adapter only records arrival
//! times; tests exercise silence, recovery and thresholds without ROS installed.

use std::{collections::VecDeque, time::Duration};

use ros_studio_protocol::health::{TopicHealth, TopicHealthState, TopicWatch};

pub struct TopicMeter {
    pub watch: TopicWatch,
    started: Duration,
    samples: VecDeque<Duration>,
    error: Option<String>,
}

impl TopicMeter {
    pub fn new(watch: TopicWatch, started: Duration) -> Self {
        Self {
            watch,
            started,
            samples: VecDeque::new(),
            error: None,
        }
    }

    pub fn record(&mut self, now: Duration) {
        if self.samples.back().is_none_or(|previous| now >= *previous) {
            self.samples.push_back(now);
            // Bound memory even for very fast publishers or long observation windows.
            if self.samples.len() > 4096 {
                self.samples.pop_front();
            }
        }
    }

    pub fn unavailable(&mut self, detail: String) {
        self.error = Some(detail);
    }

    pub fn report(&mut self, now: Duration) -> TopicHealth {
        let mut timeout = Duration::from_millis(self.watch.timeout_ms);
        let cutoff = now.saturating_sub(Duration::from_secs(5));
        // Keep a preceding sample for the boundary interval, including slow topics.
        while self.samples.len() > 2 && self.samples[1] < cutoff {
            self.samples.pop_front();
        }
        let age = self.samples.back().map(|last| now.saturating_sub(*last));
        let observed_hz =
            self.samples
                .front()
                .zip(self.samples.back())
                .and_then(|(first, last)| {
                    let seconds = last.saturating_sub(*first).as_secs_f64();
                    (self.samples.len() >= 2 && seconds > 0.0)
                        .then(|| (self.samples.len() - 1) as f64 / seconds)
                });
        // A slow periodic topic must not turn red between normal publications.
        // Explicit thresholds keep the user's chosen silence deadline.
        if self.watch.expected_hz.is_none()
            && let Some(hz) = observed_hz.filter(|hz| *hz > 0.0)
        {
            timeout = timeout.max(Duration::from_secs_f64((3.0 / hz).min(600.0)));
        }
        let (state, detail, observed_hz) = if let Some(error) = &self.error {
            (TopicHealthState::Unavailable, error.clone(), None)
        } else if age.is_some_and(|age| age >= timeout)
            || (age.is_none() && now.saturating_sub(self.started) >= timeout)
        {
            (
                TopicHealthState::Missing,
                "No recent message received; check publisher, network and QoS".into(),
                Some(0.0),
            )
        } else if now.saturating_sub(self.started) < Duration::from_secs(5) || observed_hz.is_none()
        {
            (
                TopicHealthState::Waiting,
                "Collecting arrival-time samples".into(),
                observed_hz,
            )
        } else if self.watch.expected_hz.is_none() {
            (
                TopicHealthState::Healthy,
                "Messages received; no expected rate configured".into(),
                observed_hz,
            )
        } else if self.watch.expected_hz.is_some_and(|expected| {
            observed_hz.is_some_and(|hz| (hz - expected).abs() <= expected * self.watch.tolerance)
        }) {
            (
                TopicHealthState::Healthy,
                "Received rate within configured tolerance".into(),
                observed_hz,
            )
        } else {
            (
                TopicHealthState::Degraded,
                "Received rate outside configured tolerance".into(),
                observed_hz,
            )
        };
        TopicHealth {
            topic: self.watch.topic.clone(),
            settings: self.watch.clone(),
            observed_hz,
            age_ms: age.map(|age| age.as_millis().min(u64::MAX as u128) as u64),
            state,
            detail,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meter() -> TopicMeter {
        TopicMeter::new(
            TopicWatch {
                topic: "/imu".into(),
                type_name: "sensor_msgs/msg/Imu".into(),
                expected_hz: Some(200.0),
                tolerance: 0.2,
                timeout_ms: 3000,
            },
            Duration::ZERO,
        )
    }

    #[test]
    fn waits_then_reports_silence_without_claiming_a_node_crash() {
        let mut meter = meter();
        assert_eq!(
            meter.report(Duration::from_secs(1)).state,
            TopicHealthState::Waiting
        );
        let report = meter.report(Duration::from_secs(3));
        assert_eq!(report.state, TopicHealthState::Missing);
        assert_eq!(report.observed_hz, Some(0.0));
        assert_eq!(report.age_ms, None);
    }

    #[test]
    fn detects_expected_slow_fast_stalled_and_recovered_rates() {
        for (step_ms, expected) in [
            (5, TopicHealthState::Healthy),
            (20, TopicHealthState::Degraded),
            (2, TopicHealthState::Degraded),
        ] {
            let mut meter = meter();
            for time in (0..=6000).step_by(step_ms) {
                meter.record(Duration::from_millis(time));
            }
            assert_eq!(meter.report(Duration::from_secs(6)).state, expected);
            assert_eq!(
                meter.report(Duration::from_secs(9)).state,
                TopicHealthState::Missing
            );
            for time in (9000..=15000).step_by(5) {
                meter.record(Duration::from_millis(time));
            }
            assert_eq!(
                meter.report(Duration::from_secs(15)).state,
                TopicHealthState::Healthy
            );
        }
    }

    #[test]
    fn handles_low_rates_bounded_history_duplicate_times_and_missing_type_support() {
        let mut meter = meter();
        meter.watch.expected_hz = Some(0.1);
        meter.watch.timeout_ms = 30_000;
        meter.record(Duration::ZERO);
        meter.record(Duration::from_secs(10));
        assert_eq!(
            meter.report(Duration::from_secs(11)).state,
            TopicHealthState::Healthy
        );
        for _ in 0..10_000 {
            meter.record(Duration::from_secs(12));
        }
        assert_eq!(meter.samples.len(), 4096);
        assert_eq!(
            meter.report(Duration::from_secs(12)).state,
            TopicHealthState::Waiting
        );
        meter.unavailable("Missing message introspection library".into());
        assert_eq!(
            meter.report(Duration::from_secs(50)).state,
            TopicHealthState::Unavailable
        );
    }

    #[test]
    fn automatic_monitoring_measures_silence_and_recovery_without_a_target() {
        let mut meter = meter();
        meter.watch.expected_hz = None;
        for time in (0..=6000).step_by(50) {
            meter.record(Duration::from_millis(time));
        }
        let report = meter.report(Duration::from_secs(6));
        assert_eq!(report.state, TopicHealthState::Healthy);
        assert_eq!(report.observed_hz, Some(20.0));
        let report = meter.report(Duration::from_secs(9));
        assert_eq!(report.state, TopicHealthState::Missing);
        assert_eq!(report.observed_hz, Some(0.0));
        for time in (9000..=15000).step_by(50) {
            meter.record(Duration::from_millis(time));
        }
        assert_eq!(
            meter.report(Duration::from_secs(15)).state,
            TopicHealthState::Healthy
        );
    }

    #[test]
    fn automatic_silence_deadline_adapts_to_slow_topics() {
        let mut meter = meter();
        meter.watch.expected_hz = None;
        meter.record(Duration::ZERO);
        meter.record(Duration::from_secs(10));
        assert_eq!(
            meter.report(Duration::from_secs(29)).state,
            TopicHealthState::Healthy
        );
        assert_eq!(
            meter.report(Duration::from_secs(40)).state,
            TopicHealthState::Missing
        );
    }
}
