//! MoveIt's `joint_limits.yaml` as batchplan joint limits.

use std::collections::HashMap;

use anyhow::{Context, Result};
use batchplan::JointLimits;
use serde::Deserialize;

#[derive(Deserialize)]
struct File {
    #[serde(default)]
    joint_limits: HashMap<String, Entry>,
}

#[derive(Default, Deserialize)]
#[serde(default)]
struct Entry {
    has_position_limits: bool,
    min_position: Option<f64>,
    max_position: Option<f64>,
    has_velocity_limits: bool,
    max_velocity: Option<f64>,
    has_acceleration_limits: bool,
    max_acceleration: Option<f64>,
    has_jerk_limits: bool,
    max_jerk: Option<f64>,
}

/// Each joint's limits that the file enables (`has_*_limits: true`).
pub fn parse(text: &str) -> Result<HashMap<String, JointLimits>> {
    let file: File = serde_saphyr::from_str(text).context("parsing joint_limits.yaml")?;
    let given =
        |enabled: bool, value: Option<f64>| value.filter(|&v| enabled && v > 0.0).map(|v| v as f32);
    Ok(file
        .joint_limits
        .into_iter()
        .map(|(name, e)| {
            let position = match (e.has_position_limits, e.min_position, e.max_position) {
                (true, Some(lo), Some(hi)) => Some([lo as f32, hi as f32]),
                _ => None,
            };
            let limits = JointLimits {
                position,
                velocity: given(e.has_velocity_limits, e.max_velocity),
                acceleration: given(e.has_acceleration_limits, e.max_acceleration),
                jerk: given(e.has_jerk_limits, e.max_jerk),
            };
            (name, limits)
        })
        .collect())
}
