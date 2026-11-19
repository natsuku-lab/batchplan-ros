//! Search settings shared by motion planning, IK and Cartesian paths.

use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use batchplan::{IkOptions, JoinOptions, PlanOptions, ToolPathOptions};

macro_rules! search_fields {
    ($float:ident, $integer:ident, $boolean:ident, $options:ident) => {
        $float!("default_planning_time", $options.default_planning_time);
        $integer!("solutions_per_pose", $options.solutions_per_pose);
        $float!("solution_distance", $options.solution_distance);
        $integer!("ik.seeds", $options.ik.seeds);
        $integer!("ik.iterations", $options.ik.iterations);
        $integer!("ik.rng_seed", $options.ik.rng_seed);
        $float!("ik.damping", $options.ik.damping);
        $float!("ik.rotation_weight", $options.ik.rotation_weight);
        $float!("ik.max_step", $options.ik.max_step);
        $float!("ik.collision_step", $options.ik.collision_step);
        $float!("ik.position_tolerance", $options.ik.position_tolerance);
        $float!("ik.rotation_tolerance", $options.ik.rotation_tolerance);
        $float!("ik.collision.world", $options.ik.collision.world);
        $float!(
            "ik.collision.self_collision",
            $options.ik.collision.self_collision
        );
        $float!("ik.collision.margin", $options.ik.collision.margin);
        $float!(
            "ik.collision.self_margin",
            $options.ik.collision.self_margin
        );
        $integer!("planning.seeds", $options.planning.seeds);
        $integer!("planning.control_points", $options.planning.control_points);
        $integer!(
            "planning.samples_per_span",
            $options.planning.samples_per_span
        );
        $integer!("planning.iterations", $options.planning.iterations);
        $integer!("planning.history", $options.planning.history);
        $integer!(
            "planning.validate_substeps",
            $options.planning.validate_substeps
        );
        $integer!("planning.rng_seed", $options.planning.rng_seed);
        $float!("planning.initial_step", $options.planning.initial_step);
        $float!(
            "planning.acceleration_weight",
            $options.planning.acceleration_weight
        );
        $float!(
            "planning.velocity_weight",
            $options.planning.velocity_weight
        );
        $float!(
            "planning.collision.world",
            $options.planning.collision.world
        );
        $float!(
            "planning.collision.self_collision",
            $options.planning.collision.self_collision
        );
        $float!(
            "planning.collision.margin",
            $options.planning.collision.margin
        );
        $float!(
            "planning.collision.self_margin",
            $options.planning.collision.self_margin
        );
        $float!("planning.rrt.range", $options.planning.rrt.range);
        $float!("planning.rrt.resolution", $options.planning.rrt.resolution);
        $integer!(
            "planning.rrt.max_iterations",
            $options.planning.rrt.max_iterations
        );
        $integer!("planning.rrt.rng_seed", $options.planning.rrt.rng_seed);
        $float!("planning.rrt.clearance", $options.planning.rrt.clearance);
        $float!(
            "planning.rrt.clearance_ramp",
            $options.planning.rrt.clearance_ramp
        );
        $float!(
            "planning.shortcut.resolution",
            $options.planning.shortcut.resolution
        );
        $integer!(
            "planning.shortcut.passes",
            $options.planning.shortcut.passes
        );
        $float!(
            "planning.shortcut.min_change",
            $options.planning.shortcut.min_change
        );
        $float!(
            "planning.shortcut.clearance",
            $options.planning.shortcut.clearance
        );
        $float!(
            "planning.shortcut.clearance_ramp",
            $options.planning.shortcut.clearance_ramp
        );
        $boolean!(
            "planning.shortcut.straighten",
            $options.planning.shortcut.straighten
        );
        $float!("cartesian.step", $options.cartesian.step);
        $float!("cartesian.rotation_step", $options.cartesian.rotation_step);
        $float!("cartesian.precision", $options.cartesian.precision);
        $float!(
            "cartesian.rotation_precision",
            $options.cartesian.rotation_precision
        );
        $float!("cartesian.resolution", $options.cartesian.resolution);
        $integer!("cartesian.iterations", $options.cartesian.iterations);
        $float!("join.resolution", $options.join.resolution);
    };
}

/// A request's immutable settings, selected by its MoveIt planner configuration.
#[derive(Clone, Debug)]
pub struct PlannerSettings {
    /// Native search options used by this configuration.
    pub search: SearchOptions,
    /// Temporal waypoint spacing in seconds.
    pub trajectory_dt: f32,
}

/// The ROS parameter type that carries a setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A `double`.
    Float,
    /// An `integer`.
    Integer,
    /// A `bool`.
    Boolean,
}

impl PlannerSettings {
    /// Every setting's name, as in [`Self::parameters`], and the ROS parameter type for it.
    #[must_use]
    pub fn kinds() -> Vec<(&'static str, Kind)> {
        let mut kinds = vec![];
        macro_rules! float {
            ($name:literal, $value:expr) => {
                kinds.push(($name, Kind::Float));
            };
        }
        macro_rules! integer {
            ($name:literal, $value:expr) => {
                kinds.push(($name, Kind::Integer));
            };
        }
        macro_rules! boolean {
            ($name:literal, $value:expr) => {
                kinds.push(($name, Kind::Boolean));
            };
        }
        search_fields!(float, integer, boolean, options);
        float!("trajectory_dt", trajectory_dt);
        kinds
    }

    /// Every setting as text, by name: what MoveIt's planner-parameter services carry.
    #[must_use]
    pub fn parameters(&self) -> BTreeMap<String, String> {
        let mut values = BTreeMap::new();
        let options = &self.search;
        macro_rules! field {
            ($name:literal, $value:expr) => {
                values.insert($name.into(), $value.to_string());
            };
        }
        search_fields!(field, field, field, options);
        field!("trajectory_dt", self.trajectory_dt);
        values
    }

    /// A copy with `values` applied, or the first one that names no setting or does not parse.
    /// Whether the values can be planned with is the planner's to say, which asks batchplan.
    pub fn with_parameters(&self, values: &BTreeMap<String, String>) -> Result<Self> {
        let mut result = self.clone();
        let options = &mut result.search;
        let mut remaining = values.clone();
        macro_rules! field {
            ($name:literal, $value:expr) => {
                if let Some(value) = remaining.remove($name) {
                    $value = value
                        .parse()
                        .map_err(|e| anyhow::anyhow!("{}: {e}", $name))?;
                }
            };
        }
        search_fields!(field, field, field, options);
        field!("trajectory_dt", result.trajectory_dt);
        ensure!(
            remaining.is_empty(),
            "unknown planner parameters: {:?}",
            remaining.keys()
        );
        Ok(result)
    }

    /// Refuses values of the node's own settings that it cannot plan with. batchplan's options
    /// are batchplan's to check.
    pub(crate) fn check(&self) -> Result<()> {
        let search = &self.search;
        ensure!(
            self.trajectory_dt.is_finite() && self.trajectory_dt > 0.0,
            "trajectory_dt must be finite and positive"
        );
        ensure!(
            search.solutions_per_pose > 0,
            "solutions_per_pose must be positive"
        );
        ensure!(
            search.solution_distance.is_finite() && search.solution_distance >= 0.0,
            "solution_distance must be finite and not negative"
        );
        ensure!(
            search.default_planning_time > 0.0
                && std::time::Duration::try_from_secs_f64(search.default_planning_time).is_ok(),
            "default_planning_time must be positive and representable as a duration"
        );
        Ok(())
    }
}

/// Configurable BatchPlan searches; request time and attempts remain MoveIt controls.
#[derive(Clone, Debug)]
pub struct SearchOptions {
    /// Pose-goal and standalone IK searches.
    pub ik: IkOptions,
    /// Free-space path searches and optional trajectory optimization.
    pub planning: PlanOptions,
    /// Straight-line and Cartesian waypoint tracking.
    pub cartesian: ToolPathOptions,
    /// Motion-sequence blend collision checks.
    pub join: JoinOptions,
    /// Maximum distinct IK configurations planned toward per pose goal.
    pub solutions_per_pose: usize,
    /// Minimum joint-space distance between retained IK configurations.
    pub solution_distance: f32,
    /// Time budget in seconds when a request gives none.
    pub default_planning_time: f64,
}

impl Default for SearchOptions {
    fn default() -> Self {
        Self {
            ik: IkOptions::default(),
            planning: PlanOptions::default(),
            cartesian: ToolPathOptions::default(),
            join: JoinOptions::default(),
            solutions_per_pose: 4,
            solution_distance: 0.05,
            default_planning_time: 5.0,
        }
    }
}

impl SearchOptions {
    /// Diversifies retries without changing the configured first attempt.
    #[must_use]
    pub fn for_attempt(&self, attempt: u64) -> Self {
        let mut options = self.clone();
        let offset = attempt.wrapping_mul(104_729);
        options.ik.rng_seed = options.ik.rng_seed.wrapping_add(offset);
        options.planning.rng_seed = options.planning.rng_seed.wrapping_add(offset);
        options.planning.rrt.rng_seed = options.planning.rrt.rng_seed.wrapping_add(offset);
        options
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parameters_round_trip_and_an_update_that_does_not_parse_changes_nothing() {
        let settings = PlannerSettings {
            search: SearchOptions::default(),
            trajectory_dt: 0.01,
        };
        let original = settings.parameters();
        assert_eq!(
            settings.with_parameters(&original).unwrap().parameters(),
            original
        );
        for (key, value) in [
            ("ik.seeds", "-1"),
            ("ik.seeds", "many"),
            ("planning.shortcut.straighten", "1"),
            ("unknown", "1"),
        ] {
            let update = BTreeMap::from([
                ("solution_distance".into(), "0.2".into()),
                (key.into(), value.into()),
            ]);
            assert!(settings.with_parameters(&update).is_err(), "{key}");
        }
        let changed = settings
            .with_parameters(&BTreeMap::from([
                ("planning.collision.margin".into(), "0.03".into()),
                ("planning.shortcut.straighten".into(), "false".into()),
            ]))
            .unwrap();
        assert_eq!(changed.search.planning.collision.margin, 0.03);
        assert!(!changed.search.planning.shortcut.straighten);
    }

    #[test]
    fn the_nodes_own_settings_are_checked() {
        let settings = PlannerSettings {
            search: SearchOptions::default(),
            trajectory_dt: 0.02,
        };
        settings.check().unwrap();
        for (key, value) in [
            ("trajectory_dt", "inf"),
            ("trajectory_dt", "0"),
            ("solutions_per_pose", "0"),
            ("solution_distance", "NaN"),
            ("default_planning_time", "inf"),
            ("default_planning_time", "0"),
        ] {
            let update = BTreeMap::from([(key.into(), value.into())]);
            let changed = settings.with_parameters(&update).unwrap();
            assert!(changed.check().is_err(), "{key}={value}");
        }
    }

    #[test]
    fn a_retry_changes_every_seed_and_the_first_attempt_none() {
        let options = SearchOptions::default();
        let first = options.for_attempt(0);
        assert_eq!(first.ik.rng_seed, options.ik.rng_seed);
        assert_eq!(first.planning.rng_seed, options.planning.rng_seed);
        assert_eq!(first.planning.rrt.rng_seed, options.planning.rrt.rng_seed);
        let second = options.for_attempt(1);
        assert_ne!(second.ik.rng_seed, options.ik.rng_seed);
        assert_ne!(second.planning.rng_seed, options.planning.rng_seed);
        assert_ne!(second.planning.rrt.rng_seed, options.planning.rrt.rng_seed);
    }
}
