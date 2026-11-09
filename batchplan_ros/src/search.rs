//! Search settings shared by motion planning, IK and Cartesian paths.

use anyhow::{Result, ensure};
use batchplan::{CollisionWeights, IkOptions, JoinOptions, PlanOptions, ToolPathOptions};
use rclrs::ReadOnlyParameter;
use std::collections::BTreeMap;

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
        $float!("ik.collision.self_collision", $options.ik.collision.self_collision);
        $float!("ik.collision.margin", $options.ik.collision.margin);
        $float!("ik.collision.self_margin", $options.ik.collision.self_margin);
        $integer!("planning.seeds", $options.planning.seeds);
        $integer!("planning.control_points", $options.planning.control_points);
        $integer!("planning.samples_per_span", $options.planning.samples_per_span);
        $integer!("planning.iterations", $options.planning.iterations);
        $integer!("planning.history", $options.planning.history);
        $integer!("planning.validate_substeps", $options.planning.validate_substeps);
        $integer!("planning.rng_seed", $options.planning.rng_seed);
        $float!("planning.initial_step", $options.planning.initial_step);
        $float!("planning.acceleration_weight", $options.planning.acceleration_weight);
        $float!("planning.velocity_weight", $options.planning.velocity_weight);
        $float!("planning.collision.world", $options.planning.collision.world);
        $float!("planning.collision.self_collision", $options.planning.collision.self_collision);
        $float!("planning.collision.margin", $options.planning.collision.margin);
        $float!("planning.collision.self_margin", $options.planning.collision.self_margin);
        $float!("planning.rrt.range", $options.planning.rrt.range);
        $float!("planning.rrt.resolution", $options.planning.rrt.resolution);
        $integer!("planning.rrt.max_iterations", $options.planning.rrt.max_iterations);
        $integer!("planning.rrt.rng_seed", $options.planning.rrt.rng_seed);
        $float!("planning.rrt.clearance", $options.planning.rrt.clearance);
        $float!("planning.rrt.clearance_ramp", $options.planning.rrt.clearance_ramp);
        $float!("planning.shortcut.resolution", $options.planning.shortcut.resolution);
        $integer!("planning.shortcut.passes", $options.planning.shortcut.passes);
        $float!("planning.shortcut.min_change", $options.planning.shortcut.min_change);
        $float!("planning.shortcut.clearance", $options.planning.shortcut.clearance);
        $float!("planning.shortcut.clearance_ramp", $options.planning.shortcut.clearance_ramp);
        $boolean!("planning.shortcut.straighten", $options.planning.shortcut.straighten);
        $float!("cartesian.step", $options.cartesian.step);
        $float!("cartesian.rotation_step", $options.cartesian.rotation_step);
        $float!("cartesian.precision", $options.cartesian.precision);
        $float!("cartesian.rotation_precision", $options.cartesian.rotation_precision);
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

impl PlannerSettings {
    /// The editable fields shown by MoveIt's planner-parameter interfaces.
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

    /// Applies a complete update to a copy; failures never change the active configuration.
    pub fn with_parameters(&self, values: &BTreeMap<String, String>) -> Result<Self> {
        let mut result = self.clone();
        let options = &mut result.search;
        let mut remaining = values.clone();
        macro_rules! field {
            ($name:literal, $value:expr) => {
                if let Some(value) = remaining.remove($name) {
                    $value = value.parse().map_err(|e| anyhow::anyhow!("{}: {e}", $name))?;
                }
            };
        }
        search_fields!(field, field, field, options);
        field!("trajectory_dt", result.trajectory_dt);
        ensure!(remaining.is_empty(), "unknown planner parameters: {:?}", remaining.keys());
        result.search.validate()?;
        ensure!(
            result.trajectory_dt.is_finite() && result.trajectory_dt > 0.0,
            "trajectory_dt must be finite and positive"
        );
        Ok(result)
    }
}

/// Keeps declared ROS settings visible and read-only for the node's lifetime.
#[derive(Debug)]
pub struct SearchParameters {
    /// Validated values copied into a planner at startup.
    pub options: SearchOptions,
    floats: Vec<ReadOnlyParameter<f64>>,
    integers: Vec<ReadOnlyParameter<i64>>,
    booleans: Vec<ReadOnlyParameter<bool>>,
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
    /// Rejects invalid settings before loading robot geometry or accepting requests.
    pub fn validate(&self) -> Result<()> {
        let positive = |name: &str, value: f32| -> Result<()> {
            ensure!(value.is_finite() && value > 0.0, "{name} must be finite and positive");
            Ok(())
        };
        let nonnegative = |name: &str, value: f32| -> Result<()> {
            ensure!(value.is_finite() && value >= 0.0, "{name} must be finite and nonnegative");
            Ok(())
        };
        let resolution = |name: &str, value: f32| -> Result<()> {
            ensure!(value.is_finite() && value >= 1e-4, "{name} must be finite and at least 0.0001");
            Ok(())
        };
        let weights = |name: &str, value: CollisionWeights| -> Result<()> {
            nonnegative(&format!("{name}.world"), value.world)?;
            nonnegative(&format!("{name}.self_collision"), value.self_collision)?;
            nonnegative(&format!("{name}.margin"), value.margin)?;
            nonnegative(&format!("{name}.self_margin"), value.self_margin)
        };
        ensure!(self.solutions_per_pose > 0, "solutions_per_pose must be positive");
        nonnegative("solution_distance", self.solution_distance)?;
        ensure!(
            self.default_planning_time > 0.0
                && std::time::Duration::try_from_secs_f64(self.default_planning_time).is_ok(),
            "default_planning_time must be positive and representable as a duration"
        );
        let ik = &self.ik;
        ensure!(ik.seeds > 0, "ik.seeds must be positive");
        ensure!(ik.iterations > 0, "ik.iterations must be positive");
        nonnegative("ik.damping", ik.damping)?;
        positive("ik.rotation_weight", ik.rotation_weight)?;
        positive("ik.max_step", ik.max_step)?;
        nonnegative("ik.collision_step", ik.collision_step)?;
        positive("ik.position_tolerance", ik.position_tolerance)?;
        positive("ik.rotation_tolerance", ik.rotation_tolerance)?;
        weights("ik.collision", ik.collision)?;
        let p = &self.planning;
        ensure!(p.control_points >= 7, "planning.control_points must be at least 7");
        ensure!(p.samples_per_span > 0, "planning.samples_per_span must be positive");
        ensure!(p.validate_substeps > 0, "planning.validate_substeps must be positive");
        ensure!((1..=8).contains(&p.history), "planning.history must be between 1 and 8");
        positive("planning.initial_step", p.initial_step)?;
        nonnegative("planning.acceleration_weight", p.acceleration_weight)?;
        nonnegative("planning.velocity_weight", p.velocity_weight)?;
        weights("planning.collision", p.collision)?;
        ensure!(p.time_budget.is_none(), "use default_planning_time or the request's allowed_planning_time");
        positive("planning.rrt.range", p.rrt.range)?;
        resolution("planning.rrt.resolution", p.rrt.resolution)?;
        nonnegative("planning.rrt.clearance", p.rrt.clearance)?;
        positive("planning.rrt.clearance_ramp", p.rrt.clearance_ramp)?;
        resolution("planning.shortcut.resolution", p.shortcut.resolution)?;
        // Infinity is the native way to disable smoothing and straightening.
        ensure!(p.shortcut.min_change > 0.0, "planning.shortcut.min_change must be positive");
        nonnegative("planning.shortcut.clearance", p.shortcut.clearance)?;
        positive("planning.shortcut.clearance_ramp", p.shortcut.clearance_ramp)?;
        let c = &self.cartesian;
        ensure!(c.step.is_finite() && c.step >= 1e-5, "cartesian.step must be finite and at least 0.00001");
        ensure!(c.rotation_step.is_finite() && c.rotation_step >= 1e-5, "cartesian.rotation_step must be finite and at least 0.00001");
        positive("cartesian.precision", c.precision)?;
        positive("cartesian.rotation_precision", c.rotation_precision)?;
        resolution("cartesian.resolution", c.resolution)?;
        ensure!(c.iterations > 0, "cartesian.iterations must be positive");
        resolution("join.resolution", self.join.resolution)?;
        Ok(())
    }

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

    /// Declares startup-only ROS parameters, with the native library's defaults.
    pub fn from_node(node: &rclrs::Node) -> Result<SearchParameters> {
        let mut options = Self::default();
        let mut parameters = SearchParameters {
            options: options.clone(),
            floats: vec![],
            integers: vec![],
            booleans: vec![],
        };
        macro_rules! float {
            ($name:literal, $field:expr) => {
                let parameter = node.declare_parameter::<f64>($name)
                    .default(f64::from($field)).read_only()?;
                $field = parameter.get() as _;
                parameters.floats.push(parameter);
            };
        }
        macro_rules! integer {
            ($name:literal, $field:expr) => {
                let parameter = node.declare_parameter::<i64>($name)
                    .default(i64::try_from($field)?).read_only()?;
                $field = parameter.get().try_into()
                    .map_err(|_| anyhow::anyhow!("{} is outside its nonnegative integer range", $name))?;
                parameters.integers.push(parameter);
            };
        }
        macro_rules! boolean {
            ($name:literal, $field:expr) => {
                let parameter = node.declare_parameter::<bool>($name)
                    .default($field).read_only()?;
                $field = parameter.get();
                parameters.booleans.push(parameter);
            };
        }
        search_fields!(float, integer, boolean, options);
        options.validate()?;
        parameters.options = options;
        Ok(parameters)
    }
}
