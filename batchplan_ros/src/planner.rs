//! MoveIt planning requests, each against the scene it carries.

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::hash::Hasher;
use std::ops::Range;
use std::path::PathBuf;
use std::sync::RwLock;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use batchplan::{
    CollisionModel, CollisionWeights, Contact, Device, HandConstraint, HandSpeed, IkOptions,
    IkProblem, JointLimits, JointTrajectory, MeshModel, PlanOptions, PlanProblem, PlanResult,
    Robot, RobotOptions, SdfOptions, Sequence, SphereOptions, ToolPath, ToolPathOptions,
    ToolPathStop, Trajectory, World, Worlds, solve_ik,
};
use ros_env::moveit_msgs::msg::{
    AttachedCollisionObject, MotionPlanRequest, MotionSequenceRequest, MotionSequenceResponse,
    MoveItErrorCodes, PlanningScene, PositionIKRequest, RobotState, RobotTrajectory,
};
use ros_env::moveit_msgs::srv::{GetCartesianPath_Request, GetCartesianPath_Response};
use ros_env::sensor_msgs::msg::JointState;

use crate::goals::{self, Goal};
use crate::scene::{self, Fnv, Frames, Grids};
use crate::srdf::{Srdf, Tree};
use crate::{
    PlannerSettings, Refusal, SearchOptions, constraints, state, trajectory, workcell,
    world_geometry,
};

/// The batchplan release whose sphere fit the cache holds: spheres another one fitted are fitted
/// again. Bump it with a release that changes the fit, as 0.9.0 and 0.10.0 did.
const FIT: &str = "batchplan 0.10 fit";

type Code = MoveItErrorCodes;

/// Where batches run.
#[derive(Clone, Copy, Debug)]
pub enum DeviceKind {
    /// CPU threads.
    Cpu {
        /// How many; `0` uses every core.
        threads: usize,
    },
    /// The first GPU wgpu finds; searches still run on CPU threads.
    Gpu,
}

/// How a [`Planner`] loads its robot and answers requests.
#[derive(Debug)]
pub struct PlannerOptions {
    /// Directories holding the packages `package://` paths name (each prefix's `share/`).
    pub package_dirs: Vec<PathBuf>,
    /// Limits replacing the URDF's, by joint name, as in MoveIt's `joint_limits.yaml`.
    pub joint_limits: HashMap<String, JointLimits>,
    /// IK frames; the SRDF's end effectors and chain tips when empty.
    pub ee_links: Vec<String>,
    /// Where batches run.
    pub device: DeviceKind,
    /// Where fitted collision spheres are kept between runs.
    pub cache_dir: Option<PathBuf>,
    /// Roots whose strict descendants own robot collision geometry; empty selects all links.
    pub robot_link_roots: Vec<String>,
    /// Maximum grid spacing in meters for URDF world meshes.
    pub world_sdf_voxel_size: f32,
    /// Spheres shared by the selected robot links.
    pub sphere_budget: usize,
    /// Spacing of returned waypoints, in seconds.
    pub trajectory_dt: f32,
    /// IK, free-space and Cartesian search settings.
    pub search: SearchOptions,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            package_dirs: vec![],
            joint_limits: HashMap::new(),
            ee_links: vec![],
            device: DeviceKind::Cpu { threads: 0 },
            cache_dir: None,
            sphere_budget: SphereOptions::default().budget,
            robot_link_roots: vec![],
            world_sdf_voxel_size: SdfOptions::default().voxel,
            trajectory_dt: 0.02,
            search: SearchOptions::default(),
        }
    }
}

/// What a request comes to: an error code, and on success the start and the trajectory.
#[derive(Debug)]
pub struct Answer {
    /// `SUCCESS`, or why the request failed.
    pub error_code: MoveItErrorCodes,
    /// The state the trajectory starts from.
    pub trajectory_start: RobotState,
    /// The timed trajectory; empty unless the request succeeded.
    pub trajectory: RobotTrajectory,
    /// Seconds spent answering.
    pub planning_time: f64,
}

/// Positions this close outside a limit are moved onto it.
const LIMIT_TOLERANCE: f32 = 1e-3;
/// The first round's time slice for constrained goals, as a fraction of the request's budget.
const FIRST_SLICE: u32 = 16;
/// The planner id that asks for the hand to move in a straight line to its goal, as Pilz's
/// industrial motion planner names it.
const STRAIGHT: &str = "LIN";

/// What a request's motions are planned in: the robot holding the scene's attached objects, its
/// device and meshes, the scene's world, and the states that place the robot's frames.
struct Setting<'a> {
    robot: &'a Robot,
    device: &'a Device,
    meshes: &'a MeshModel,
    world: &'a World,
    worlds: &'a Worlds,
    states: [&'a RobotState; 2],
    scene: &'a PlanningScene,
    search: &'a SearchOptions,
    dt: f32,
}

/// One planned motion: the joints it moves, its path (B-spline control points, empty for one that
/// stays where it starts), how its timing is slowed and capped, and the trajectory so timed.
struct Motion {
    group: Vec<usize>,
    path: Vec<f32>,
    speed: f32,
    cap: Option<HandSpeed>,
    /// What the hands keep along it, which a join passing into or out of it keeps too.
    constraints: Vec<HandConstraint>,
    sampled: JointTrajectory,
}

impl Motion {
    /// Where it ends: its path's last control point, or where it stays.
    fn end(&self) -> &[f32] {
        let n = self.sampled.dof;
        if self.path.is_empty() {
            &self.sampled.positions[..n]
        } else {
            &self.path[self.path.len() - n..]
        }
    }
}

/// Answers MoveIt planning and IK requests for one robot with batchplan.
pub struct Planner {
    robot: Robot,
    device: Device,
    meshes: MeshModel,
    tree: Tree,
    srdf: Srdf,
    root: String,
    /// SRDF virtual joints: name and parent frame.
    virtual_joints: Vec<(String, String)>,
    hands: Vec<String>,
    grids: Grids,
    dt: f32,
    search: SearchOptions,
    configurations: RwLock<HashMap<(String, String), PlannerSettings>>,
    world_links: Vec<scene::WorldLink>,
    world_markers: Vec<ros_env::visualization_msgs::msg::Marker>,
    world_exemptions: Vec<(String, String)>,
}

fn check_sphere_budget(model: &CollisionModel, budget: usize) -> Result<usize> {
    let count: usize = model.spheres.values().map(Vec::len).sum();
    anyhow::ensure!(
        count <= budget,
        "URDF collision model uses {count} spheres, exceeding sphere_budget={budget}; increase the budget"
    );
    Ok(count)
}

impl fmt::Debug for Planner {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Planner")
            .field("robot", &self.robot.name())
            .field("device", &self.device.name())
            .finish_non_exhaustive()
    }
}

impl Planner {
    /// Loads the robot from its URDF and SRDF text: the SRDF's disabled collision pairs and end
    /// effectors, collision spheres fitted once and kept in `o.cache_dir`, and the limits given.
    pub fn try_new(urdf: &str, srdf: Option<&str>, o: &PlannerOptions) -> Result<Self> {
        Self::try_new_with_scene(urdf, srdf, o, |_, _| Ok(()))
    }

    /// Reports robot spheres and URDF world markers before planning-data allocation.
    /// Reporting failures are logged without preventing planning.
    pub fn try_new_with_scene(
        urdf: &str,
        srdf: Option<&str>,
        o: &PlannerOptions,
        mut report: impl FnMut(
            &CollisionModel,
            &[ros_env::visualization_msgs::msg::Marker],
        ) -> Result<()>,
    ) -> Result<Self> {
        o.search.validate()?;
        anyhow::ensure!(
            o.trajectory_dt.is_finite() && o.trajectory_dt > 0.0,
            "trajectory_dt must be finite and positive"
        );
        let tree = Tree::parse(urdf)?;
        anyhow::ensure!(
            o.sphere_budget > 0,
            "sphere_budget must be a positive integer"
        );
        let parsed = srdf.map(Srdf::parse).transpose()?.unwrap_or_default();
        let partition = workcell::Partition::new(urdf, &o.robot_link_roots)?;
        let robot_urdf = partition.robot_urdf.as_str();
        let doc = roxmltree::Document::parse(robot_urdf)?;
        let minimum = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("link"))
            .filter(|n| {
                n.children()
                    .any(|c| c.has_tag_name("collision") || c.has_tag_name("visual"))
            })
            .count();
        anyhow::ensure!(
            o.sphere_budget >= minimum,
            "sphere_budget={} is below the minimum of {} geometry-bearing robot links",
            o.sphere_budget,
            minimum
        );
        let (world_links, world_markers) = world_geometry::world_model(
            urdf,
            partition.robot_links.as_ref(),
            &o.package_dirs,
            &SdfOptions {
                voxel: o.world_sdf_voxel_size,
                ..Default::default()
            },
        )?;
        let mut world_exemptions = partition.exemptions;
        world_exemptions.extend(
            parsed
                .disabled_collisions()
                .iter()
                .filter(|(a, b)| world_links.iter().any(|l| l.name == *a || l.name == *b))
                .cloned(),
        );
        let hands = if o.ee_links.is_empty() {
            parsed.tips()
        } else {
            o.ee_links.clone()
        };
        if let Some(links) = &partition.robot_links {
            for hand in &hands {
                anyhow::ensure!(
                    links.contains(hand),
                    "IK frame '{hand}' is outside robot_link_roots descendants"
                );
            }
        }
        let mut options = RobotOptions {
            ee_links: hands.clone(),
            package_dirs: o.package_dirs.clone(),
            joint_limits: o.joint_limits.clone(),
            srdf: srdf.map(str::to_string),
            spheres: SphereOptions {
                budget: o.sphere_budget,
                ..Default::default()
            },
            ..Default::default()
        };
        let cached = o.cache_dir.as_ref().map(|dir| {
            let mut hash = Fnv::default();
            hash.write(FIT.as_bytes());
            hash.write(robot_urdf.as_bytes());
            hash.write(srdf.unwrap_or_default().as_bytes());
            hash.write(format!("{:?}", options.spheres).as_bytes());
            dir.join(format!("{:016x}.json", hash.finish()))
        });
        if let Some(path) = cached.as_ref().filter(|p| p.exists()) {
            let model = CollisionModel::load(path)?;
            check_sphere_budget(&model, o.sphere_budget)?;
            options.collision_model = Some(model);
            // The cached model already holds the SRDF's pairs.
            options.srdf = None;
        }
        let robot = Robot::from_urdf(robot_urdf, &options)?;
        let count = check_sphere_budget(robot.collision_model(), o.sphere_budget)?;
        rclrs::log_info!(
            "batchplan_planner",
            "budgeted collision model: {} robot spheres (maximum {})",
            count,
            o.sphere_budget
        );
        if let Err(e) = report(robot.collision_model(), &world_markers) {
            rclrs::log_error!(
                "batchplan_planner",
                "cannot publish collision visualization: {e:#}"
            );
        }
        if let Some(path) = cached.filter(|_| options.collision_model.is_none()) {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)
                    .with_context(|| format!("creating {}", dir.display()))?;
            }
            robot.collision_model().save(&path)?;
        }
        let device = match o.device {
            DeviceKind::Cpu { threads: 0 } => Device::cpu(&robot)?,
            DeviceKind::Cpu { threads } => Device::cpu_threads(&robot, threads)?,
            DeviceKind::Gpu => Device::gpu(&robot)?,
        };
        let meshes = MeshModel::try_new(&robot)?;
        let root = tree
            .root()
            .context("the URDF has no root link")?
            .to_string();
        let virtual_joints = parsed.virtual_joints().to_vec();
        Ok(Self {
            robot,
            device,
            meshes,
            tree,
            srdf: parsed,
            root,
            virtual_joints,
            hands,
            grids: Grids::new(SdfOptions::default()),
            dt: o.trajectory_dt,
            search: o.search.clone(),
            configurations: RwLock::new(HashMap::new()),
            world_links,
            world_markers,
            world_exemptions,
        })
    }

    /// The robot it plans for.
    #[must_use]
    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// Full link-local collision model for a frame-locked RViz MarkerArray display.
    #[must_use]
    pub fn collision_markers(&self) -> ros_env::visualization_msgs::msg::MarkerArray {
        crate::visualization::scene(self.robot.collision_model(), &self.world_markers)
    }

    /// Native URDF world obstacles, grouped by link and expressed in that link's frame.
    pub fn urdf_world_links(&self) -> impl Iterator<Item = (&str, &[batchplan::Obstacle])> {
        self.world_links
            .iter()
            .map(|l| (l.name.as_str(), l.obstacles.as_slice()))
    }

    /// What runs the batches: the GPU adapter, or the CPU's thread count.
    #[must_use]
    pub fn device_name(&self) -> String {
        self.device.name()
    }

    /// Settings are copied before planning; edits cannot change an in-flight motion.
    pub fn planner_settings(&self, group: &str, planner: &str) -> Result<PlannerSettings> {
        let planner = self.configuration_name(group, planner)?;
        let configs = self
            .configurations
            .read()
            .map_err(|_| anyhow::anyhow!("planner configuration lock poisoned"))?;
        Ok(configs
            .get(&(group.into(), planner.into()))
            .or_else(|| configs.get(&(String::new(), planner.into())))
            .cloned()
            .unwrap_or_else(|| PlannerSettings {
                search: self.search.clone(),
                trajectory_dt: self.dt,
            }))
    }

    /// Standard MoveIt configuration updates are validated atomically.
    pub fn set_planner_parameters(
        &self,
        group: &str,
        planner: &str,
        values: &BTreeMap<String, String>,
        replace: bool,
    ) -> Result<()> {
        let planner = self.configuration_name(group, planner)?;
        let mut configs = self
            .configurations
            .write()
            .map_err(|_| anyhow::anyhow!("planner configuration lock poisoned"))?;
        let key = (group.into(), planner.into());
        let defaults = PlannerSettings {
            search: self.search.clone(),
            trajectory_dt: self.dt,
        };
        let previous = if replace {
            &defaults
        } else {
            configs
                .get(&key)
                .or_else(|| configs.get(&(String::new(), planner.into())))
                .unwrap_or(&defaults)
        };
        let settings = previous.with_parameters(values)?;
        configs.insert(key, settings);
        Ok(())
    }

    fn configuration_name<'a>(&self, group: &str, planner: &'a str) -> Result<&'a str> {
        if !group.is_empty() {
            self.group(group)
                .map_err(|e| anyhow::anyhow!("{}", e.message))?;
        }
        let planner = if planner.is_empty() {
            "batchplan"
        } else {
            planner
        };
        anyhow::ensure!(
            matches!(planner, "batchplan" | STRAIGHT),
            "unknown planner '{planner}'"
        );
        Ok(planner)
    }

    /// Plans `request` in `scene`, whose objects augment the URDF world. The start is
    /// `current` (the latest joint states), overridden by the scene's robot state and then the
    /// request's start state.
    pub fn plan(
        &self,
        request: &MotionPlanRequest,
        scene: &PlanningScene,
        current: Option<&JointState>,
    ) -> Answer {
        let begun = Instant::now();
        let outcome = self.try_sequence(&[(request, 0.0)], scene, current, begun);
        let planning_time = begun.elapsed().as_secs_f64();
        match outcome {
            Ok((trajectory_start, mut trajectories)) => Answer {
                error_code: code(Code::SUCCESS, ""),
                trajectory_start,
                trajectory: trajectories.pop().unwrap_or_default(),
                planning_time,
            },
            Err(r) => Answer {
                error_code: code(r.code, &r.message),
                trajectory_start: RobotState::default(),
                trajectory: RobotTrajectory::default(),
                planning_time,
            },
        }
    }

    /// Plans `request`'s items one after another in `scene`, as Pilz's sequence planner does: each
    /// starts where the one before ends, the first as [`plan`](Self::plan) starts. There is one
    /// trajectory per run of items planning one group, which passes each join whose blend radius is
    /// above 0 without stopping, within that radius, where that is sooner and clear of collision
    /// and the motions either side are timed alike.
    pub fn plan_sequence(
        &self,
        request: &MotionSequenceRequest,
        scene: &PlanningScene,
        current: Option<&JointState>,
    ) -> MotionSequenceResponse {
        let begun = Instant::now();
        let items: Vec<(&MotionPlanRequest, f64)> = request
            .items
            .iter()
            .map(|item| (&item.req, item.blend_radius))
            .collect();
        let outcome = self.try_sequence(&items, scene, current, begun);
        let planning_time = begun.elapsed().as_secs_f64();
        match outcome {
            Ok((sequence_start, planned_trajectories)) => MotionSequenceResponse {
                error_code: code(Code::SUCCESS, ""),
                sequence_start,
                planned_trajectories,
                planning_time,
            },
            Err(r) => MotionSequenceResponse {
                error_code: code(r.code, &r.message),
                planning_time,
                ..Default::default()
            },
        }
    }

    /// The start and the trajectories of `items`, each a request and the blend radius of the join
    /// after it.
    fn try_sequence(
        &self,
        items: &[(&MotionPlanRequest, f64)],
        scene: &PlanningScene,
        current: Option<&JointState>,
        begun: Instant,
    ) -> Result<(RobotState, Vec<RobotTrajectory>), Refusal> {
        let Some(&(first, _)) = items.first() else {
            return Ok((RobotState::default(), vec![]));
        };
        let invalid = |message: String| Refusal::new(Code::INVALID_MOTION_PLAN, message);
        // An item's refusal, naming the item where there are several.
        let at = |k: usize| {
            move |r: Refusal| match items.len() {
                1 => r,
                _ => Refusal::new(r.code, format!("item {k}: {}", r.message)),
            }
        };
        // What each request asks for is checked before the scene is built or anything planned.
        let mut groups = Vec::with_capacity(items.len());
        for (k, &(request, radius)) in items.iter().enumerate() {
            groups.push(self.asked(request).map_err(at(k))?);
            let given = &request.start_state;
            if k > 0
                && !(given.joint_state.name.is_empty()
                    && given.multi_dof_joint_state.joint_names.is_empty()
                    && given.attached_collision_objects.is_empty())
            {
                return Err(Refusal::new(
                    Code::INVALID_ROBOT_STATE,
                    format!("item {k} gives a start state: only the first may"),
                ));
            }
            if !(radius >= 0.0 && radius.is_finite()) {
                return Err(invalid(format!(
                    "item {k}'s blend radius must be finite and not negative, got {radius}"
                )));
            }
            if k + 1 == items.len() && radius != 0.0 {
                return Err(invalid(format!(
                    "the last item's blend radius must be 0, got {radius}"
                )));
            }
        }
        let settings = items
            .iter()
            .map(|(request, _)| {
                self.planner_settings(&request.group_name, &request.planner_id)
                    .map_err(|e| invalid(e.to_string()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let start = self.start(&first.start_state, scene, current)?;
        let states = [&scene.robot_state, &first.start_state];
        let frames = Frames::new(
            &self.robot,
            &start,
            &self.root,
            &self.virtual_joints,
            &states,
            scene,
        );
        let attached: Vec<&AttachedCollisionObject> = scene
            .robot_state
            .attached_collision_objects
            .iter()
            .chain(&first.start_state.attached_collision_objects)
            .collect();
        let holding = self.holding(&attached, &frames)?;
        let (robot, device, meshes) = holding
            .as_ref()
            .map_or((&self.robot, &self.device, &self.meshes), |h| {
                (&h.0, &h.1, &h.2)
            });
        let (world, ids) = scene::world(
            scene,
            &frames,
            &self.grids,
            robot,
            &self.world_links,
            &self.world_exemptions,
        )?;
        let worlds = device.upload(std::slice::from_ref(&world)).map_err(crash)?;

        let contacts = device.contacts(&worlds, &[0], &start, 0.0).map_err(crash)?;
        if let Some(nearest) = contacts[0].first() {
            let more = match contacts[0].len() - 1 {
                0 => String::new(),
                others => format!(", and {others} more"),
            };
            return Err(Refusal::new(
                Code::START_STATE_IN_COLLISION,
                format!("the start collides: {}{more}", described(nearest, &ids)),
            ));
        }
        let mut motions: Vec<Motion> = Vec::with_capacity(items.len());
        for (k, (&(request, _), group)) in items.iter().zip(groups).enumerate() {
            let setting = Setting {
                robot,
                device,
                meshes,
                world: &world,
                worlds: &worlds,
                states,
                scene,
                search: &settings[k].search,
                dt: settings[k].trajectory_dt,
            };
            let from = motions.last().map_or(&start[..], |m| m.end());
            // The first item's budget counts from the request, the others' from their own start.
            let begun = if k == 0 { begun } else { Instant::now() };
            let motion = self
                .motion(&setting, request, group, from, begun)
                .map_err(at(k))?;
            motions.push(motion);
        }

        // Each join's radius; 0 where the motions either side are timed apart or one stays put.
        let radii: Vec<f32> = motions
            .windows(2)
            .zip(items)
            .map(|(pair, &(_, radius))| {
                let (a, b) = (&pair[0], &pair[1]);
                let alike = a.group == b.group
                    && a.speed == b.speed
                    && a.cap == b.cap
                    && a.constraints == b.constraints;
                let moving = !a.path.is_empty() && !b.path.is_empty();
                if alike && moving { radius as f32 } else { 0.0 }
            })
            .collect();
        // Chains of motions whose joins may blend, one join call for them all.
        let mut chains: Vec<Range<usize>> = vec![];
        for k in 0..motions.len() {
            match chains.last_mut() {
                Some(chain) if radii[k - 1] > 0.0 => chain.end = k + 1,
                _ => chains.push(k..k + 1),
            }
        }
        let sequences: Vec<Sequence> = chains
            .iter()
            .filter(|c| c.len() > 1)
            .map(|c| Sequence {
                world: 0,
                paths: motions[c.clone()].iter().map(|m| m.path.clone()).collect(),
                blends: radii[c.start..c.end - 1].to_vec(),
                constraints: motions[c.start].constraints.clone(),
            })
            .collect();
        let joined = if sequences.is_empty() {
            None
        } else {
            // A shared blend batch must meet the finest resolution requested by any item.
            let join = batchplan::JoinOptions {
                resolution: settings
                    .iter()
                    .map(|s| s.search.join.resolution)
                    .fold(f32::INFINITY, f32::min),
            };
            Some(batchplan::join(device, &worlds, &sequences, &join).map_err(crash)?)
        };
        let mut joined = joined.iter().flat_map(|j| j.paths.iter().zip(&j.blends));

        // Each stretch of motions joined, timed and clear of the meshes; a lone motion, or motions
        // whose joined stretch grazes the meshes, as planned.
        let n = robot.dof();
        let mut pieces: Vec<(&[usize], JointTrajectory)> = vec![];
        for chain in chains {
            if chain.len() == 1 {
                let motion = &motions[chain.start];
                pieces.push((&motion.group, motion.sampled.clone()));
                continue;
            }
            let (stretches, blends) = joined
                .next()
                .expect("a joined sequence per chain of several");
            // A stretch ends at each join that keeps its stop, and at the chain's end.
            let ends = blends
                .iter()
                .enumerate()
                .filter(|(_, b)| b.is_none())
                .map(|(k, _)| chain.start + k + 1)
                .chain([chain.end]);
            let mut from = chain.start;
            for (stretch, to) in stretches.iter().zip(ends) {
                let planned = &motions[from..to];
                from = to;
                let first = &planned[0];
                if planned.len() > 1 {
                    let sampled = timed(
                        robot,
                        stretch,
                        &stretch[..n],
                        first.speed,
                        first.cap,
                        settings[chain.clone()]
                            .iter()
                            .map(|s| s.trajectory_dt)
                            .fold(f32::INFINITY, f32::min),
                    )?;
                    if clear_on_meshes(meshes, &world, &sampled)? {
                        pieces.push((&first.group, sampled));
                        continue;
                    }
                }
                pieces.extend(
                    planned
                        .iter()
                        .map(|m| (m.group.as_slice(), m.sampled.clone())),
                );
            }
        }
        // A trajectory per run of one group, as Pilz builds them.
        let mut runs: Vec<(&[usize], Vec<JointTrajectory>)> = vec![];
        for (group, piece) in pieces {
            match runs.last_mut() {
                Some((run_group, run)) if *run_group == group => run.push(piece),
                _ => runs.push((group, vec![piece])),
            }
        }
        let trajectories = runs
            .iter()
            .map(|(group, pieces)| trajectory::robot_trajectory(pieces, robot, group))
            .collect();
        let start_state = RobotState {
            joint_state: state::joint_state(robot, &start),
            ..Default::default()
        };
        Ok((start_state, trajectories))
    }

    /// The joints `request` plans, once its constraints are ones the planner takes: path
    /// constraints on links' positions and orientations (`constraints::path`), and no trajectory
    /// constraints.
    fn asked(&self, request: &MotionPlanRequest) -> Result<Vec<usize>, Refusal> {
        let path = &request.path_constraints;
        if !(path.joint_constraints.is_empty()
            && path.visibility_constraints.is_empty()
            && request.trajectory_constraints.constraints.is_empty())
        {
            return Err(Refusal::new(
                Code::INVALID_GOAL_CONSTRAINTS,
                "path constraints may bound links' positions and orientations, not joints or \
                 visibility, and trajectory constraints are not supported",
            ));
        }
        self.group(&request.group_name)
    }

    /// One motion of the joints `group` for `request` from `start` at rest, within the request's
    /// planning time counted from `begun`: a straight move for planner id `LIN`, otherwise a
    /// planned path to the nearest reachable goal, timed and clear of the meshes.
    fn motion(
        &self,
        s: &Setting,
        request: &MotionPlanRequest,
        group: Vec<usize>,
        start: &[f32],
        begun: Instant,
    ) -> Result<Motion, Refusal> {
        if !request.allowed_planning_time.is_finite()
            || Duration::try_from_secs_f64(request.allowed_planning_time.max(0.0)).is_err()
        {
            return Err(Refusal::new(
                Code::INVALID_MOTION_PLAN,
                "allowed_planning_time must be finite and representable as a duration",
            ));
        }
        let seconds = if request.allowed_planning_time > 0.0 {
            request.allowed_planning_time
        } else {
            s.search.default_planning_time
        };
        let mut last = Refusal::new(Code::TIMED_OUT, "planning time exhausted");
        for attempt in 0..request.num_planning_attempts.max(1) {
            if begun.elapsed().as_secs_f64() >= seconds {
                break;
            }
            match self.motion_attempt(s, request, group.clone(), start, begun, attempt as u64) {
                Ok(motion) => return Ok(motion),
                Err(r)
                    if matches!(
                        r.code,
                        Code::PLANNING_FAILED | Code::NO_IK_SOLUTION | Code::TIMED_OUT
                    ) =>
                {
                    last = r;
                }
                Err(r) => return Err(r),
            }
            if request.planner_id == STRAIGHT {
                break;
            }
        }
        Err(last)
    }

    fn motion_attempt(
        &self,
        s: &Setting,
        request: &MotionPlanRequest,
        group: Vec<usize>,
        start: &[f32],
        begun: Instant,
        attempt: u64,
    ) -> Result<Motion, Refusal> {
        let seconds = if request.allowed_planning_time > 0.0 {
            request.allowed_planning_time
        } else {
            s.search.default_planning_time
        };
        let budget = Duration::from_secs_f64(seconds);
        let frames = Frames::new(
            &self.robot,
            start,
            &self.root,
            &self.virtual_joints,
            &s.states,
            s.scene,
        );
        let robot = s.robot;
        let goals = goals::goals(
            &request.goal_constraints,
            robot,
            &group,
            start,
            &self.hands,
            &frames,
            &s.search.ik,
        )?;
        let constraints = constraints::path(&request.path_constraints, &self.hands, &frames)?;
        if let Some(why) = breach(robot, &constraints, start)? {
            return Err(Refusal::new(
                Code::START_STATE_VIOLATES_PATH_CONSTRAINTS,
                format!("the start breaks the path constraints: {why}"),
            ));
        }
        let speed = factor(request.max_velocity_scaling_factor)
            .min(factor(request.max_acceleration_scaling_factor).sqrt());
        let cap = self.cap(
            &request.cartesian_speed_limited_link,
            request.max_cartesian_speed,
        )?;
        let motion = |path: Vec<f32>, sampled: JointTrajectory| Motion {
            group: group.clone(),
            path,
            speed,
            cap,
            constraints: constraints.clone(),
            sampled,
        };
        if request.planner_id == STRAIGHT {
            if !constraints.is_empty() {
                return Err(Refusal::new(
                    Code::INVALID_GOAL_CONSTRAINTS,
                    "a LIN request's straight line takes no path constraints",
                ));
            }
            let (hand, pose) = match goals.as_slice() {
                [Goal::Poses { targets, .. }] if targets.len() == 1 => targets[0],
                _ => {
                    return Err(Refusal::new(
                        Code::INVALID_GOAL_CONSTRAINTS,
                        "a LIN request needs exactly one pose goal",
                    ));
                }
            };
            moves_within(robot, hand, &group)?;
            let path = ToolPath {
                world: 0,
                start: start.to_vec(),
                hand,
                waypoints: vec![pose],
            };
            let mut result = batchplan::follow(s.device, s.worlds, &[path], &s.search.cartesian)
                .map_err(crash)?;
            if let Some(why) = result.stopped[0] {
                return Err(Refusal::new(
                    Code::PLANNING_FAILED,
                    format!(
                        "the straight line stops {:.0}% of the way: {}",
                        100.0 * result.fraction[0],
                        stop(why)
                    ),
                ));
            }
            let path = result.paths.swap_remove(0);
            let sampled = timed(robot, &path, start, speed, cap, s.dt)?;
            if !clear_on_meshes(s.meshes, s.world, &sampled)? {
                return Err(Refusal::new(
                    Code::PLANNING_FAILED,
                    "the straight line touches the robot's or the scene's meshes",
                ));
            }
            return Ok(motion(path, sampled));
        }
        let search = s.search.for_attempt(attempt);
        let candidates = candidates(&goals, robot, s.device, s.worlds, start, &group, &search)?;
        let mut candidates_kept = Vec::with_capacity(candidates.len());
        for goal in candidates {
            if breach(robot, &constraints, &goal)?.is_none() {
                candidates_kept.push(goal);
            }
        }
        if candidates_kept.is_empty() {
            return Err(Refusal::new(
                Code::GOAL_VIOLATES_PATH_CONSTRAINTS,
                "every goal breaks the path constraints",
            ));
        }
        let candidates = candidates_kept;
        let hold = outside(robot, &group);
        let problems: Vec<PlanProblem> = candidates
            .iter()
            .map(|goal| PlanProblem {
                world: 0,
                start: start.to_vec(),
                goal: goal.clone(),
                start_motion: None,
                hold: hold.clone(),
                constraints: constraints.clone(),
            })
            .collect();

        // A constrained search projects every hop, and one toward a goal it cannot reach on the
        // constraints spends all the time it is given. So constrained goals are planned in rounds
        // that each stop at a time slice, doubling each round, until one has a path: the nearest
        // alone, then all of them together, as anytime planners restart with more time.
        let constrained = !constraints.is_empty();
        let mut problems = problems;
        if constrained {
            problems.sort_by(|a, b| distance(&a.goal, start).total_cmp(&distance(&b.goal, start)));
        }
        let mut touched = false;
        // A path that grazes the meshes is planned again keeping twice the clearance.
        for widen in [1.0, 2.0] {
            let (mut found, mut slice) = (false, budget / FIRST_SLICE);
            for round in 0.. {
                let remaining = budget.saturating_sub(begun.elapsed());
                if remaining.is_zero() {
                    break;
                }
                let (batch, given) = match (constrained, round) {
                    (false, _) => (&problems[..], remaining),
                    (true, 0) => (&problems[..1], slice.min(remaining)),
                    (true, _) => (&problems[..], slice.min(remaining)),
                };
                let mut options = PlanOptions {
                    time_budget: Some(given),
                    ..search.planning
                };
                options.rrt.clearance *= widen;
                options.shortcut.clearance *= widen;
                let result = batchplan::plan(s.device, s.worlds, batch, &options).map_err(crash)?;
                let mut solved: Vec<(f32, &[f32])> = result
                    .solved()
                    .map(|s| (best_length(&result, s.index), s.solution))
                    .collect();
                found |= !solved.is_empty();
                solved.sort_by(|a, b| a.0.total_cmp(&b.0));
                for (_, path) in solved {
                    let sampled = timed(robot, path, start, speed, cap, s.dt)?;
                    if clear_on_meshes(s.meshes, s.world, &sampled)? {
                        return Ok(motion(path.to_vec(), sampled));
                    }
                    touched = true;
                }
                if found || !constrained {
                    break;
                }
                slice *= 2;
            }
            if !found {
                break;
            }
        }
        Err(if begun.elapsed() >= budget {
            Refusal::new(Code::TIMED_OUT, format!("no path within {seconds} s"))
        } else if touched {
            Refusal::new(
                Code::PLANNING_FAILED,
                "every path found touches the robot's or the scene's meshes",
            )
        } else {
            Refusal::new(Code::PLANNING_FAILED, "no collision-free path found")
        })
    }

    /// The robot holding the scene's attached objects, its device and its meshes; `None` when it
    /// holds nothing.
    fn holding(
        &self,
        attached: &[&AttachedCollisionObject],
        frames: &Frames,
    ) -> Result<Option<(Robot, Device, MeshModel)>, Refusal> {
        if attached.is_empty() {
            return Ok(None);
        }
        let objects = scene::attached(attached, frames)?;
        let mut robot = self.robot.clone();
        for object in &objects {
            robot = robot
                .attach(object)
                .map_err(|e| Refusal::new(Code::INVALID_OBJECT_NAME, e.to_string()))?;
        }
        let device = self.device.with_robot(&robot).map_err(crash)?;
        let meshes = MeshModel::try_new(&robot).map_err(crash)?;
        Ok(Some((robot, device, meshes)))
    }

    /// The hand whose IK frame is `link`; with none named, the robot's only hand.
    fn hand(&self, link: &str) -> Result<usize, Refusal> {
        let link = match (link, self.hands.as_slice()) {
            ("", [only]) => only.as_str(),
            ("", _) => {
                return Err(Refusal::new(
                    Code::INVALID_LINK_NAME,
                    "name the IK link: the robot has several",
                ));
            }
            (name, _) => name,
        };
        self.hands.iter().position(|h| h == link).ok_or_else(|| {
            Refusal::new(
                Code::INVALID_LINK_NAME,
                format!("'{link}' is not an IK frame"),
            )
        })
    }

    /// A request's cap on a link's speed (m/s), as a cap on the hand whose IK frame it is; none
    /// where the request gives no positive speed.
    fn cap(&self, link: &str, speed: f64) -> Result<Option<HandSpeed>, Refusal> {
        if !(speed > 0.0 && speed.is_finite()) {
            return Ok(None);
        }
        Ok(Some(HandSpeed {
            hand: self.hand(link)?,
            linear: speed as f32,
            angular: f32::INFINITY,
        }))
    }

    /// The planned joints `group` moves: every one for an empty name.
    fn group(&self, name: &str) -> Result<Vec<usize>, Refusal> {
        let invalid = |message: String| Refusal::new(Code::INVALID_GROUP_NAME, message);
        if name.is_empty() {
            return Ok((0..self.robot.dof()).collect());
        }
        if !self.srdf.has_group(name) {
            return Err(invalid(format!("unknown group '{name}'")));
        }
        let joints = self
            .srdf
            .group_joints(name, &self.tree)
            .map_err(|e| invalid(format!("{e:#}")))?;
        let dofs: Vec<usize> = (0..self.robot.dof())
            .filter(|&j| joints.contains(&self.robot.joint_names()[j]))
            .collect();
        if dofs.is_empty() {
            return Err(invalid(format!(
                "group '{name}' moves none of the planned joints"
            )));
        }
        Ok(dofs)
    }

    fn start(
        &self,
        given: &RobotState,
        scene: &PlanningScene,
        current: Option<&JointState>,
    ) -> Result<Vec<f32>, Refusal> {
        let invalid = |message: String| Refusal::new(Code::INVALID_ROBOT_STATE, message);
        let mut states: Vec<&JointState> = current.into_iter().collect();
        states.extend([&scene.robot_state.joint_state, &given.joint_state]);
        let mut q = state::configuration(&self.robot, &states).map_err(invalid)?;
        state::clamp(&self.robot, &mut q, LIMIT_TOLERANCE).map_err(invalid)?;
        Ok(q)
    }

    /// The configuration nearest `request`'s seed that puts its link at its pose, checked against
    /// the robot and URDF world (the service carries no additional scene).
    pub fn ik(
        &self,
        request: &PositionIKRequest,
        current: Option<&JointState>,
    ) -> (RobotState, MoveItErrorCodes) {
        match self.try_ik(request, current) {
            Ok(q) => (
                RobotState {
                    joint_state: state::joint_state(&self.robot, &q),
                    ..Default::default()
                },
                code(Code::SUCCESS, ""),
            ),
            Err(r) => (RobotState::default(), code(r.code, &r.message)),
        }
    }

    /// The hand moved in straight lines through `request`'s waypoints, as MoveIt's
    /// `compute_cartesian_path` moves it, as far as it gets: `fraction` says how much of the path
    /// the trajectory covers. The service carries no scene, so the motion is checked against the
    /// robot, URDF world and the objects it holds, whatever `avoid_collisions` says.
    pub fn cartesian_path(
        &self,
        request: &GetCartesianPath_Request,
        current: Option<&JointState>,
    ) -> GetCartesianPath_Response {
        match self.try_cartesian_path(request, current) {
            Ok((start_state, solution, fraction)) => GetCartesianPath_Response {
                start_state,
                solution,
                fraction,
                error_code: code(Code::SUCCESS, ""),
            },
            Err(r) => GetCartesianPath_Response {
                error_code: code(r.code, &r.message),
                ..Default::default()
            },
        }
    }

    fn try_cartesian_path(
        &self,
        request: &GetCartesianPath_Request,
        current: Option<&JointState>,
    ) -> Result<(RobotState, RobotTrajectory, f64), Refusal> {
        let settings = self
            .planner_settings(&request.group_name, "batchplan")
            .map_err(|e| Refusal::new(Code::INVALID_MOTION_PLAN, e.to_string()))?;
        let constraints = &request.path_constraints;
        if !(constraints.joint_constraints.is_empty()
            && constraints.position_constraints.is_empty()
            && constraints.orientation_constraints.is_empty()
            && constraints.visibility_constraints.is_empty())
        {
            return Err(Refusal::new(
                Code::INVALID_GOAL_CONSTRAINTS,
                "path constraints are not supported",
            ));
        }
        let scene = PlanningScene::default();
        let group = self.group(&request.group_name)?;
        let start = self.start(&request.start_state, &scene, current)?;
        let frames = Frames::new(
            &self.robot,
            &start,
            &self.root,
            &self.virtual_joints,
            &[&request.start_state],
            &scene,
        );
        let attached: Vec<&AttachedCollisionObject> = request
            .start_state
            .attached_collision_objects
            .iter()
            .collect();
        let holding = self.holding(&attached, &frames)?;
        let (robot, device) = holding
            .as_ref()
            .map_or((&self.robot, &self.device), |h| (&h.0, &h.1));
        let hand = self.hand(&request.link_name)?;
        moves_within(robot, hand, &group)?;
        if request.waypoints.is_empty() {
            return Err(Refusal::new(
                Code::INVALID_GOAL_CONSTRAINTS,
                "the request has no waypoints",
            ));
        }
        let base = frames.pose_of(&request.header.frame_id)?;
        let waypoints = request
            .waypoints
            .iter()
            .map(|w| base.mul_pose(scene::pose(w)))
            .collect();
        let options = ToolPathOptions {
            step: if request.max_step > 0.0 {
                request.max_step as f32
            } else {
                settings.search.cartesian.step
            },
            ..settings.search.cartesian
        };
        let path = ToolPath {
            world: 0,
            start: start.clone(),
            hand,
            waypoints,
        };
        let (world, _) = scene::world(
            &scene,
            &frames,
            &self.grids,
            robot,
            &self.world_links,
            &self.world_exemptions,
        )?;
        let worlds = device.upload(&[world]).map_err(crash)?;
        let result = batchplan::follow(device, &worlds, &[path], &options).map_err(crash)?;
        let speed = factor(request.max_velocity_scaling_factor)
            .min(factor(request.max_acceleration_scaling_factor).sqrt());
        let cap = self.cap(
            &request.cartesian_speed_limited_link,
            request.max_cartesian_speed,
        )?;
        let sampled = timed(
            robot,
            &result.paths[0],
            &start,
            speed,
            cap,
            settings.trajectory_dt,
        )?;
        let start_state = RobotState {
            joint_state: state::joint_state(robot, &start),
            ..Default::default()
        };
        Ok((
            start_state,
            trajectory::robot_trajectory(std::slice::from_ref(&sampled), robot, &group),
            f64::from(result.fraction[0]),
        ))
    }

    fn try_ik(
        &self,
        request: &PositionIKRequest,
        current: Option<&JointState>,
    ) -> Result<Vec<f32>, Refusal> {
        let settings = self
            .planner_settings(&request.group_name, "batchplan")
            .map_err(|e| Refusal::new(Code::INVALID_MOTION_PLAN, e.to_string()))?;
        let seed = self.start(&request.robot_state, &PlanningScene::default(), current)?;
        let hand = self.hand(&request.ik_link_name)?;
        let scene = PlanningScene::default();
        let group = self.group(&request.group_name)?;
        let frames = Frames::new(
            &self.robot,
            &seed,
            &self.root,
            &self.virtual_joints,
            &[&request.robot_state],
            &scene,
        );
        let target = frames
            .pose_of(&request.pose_stamped.header.frame_id)?
            .mul_pose(scene::pose(&request.pose_stamped.pose));
        let (world, _) = scene::world(
            &scene,
            &frames,
            &self.grids,
            &self.robot,
            &self.world_links,
            &self.world_exemptions,
        )?;
        let worlds = self.device.upload(&[world]).map_err(crash)?;
        let targets = (0..self.robot.hands())
            .map(|h| (h == hand).then_some(target))
            .collect();
        let problem = IkProblem {
            world: 0,
            targets,
            seed: Some(seed.clone()),
            hold: outside(&self.robot, &group),
        };
        let result =
            solve_ik(&self.device, &worlds, &[problem], &settings.search.ik).map_err(crash)?;
        result
            .nearest(0, &seed)
            .map(<[f32]>::to_vec)
            .ok_or_else(|| Refusal::new(Code::NO_IK_SOLUTION, "no collision-free IK solution"))
    }
}

/// The joint-space distance between `a` and `b`.
fn distance(a: &[f32], b: &[f32]) -> f32 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f32>()
        .sqrt()
}

/// The joints outside `group`, which IK and planning hold where the start has them.
fn outside(robot: &Robot, group: &[usize]) -> Vec<usize> {
    (0..robot.dof()).filter(|j| !group.contains(j)).collect()
}

/// Goal configurations to plan to: joint goals clear of collision, and the nearest distinct IK
/// solutions of each pose goal, the joints outside the group held where they start.
fn candidates(
    goals: &[Goal],
    robot: &Robot,
    device: &Device,
    worlds: &Worlds,
    start: &[f32],
    group: &[usize],
    search: &SearchOptions,
) -> Result<Vec<Vec<f32>>, Refusal> {
    let mut out = vec![];
    let mut joint_goals = vec![];
    for goal in goals {
        if let Goal::Joints(q) = goal {
            let mut q = q.clone();
            state::clamp(robot, &mut q, LIMIT_TOLERANCE)
                .map_err(|m| Refusal::new(Code::INVALID_GOAL_CONSTRAINTS, m))?;
            joint_goals.push(q);
        }
    }
    if !joint_goals.is_empty() {
        let clear = clearances(device, worlds, &joint_goals.concat())?;
        out.extend(
            joint_goals
                .into_iter()
                .zip(clear)
                .filter(|(_, [w, s])| *w >= 0.0 && *s >= 0.0)
                .map(|(q, _)| q),
        );
    }
    // Each pose goal's targets, and the tightest tolerances any asks for.
    let (mut poses, mut position_tolerance, mut rotation_tolerance) =
        (vec![], f32::INFINITY, f32::INFINITY);
    for goal in goals {
        if let Goal::Poses {
            targets,
            position_tolerance: p,
            rotation_tolerance: r,
        } = goal
        {
            poses.push(targets.as_slice());
            position_tolerance = position_tolerance.min(*p);
            rotation_tolerance = rotation_tolerance.min(*r);
        }
    }
    if !poses.is_empty() {
        let options = IkOptions {
            position_tolerance,
            rotation_tolerance,
            ..search.ik
        };
        let hold = outside(robot, group);
        let problems: Vec<IkProblem> = poses
            .iter()
            .map(|placed| IkProblem {
                world: 0,
                targets: (0..robot.hands())
                    .map(|h| {
                        placed
                            .iter()
                            .find(|(hand, _)| *hand == h)
                            .map(|&(_, pose)| pose)
                    })
                    .collect(),
                seed: Some(start.to_vec()),
                hold: hold.clone(),
            })
            .collect();
        let ik = solve_ik(device, worlds, &problems, &options).map_err(crash)?;
        for p in 0..problems.len() {
            let mut solutions: Vec<&[f32]> = (p * ik.seeds..(p + 1) * ik.seeds)
                .filter(|&i| ik.success[i])
                .map(|i| ik.solution(i))
                .collect();
            solutions.sort_by(|a, b| distance(a, start).total_cmp(&distance(b, start)));
            let mut kept: Vec<&[f32]> = vec![];
            for q in solutions {
                if kept.len() < search.solutions_per_pose
                    && kept
                        .iter()
                        .all(|k| distance(k, q) > search.solution_distance)
                {
                    kept.push(q);
                }
            }
            out.extend(kept.into_iter().map(<[f32]>::to_vec));
        }
    }
    if out.is_empty() {
        let only_poses = goals.iter().all(|g| matches!(g, Goal::Poses { .. }));
        return Err(if only_poses {
            Refusal::new(
                Code::NO_IK_SOLUTION,
                "no collision-free IK solution for any pose goal",
            )
        } else {
            Refusal::new(Code::GOAL_IN_COLLISION, "every goal is in collision")
        });
    }
    Ok(out)
}

/// Why `q` breaks `constraints`, if it does; a refusal for constraints batchplan cannot keep, such
/// as a bounded turn about an axis whose tilt is not bounded to a quarter turn.
fn breach(
    robot: &Robot,
    constraints: &[HandConstraint],
    q: &[f32],
) -> Result<Option<String>, Refusal> {
    robot.breach(constraints, q).map_err(|e| match e {
        batchplan::Error::Input(m) => Refusal::new(Code::INVALID_GOAL_CONSTRAINTS, m),
        other => crash(other),
    })
}

/// Refuses a straight move whose hand moves joints `group` does not plan.
fn moves_within(robot: &Robot, hand: usize, group: &[usize]) -> Result<(), Refusal> {
    match robot
        .hand_joints(hand)
        .into_iter()
        .find(|j| !group.contains(j))
    {
        Some(j) => Err(Refusal::new(
            Code::INVALID_GROUP_NAME,
            format!(
                "moving the hand in a straight line moves '{}', which the group does not plan",
                robot.joint_names()[j]
            ),
        )),
        None => Ok(()),
    }
}

/// Why a straight line stops short, in words.
fn stop(why: ToolPathStop) -> &'static str {
    match why {
        ToolPathStop::Unreachable { .. } => "the next pose is out of reach",
        ToolPathStop::Collision { .. } => "going on would collide",
        ToolPathStop::Jump { .. } => "the arm would have to jump between configurations",
        _ => "it cannot go on",
    }
}

/// `path` (B-spline control points, or none for a motion that stays at `start`) timed within the
/// joints' limits slowed by `speed`, as MoveIt's scaling factors slow them, and with the hand under
/// `cap` where there is one; sampled every `dt`.
fn timed(
    robot: &Robot,
    path: &[f32],
    start: &[f32],
    speed: f32,
    cap: Option<HandSpeed>,
    dt: f32,
) -> Result<JointTrajectory, Refusal> {
    if path.is_empty() {
        let zeros = vec![0.0; start.len()];
        return Ok(JointTrajectory {
            dof: start.len(),
            dt,
            duration: 0.0,
            positions: start.to_vec(),
            velocities: zeros.clone(),
            accelerations: zeros,
        });
    }
    let Some(cap) = cap else {
        return Trajectory::try_new(robot, path, speed)
            .and_then(|t| t.sample(1.0 / dt))
            .map_err(crash);
    };
    // Timed at full speed with the cap raised by 1 / speed, then slowed by speed: the joints keep
    // within their limits scaled by speed (accelerations by its square), and the hand under the cap.
    let raised = HandSpeed {
        linear: cap.linear / speed,
        ..cap
    };
    let mut sampled = Trajectory::with_hand_speed(robot, path, &raised)
        .and_then(|t| t.sample(1.0 / (speed * dt)))
        .map_err(crash)?;
    sampled.dt = dt;
    sampled.duration /= speed;
    sampled.velocities.iter_mut().for_each(|v| *v *= speed);
    sampled
        .accelerations
        .iter_mut()
        .for_each(|a| *a *= speed * speed);
    Ok(sampled)
}

/// A contact in words, naming the scene object an obstacle comes from (`ids`, per obstacle).
fn described(contact: &Contact, ids: &[String]) -> String {
    match contact {
        Contact::World {
            link,
            obstacle,
            distance,
            ..
        } => format!(
            "{link} is {:.1} mm into '{}'",
            -distance * 1e3,
            ids[*obstacle]
        ),
        Contact::SelfCollision {
            links: [a, b],
            distance,
            ..
        } => format!("{a} and {b} overlap by {:.1} mm", -distance * 1e3),
        other => format!("{other:?}"),
    }
}

/// `[world, self]` sphere clearances of each configuration in `q` (`[n, dof]`).
fn clearances(device: &Device, worlds: &Worlds, q: &[f32]) -> Result<Vec<[f32; 2]>, Refusal> {
    let n = q.len() / device.robot().dof();
    let e = device
        .evaluate(worlds, &vec![0; n], q, &CollisionWeights::NONE)
        .map_err(crash)?;
    Ok(e.world_clearance
        .into_iter()
        .zip(e.self_clearance)
        .map(|(w, s)| [w, s])
        .collect())
}

/// Whether every waypoint clears the scene and the robot itself on the robot's meshes, as
/// MoveIt checks a returned trajectory.
fn clear_on_meshes(
    meshes: &MeshModel,
    world: &World,
    sampled: &JointTrajectory,
) -> Result<bool, Refusal> {
    let samples = sampled.positions.len() / sampled.dof;
    let clear = meshes
        .clearance(
            std::slice::from_ref(world),
            &vec![0; samples],
            &sampled.positions,
        )
        .map_err(crash)?;
    Ok(clear
        .iter()
        .all(|c| c.world > 0.0 && c.self_collision > 0.0))
}

fn best_length(result: &PlanResult, problem: usize) -> f32 {
    (problem * result.seeds..(problem + 1) * result.seeds)
        .filter(|&i| result.valid[i])
        .map(|i| result.length[i])
        .fold(f32::INFINITY, f32::min)
}

/// MoveIt's scaling factors: in (0, 1], with anything else meaning full speed.
fn factor(f: f64) -> f32 {
    if f > 0.0 && f <= 1.0 { f as f32 } else { 1.0 }
}

fn code(val: i32, message: &str) -> MoveItErrorCodes {
    MoveItErrorCodes {
        val,
        message: message.to_string(),
        source: "batchplan".to_string(),
    }
}

fn crash(e: batchplan::Error) -> Refusal {
    Refusal::new(Code::FAILURE, e.to_string())
}
