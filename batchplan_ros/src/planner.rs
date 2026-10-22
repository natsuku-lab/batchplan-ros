//! MoveIt planning requests, each against the scene it carries.

use std::collections::HashMap;
use std::fmt;
use std::hash::Hasher;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use batchplan::{
    CollisionModel, CollisionWeights, Device, IkOptions, IkProblem, JointLimits, JointTrajectory,
    MeshModel, PlanOptions, PlanProblem, PlanResult, Robot, RobotOptions, SdfOptions, Trajectory,
    World, Worlds, solve_ik,
};
use ros_env::moveit_msgs::msg::{
    AttachedCollisionObject, MotionPlanRequest, MoveItErrorCodes, PlanningScene, PositionIKRequest,
    RobotState, RobotTrajectory,
};
use ros_env::sensor_msgs::msg::JointState;

use crate::goals::{self, Goal};
use crate::scene::{self, Fnv, Frames, Grids};
use crate::srdf::{Srdf, Tree};
use crate::{Refusal, state, trajectory};

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
    /// Spacing of returned waypoints, in seconds.
    pub trajectory_dt: f32,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            package_dirs: vec![],
            joint_limits: HashMap::new(),
            ee_links: vec![],
            device: DeviceKind::Cpu { threads: 0 },
            cache_dir: None,
            trajectory_dt: 0.02,
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

/// IK solutions planned to per pose goal: the nearest distinct ones to the start.
const SOLUTIONS_PER_POSE: usize = 4;
/// Solutions nearer than this (radians, joint space) to a kept one are the same solution.
const DISTINCT: f32 = 0.05;
/// Planning time when a request allows none.
const DEFAULT_BUDGET: f64 = 5.0;
/// Positions this close outside a limit are moved onto it.
const LIMIT_TOLERANCE: f32 = 1e-3;

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
        let tree = Tree::parse(urdf)?;
        let parsed = srdf.map(Srdf::parse).transpose()?.unwrap_or_default();
        let hands = if o.ee_links.is_empty() {
            parsed.tips()
        } else {
            o.ee_links.clone()
        };
        let mut options = RobotOptions {
            ee_links: hands.clone(),
            package_dirs: o.package_dirs.clone(),
            joint_limits: o.joint_limits.clone(),
            srdf: srdf.map(str::to_string),
            ..Default::default()
        };
        let cached = o.cache_dir.as_ref().map(|dir| {
            let mut hash = Fnv::default();
            hash.write(urdf.as_bytes());
            hash.write(srdf.unwrap_or_default().as_bytes());
            hash.write(format!("{:?}", options.spheres).as_bytes());
            dir.join(format!("{:016x}.json", hash.finish()))
        });
        if let Some(path) = cached.as_ref().filter(|p| p.exists()) {
            options.collision_model = Some(CollisionModel::load(path)?);
            // The cached model already holds the SRDF's pairs.
            options.srdf = None;
        }
        let robot = Robot::from_urdf(urdf, &options)?;
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
        })
    }

    /// The robot it plans for.
    #[must_use]
    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// What runs the batches: the GPU adapter, or the CPU's thread count.
    #[must_use]
    pub fn device_name(&self) -> String {
        self.device.name()
    }

    /// Plans `request` in `scene`, whose collision objects are the whole world. The start is
    /// `current` (the latest joint states), overridden by the scene's robot state and then the
    /// request's start state.
    pub fn plan(
        &self,
        request: &MotionPlanRequest,
        scene: &PlanningScene,
        current: Option<&JointState>,
    ) -> Answer {
        let begun = Instant::now();
        let outcome = self.try_plan(request, scene, current, begun);
        let planning_time = begun.elapsed().as_secs_f64();
        match outcome {
            Ok((trajectory_start, trajectory)) => Answer {
                error_code: code(Code::SUCCESS, ""),
                trajectory_start,
                trajectory,
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

    fn try_plan(
        &self,
        request: &MotionPlanRequest,
        scene: &PlanningScene,
        current: Option<&JointState>,
        begun: Instant,
    ) -> Result<(RobotState, RobotTrajectory), Refusal> {
        let seconds = if request.allowed_planning_time > 0.0 {
            request.allowed_planning_time
        } else {
            DEFAULT_BUDGET
        };
        let budget = Duration::from_secs_f64(seconds);
        let path = &request.path_constraints;
        let constrained = !(path.joint_constraints.is_empty()
            && path.position_constraints.is_empty()
            && path.orientation_constraints.is_empty()
            && path.visibility_constraints.is_empty()
            && request.trajectory_constraints.constraints.is_empty());
        if constrained {
            return Err(Refusal::new(
                Code::INVALID_GOAL_CONSTRAINTS,
                "path and trajectory constraints are not supported",
            ));
        }
        let group = self.group(&request.group_name)?;
        let start = self.start(&request.start_state, scene, current)?;
        let states = [&scene.robot_state, &request.start_state];
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
            .chain(&request.start_state.attached_collision_objects)
            .collect();
        let holding = self.holding(&attached, &frames)?;
        let (robot, device, meshes) = holding
            .as_ref()
            .map_or((&self.robot, &self.device, &self.meshes), |h| {
                (&h.0, &h.1, &h.2)
            });
        let world = scene::world(scene, &frames, &self.grids)?;
        let worlds = device.upload(std::slice::from_ref(&world)).map_err(crash)?;

        let clear = clearances(device, &worlds, &start)?;
        if let Some(&[w, _]) = clear.first().filter(|[w, s]| *w < 0.0 || *s < 0.0) {
            let what = if w < 0.0 { "an obstacle" } else { "each other" };
            return Err(Refusal::new(
                Code::START_STATE_IN_COLLISION,
                format!("the start's collision spheres touch {what}"),
            ));
        }

        let goals = goals::goals(
            &request.goal_constraints,
            robot,
            &group,
            &start,
            &self.hands,
            &frames,
        )?;
        let candidates = candidates(&goals, robot, device, &worlds, &start, &group)?;
        let speed = factor(request.max_velocity_scaling_factor)
            .min(factor(request.max_acceleration_scaling_factor).sqrt());
        let hold: Vec<usize> = (0..robot.dof()).filter(|j| !group.contains(j)).collect();
        let problems: Vec<PlanProblem> = candidates
            .iter()
            .map(|goal| PlanProblem {
                world: 0,
                start: start.clone(),
                goal: goal.clone(),
                start_motion: None,
                hold: hold.clone(),
            })
            .collect();

        let mut touched = false;
        // A path that grazes the meshes is planned again keeping twice the clearance.
        for widen in [1.0, 2.0] {
            let remaining = budget.saturating_sub(begun.elapsed());
            if remaining.is_zero() {
                break;
            }
            let mut options = PlanOptions {
                time_budget: Some(remaining),
                ..Default::default()
            };
            options.rrt.clearance *= widen;
            options.shortcut.clearance *= widen;
            let result = batchplan::plan(device, &worlds, &problems, &options).map_err(crash)?;
            let mut solved: Vec<(f32, &[f32])> = result
                .solved()
                .map(|s| (best_length(&result, s.index), s.solution))
                .collect();
            if solved.is_empty() {
                break;
            }
            solved.sort_by(|a, b| a.0.total_cmp(&b.0));
            for (_, path) in solved {
                let sampled = Trajectory::try_new(robot, path, speed)
                    .and_then(|t| t.sample(1.0 / self.dt))
                    .map_err(crash)?;
                if clear_on_meshes(meshes, &world, &sampled)? {
                    let trajectory = trajectory::robot_trajectory(&sampled, robot, &group);
                    let start_state = RobotState {
                        joint_state: state::joint_state(robot, &start),
                        ..Default::default()
                    };
                    return Ok((start_state, trajectory));
                }
                touched = true;
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
    /// the robot itself (the service carries no scene).
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

    fn try_ik(
        &self,
        request: &PositionIKRequest,
        current: Option<&JointState>,
    ) -> Result<Vec<f32>, Refusal> {
        let seed = self.start(&request.robot_state, &PlanningScene::default(), current)?;
        let link = match (request.ik_link_name.as_str(), self.hands.as_slice()) {
            ("", [only]) => only.clone(),
            ("", _) => {
                return Err(Refusal::new(
                    Code::INVALID_LINK_NAME,
                    "name the IK link: the robot has several",
                ));
            }
            (name, _) => name.to_string(),
        };
        let hand = self.hands.iter().position(|h| *h == link).ok_or_else(|| {
            Refusal::new(
                Code::INVALID_LINK_NAME,
                format!("'{link}' is not an IK frame"),
            )
        })?;
        let frames = Frames::new(
            &self.robot,
            &seed,
            &self.root,
            &self.virtual_joints,
            &[&request.robot_state],
            &PlanningScene::default(),
        );
        let target = frames
            .pose_of(&request.pose_stamped.header.frame_id)?
            .mul_pose(scene::pose(&request.pose_stamped.pose));
        let worlds = self.device.upload(&[World::default()]).map_err(crash)?;
        let targets = (0..self.robot.hands())
            .map(|h| (h == hand).then_some(target))
            .collect();
        let problem = IkProblem {
            world: 0,
            targets,
            seed: Some(seed.clone()),
        };
        let result =
            solve_ik(&self.device, &worlds, &[problem], &IkOptions::default()).map_err(crash)?;
        result
            .nearest(0, &seed)
            .map(<[f32]>::to_vec)
            .ok_or_else(|| Refusal::new(Code::NO_IK_SOLUTION, "no collision-free IK solution"))
    }
}

/// Goal configurations to plan to: joint goals clear of collision, and the nearest distinct IK
/// solutions of each pose goal that keep the joints outside the group where they start.
fn candidates(
    goals: &[Goal],
    robot: &Robot,
    device: &Device,
    worlds: &Worlds,
    start: &[f32],
    group: &[usize],
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
    let poses: Vec<(usize, batchplan::Pose, f32, f32)> = goals
        .iter()
        .filter_map(|g| match *g {
            Goal::Pose {
                hand,
                pose,
                position_tolerance,
                rotation_tolerance,
            } => Some((hand, pose, position_tolerance, rotation_tolerance)),
            Goal::Joints(_) => None,
        })
        .collect();
    if !poses.is_empty() {
        let options = IkOptions {
            position_tolerance: poses.iter().map(|p| p.2).fold(f32::INFINITY, f32::min),
            rotation_tolerance: poses.iter().map(|p| p.3).fold(f32::INFINITY, f32::min),
            ..Default::default()
        };
        let problems: Vec<IkProblem> = poses
            .iter()
            .map(|&(hand, pose, _, _)| IkProblem {
                world: 0,
                targets: (0..robot.hands())
                    .map(|h| (h == hand).then_some(pose))
                    .collect(),
                seed: Some(start.to_vec()),
            })
            .collect();
        let ik = solve_ik(device, worlds, &problems, &options).map_err(crash)?;
        let distance = |a: &[f32], b: &[f32]| {
            a.iter()
                .zip(b)
                .map(|(x, y)| (x - y).powi(2))
                .sum::<f32>()
                .sqrt()
        };
        let keeps_held = |q: &[f32]| {
            (0..robot.dof()).all(|j| group.contains(&j) || (q[j] - start[j]).abs() < 1e-4)
        };
        for p in 0..problems.len() {
            let mut solutions: Vec<&[f32]> = (p * ik.seeds..(p + 1) * ik.seeds)
                .filter(|&i| ik.success[i])
                .map(|i| ik.solution(i))
                .filter(|q| keeps_held(q))
                .collect();
            solutions.sort_by(|a, b| distance(a, start).total_cmp(&distance(b, start)));
            let mut kept: Vec<&[f32]> = vec![];
            for q in solutions {
                if kept.len() < SOLUTIONS_PER_POSE && kept.iter().all(|k| distance(k, q) > DISTINCT)
                {
                    kept.push(q);
                }
            }
            out.extend(kept.into_iter().map(<[f32]>::to_vec));
        }
    }
    if out.is_empty() {
        let only_poses = goals.iter().all(|g| matches!(g, Goal::Pose { .. }));
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
