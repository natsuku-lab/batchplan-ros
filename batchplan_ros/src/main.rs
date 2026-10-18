//! The batchplan planning node: `~/move_group` (`moveit_msgs/action/MoveGroup`) plans, and executes
//! through a `FollowJointTrajectory` controller when the goal is not plan-only; `~/compute_ik`
//! (`moveit_msgs/srv/GetPositionIK`) solves IK.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use anyhow::{Context as _, Result};
use batchplan_ros::{DeviceKind, Planner, PlannerOptions, parse_joint_limits};
use rclrs::*;
use ros_env::control_msgs::action::{FollowJointTrajectory, FollowJointTrajectory_Goal};
use ros_env::moveit_msgs::action::{MoveGroup, MoveGroup_Feedback, MoveGroup_Result};
use ros_env::moveit_msgs::msg::{MoveItErrorCodes, RobotTrajectory};
use ros_env::moveit_msgs::srv::{GetPositionIK, GetPositionIK_Request, GetPositionIK_Response};
use ros_env::sensor_msgs::msg::JointState;
use ros_env::std_msgs;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("batchplan_planner")?;

    let text = |name: &str| -> Result<String> {
        Ok(node.declare_parameter::<Arc<str>>(name).default("".into()).mandatory()?.get().to_string())
    };
    let urdf = text("robot_description")?;
    let srdf = Some(text("robot_description_semantic")?).filter(|s| !s.is_empty());
    let limits_file = text("joint_limits_file")?;
    let device = text("device")?;
    let cache_dir = text("cache_dir")?;
    let controller_action = text("controller_action")?;
    let ee_links: Arc<[Arc<str>]> =
        node.declare_parameter("ee_links").default(Arc::from(Vec::<Arc<str>>::new())).mandatory()?.get();
    let threads = node.declare_parameter::<i64>("cpu_threads").default(0).mandatory()?.get();
    let dt = node.declare_parameter::<f64>("trajectory_dt").default(0.02).mandatory()?.get();

    let joint_limits = if limits_file.is_empty() {
        Default::default()
    } else {
        let yaml = std::fs::read_to_string(&limits_file).with_context(|| format!("reading {limits_file}"))?;
        parse_joint_limits(&yaml)?
    };
    let options = PlannerOptions {
        package_dirs: package_dirs(),
        joint_limits,
        ee_links: ee_links.iter().map(|s| s.to_string()).collect(),
        device: match device.as_str() {
            "" | "cpu" => DeviceKind::Cpu { threads: usize::try_from(threads).unwrap_or(0) },
            "gpu" => DeviceKind::Gpu,
            other => anyhow::bail!("device must be 'cpu' or 'gpu', not '{other}'"),
        },
        cache_dir: Some(if cache_dir.is_empty() { default_cache_dir() } else { PathBuf::from(cache_dir) }),
        trajectory_dt: dt as f32,
    };

    // The planner is built once, from the parameter or the first description published, on a
    // thread of its own: fitting spheres to a new robot takes seconds.
    let planner: Arc<OnceLock<Arc<Planner>>> = Arc::new(OnceLock::new());
    let pending = Arc::new(Mutex::new(Some((srdf, options))));
    let build = {
        let (planner, node) = (Arc::clone(&planner), node.clone());
        move |urdf: String| {
            let Some((srdf, options)) = pending.lock().unwrap_or_else(|e| e.into_inner()).take() else { return };
            let (planner, node) = (Arc::clone(&planner), node.clone());
            std::thread::spawn(move || {
                let begun = Instant::now();
                match Planner::new(&urdf, srdf.as_deref(), &options) {
                    Ok(p) => {
                        log_info!(
                            node.logger(),
                            "planning {} joints on {}, ready in {:.1} s",
                            p.robot().dof(),
                            p.device_name(),
                            begun.elapsed().as_secs_f64()
                        );
                        let _ = planner.set(Arc::new(p));
                    }
                    Err(e) => log_error!(node.logger(), "cannot plan for this robot: {e:#}"),
                }
            });
        }
    };
    let described = AtomicBool::new(false);
    let _description = if urdf.is_empty() {
        let build = build.clone();
        Some(node.create_subscription(
            "/robot_description".transient_local().reliable(),
            move |msg: std_msgs::msg::String| {
                if !described.swap(true, Ordering::Relaxed) {
                    build(msg.data);
                }
            },
        )?)
    } else {
        build(urdf);
        None
    };

    let latest: Arc<Mutex<Option<JointState>>> = Arc::new(Mutex::new(None));
    let _states = {
        let latest = Arc::clone(&latest);
        node.create_subscription("/joint_states", move |msg: JointState| {
            *latest.lock().unwrap_or_else(|e| e.into_inner()) = Some(msg);
        })?
    };
    let current = {
        let latest = Arc::clone(&latest);
        move || latest.lock().unwrap_or_else(|e| e.into_inner()).clone()
    };

    let controller = (!controller_action.is_empty())
        .then(|| node.create_action_client::<FollowJointTrajectory>(controller_action.as_str()));
    let controller = controller.transpose()?;

    let _move_group = {
        let (planner, current) = (Arc::clone(&planner), current.clone());
        node.create_action_server::<MoveGroup, _>("~/move_group", move |goal: RequestedGoal<MoveGroup>| {
            let (planner, current, controller) = (planner.get().cloned(), current(), controller.clone());
            async move {
                let request = Arc::clone(goal.goal());
                let goal = goal.accept().execute();
                let Some(planner) = planner else {
                    let error_code = error(MoveItErrorCodes::FAILURE, "the robot description has not arrived yet");
                    return goal.aborted_with(MoveGroup_Result { error_code, ..Default::default() });
                };
                goal.publish_feedback(MoveGroup_Feedback { state: "PLANNING".into() });
                let (tx, rx) = futures::channel::oneshot::channel();
                let asked = Arc::clone(&request);
                std::thread::spawn(move || {
                    let _ = tx.send(planner.plan(
                        &asked.request,
                        &asked.planning_options.planning_scene_diff,
                        current.as_ref(),
                    ));
                });
                let Ok(answer) = rx.await else {
                    let error_code = error(MoveItErrorCodes::CRASH, "planning stopped without an answer");
                    return goal.aborted_with(MoveGroup_Result { error_code, ..Default::default() });
                };
                let mut result = MoveGroup_Result {
                    error_code: answer.error_code,
                    trajectory_start: answer.trajectory_start,
                    planned_trajectory: answer.trajectory,
                    executed_trajectory: RobotTrajectory::default(),
                    planning_time: answer.planning_time,
                };
                if result.error_code.val == MoveItErrorCodes::SUCCESS && !request.planning_options.plan_only {
                    goal.publish_feedback(MoveGroup_Feedback { state: "MONITOR".into() });
                    match execute(controller.as_ref(), &result.planned_trajectory).await {
                        Ok(()) => result.executed_trajectory = result.planned_trajectory.clone(),
                        Err(message) => result.error_code = error(MoveItErrorCodes::CONTROL_FAILED, &message),
                    }
                }
                if result.error_code.val == MoveItErrorCodes::SUCCESS {
                    goal.succeeded_with(result)
                } else {
                    goal.aborted_with(result)
                }
            }
        })?
    };

    let _compute_ik =
        node.create_service::<GetPositionIK, _>("~/compute_ik", move |request: GetPositionIK_Request| {
            let Some(planner) = planner.get() else {
                let error_code = error(MoveItErrorCodes::FAILURE, "the robot description has not arrived yet");
                return GetPositionIK_Response { error_code, ..Default::default() };
            };
            let (solution, error_code) = planner.ik(&request.ik_request, current().as_ref());
            GetPositionIK_Response { solution, error_code }
        })?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

async fn execute(
    controller: Option<&ActionClient<FollowJointTrajectory>>,
    trajectory: &RobotTrajectory,
) -> Result<(), String> {
    let controller = controller.ok_or("set controller_action to execute; the node only plans")?;
    let goal = FollowJointTrajectory_Goal { trajectory: trajectory.joint_trajectory.clone(), ..Default::default() };
    let accepted = controller.request_goal(goal).await.ok_or("the controller rejected the trajectory")?;
    let (status, result) = accepted.result.await;
    if status == GoalStatusCode::Succeeded && result.error_code == 0 {
        Ok(())
    } else {
        Err(format!("the controller ended with {status:?}: {}", result.error_string))
    }
}

fn error(val: i32, message: &str) -> MoveItErrorCodes {
    MoveItErrorCodes { val, message: message.to_string(), source: "batchplan".to_string() }
}

/// Every installed package's parent directory: `share/` under each prefix on `AMENT_PREFIX_PATH`.
fn package_dirs() -> Vec<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .unwrap_or_default()
        .split(':')
        .filter(|p| !p.is_empty())
        .map(|p| PathBuf::from(p).join("share"))
        .collect()
}

fn default_cache_dir() -> PathBuf {
    let base = std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
        .unwrap_or_else(std::env::temp_dir);
    base.join("batchplan-ros")
}
