//! The batchplan planning node: `~/move_group` (`moveit_msgs/action/MoveGroup`) plans, and executes
//! through a `FollowJointTrajectory` controller when the goal is not plan-only; `~/compute_ik`
//! (`moveit_msgs/srv/GetPositionIK`) solves IK; `~/compute_cartesian_path`
//! (`moveit_msgs/srv/GetCartesianPath`) moves the hand in straight lines through waypoints.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use anyhow::{Context as _, Result};
use batchplan_ros::{DeviceKind, Planner, PlannerOptions};
use rclrs::{
    ActionClient, ActionServer, Context, CreateBasicExecutor, GoalStatusCode, IntoPrimitiveOptions,
    Node, RclrsError, RclrsErrorFilter, RequestedGoal, Service, SpinOptions, log_error, log_info,
};
use ros_env::control_msgs::action::{FollowJointTrajectory, FollowJointTrajectory_Goal};
use ros_env::moveit_msgs::action::{MoveGroup, MoveGroup_Feedback, MoveGroup_Result};
use ros_env::moveit_msgs::msg::{MoveItErrorCodes, RobotTrajectory};
use ros_env::moveit_msgs::srv::{
    GetCartesianPath, GetCartesianPath_Request, GetPositionIK, GetPositionIK_Request,
    GetPositionIK_Response,
};
use ros_env::sensor_msgs::msg::JointState;
use ros_env::std_msgs;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("batchplan_planner")?;

    let text = |name: &str| -> Result<String> {
        Ok(node
            .declare_parameter::<Arc<str>>(name)
            .default("".into())
            .mandatory()?
            .get()
            .to_string())
    };
    let urdf = text("robot_description")?;
    let srdf = Some(text("robot_description_semantic")?).filter(|s| !s.is_empty());
    let limits_file = text("joint_limits_file")?;
    let device = text("device")?;
    let cache_dir = text("cache_dir")?;
    let controller_action = text("controller_action")?;
    let ee_links: Arc<[Arc<str>]> = node
        .declare_parameter("ee_links")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .mandatory()?
        .get();
    let threads = node
        .declare_parameter::<i64>("cpu_threads")
        .default(0)
        .mandatory()?
        .get();
    let dt = node
        .declare_parameter::<f64>("trajectory_dt")
        .default(0.02)
        .mandatory()?
        .get();

    let joint_limits = if limits_file.is_empty() {
        Default::default()
    } else {
        let yaml = std::fs::read_to_string(&limits_file)
            .with_context(|| format!("reading {limits_file}"))?;
        batchplan_ros::parse_joint_limits(&yaml)?
    };
    let options = PlannerOptions {
        package_dirs: package_dirs(),
        joint_limits,
        ee_links: ee_links.iter().map(|s| s.to_string()).collect(),
        device: match device.as_str() {
            "" | "cpu" => DeviceKind::Cpu {
                threads: usize::try_from(threads).unwrap_or(0),
            },
            "gpu" => DeviceKind::Gpu,
            other => anyhow::bail!("device must be 'cpu' or 'gpu', not '{other}'"),
        },
        cache_dir: Some(if cache_dir.is_empty() {
            default_cache_dir()
        } else {
            PathBuf::from(cache_dir)
        }),
        trajectory_dt: dt as f32,
    };

    let latest: Arc<Mutex<Option<JointState>>> = Arc::new(Mutex::new(None));
    let _states = {
        let latest = Arc::clone(&latest);
        node.create_subscription("/joint_states", move |msg: JointState| {
            *latest.lock().unwrap_or_else(|e| e.into_inner()) = Some(msg);
        })?
    };
    let controller = (!controller_action.is_empty())
        .then(|| node.create_action_client::<FollowJointTrajectory>(controller_action.as_str()))
        .transpose()?;

    // The planner is built once, from the parameter or the first description published, on a
    // thread of its own, since fitting spheres to a new robot takes seconds. Its action and service
    // appear only once it is ready, so a client waiting for them waits for the robot too.
    let serving: Arc<OnceLock<Serving>> = Arc::new(OnceLock::new());
    let pending = Arc::new(Mutex::new(Some((srdf, options))));
    let build = {
        let (serving, node) = (Arc::clone(&serving), node.clone());
        move |urdf: String| {
            let Some((srdf, options)) = pending.lock().unwrap_or_else(|e| e.into_inner()).take()
            else {
                return;
            };
            let (serving, node, latest, controller) = (
                Arc::clone(&serving),
                node.clone(),
                Arc::clone(&latest),
                controller.clone(),
            );
            std::thread::spawn(move || {
                let begun = Instant::now();
                let planner = match Planner::try_new(&urdf, srdf.as_deref(), &options) {
                    Ok(p) => Arc::new(p),
                    Err(e) => {
                        return log_error!(node.logger(), "cannot plan for this robot: {e:#}");
                    }
                };
                match serve(&node, Arc::clone(&planner), controller, latest) {
                    Ok(served) => {
                        let _ = serving.set(served);
                        log_info!(
                            node.logger(),
                            "planning {} joints on {}, ready in {:.1} s",
                            planner.robot().dof(),
                            planner.device_name(),
                            begun.elapsed().as_secs_f64()
                        );
                    }
                    Err(e) => log_error!(node.logger(), "cannot serve requests: {e}"),
                }
            });
        }
    };
    let described = AtomicBool::new(false);
    let _description = if urdf.is_empty() {
        let build = build;
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

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's interfaces, kept alive while it runs.
struct Serving {
    _move_group: ActionServer<MoveGroup>,
    _compute_ik: Service<GetPositionIK>,
    _compute_cartesian_path: Service<GetCartesianPath>,
}

fn serve(
    node: &Node,
    planner: Arc<Planner>,
    controller: Option<ActionClient<FollowJointTrajectory>>,
    latest: Arc<Mutex<Option<JointState>>>,
) -> Result<Serving, RclrsError> {
    let current = move || latest.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let move_group = {
        let (planner, current, node) = (Arc::clone(&planner), current.clone(), node.clone());
        node.clone().create_action_server::<MoveGroup, _>(
            "~/move_group",
            move |goal: RequestedGoal<MoveGroup>| {
                let (planner, current, controller, node) = (
                    Arc::clone(&planner),
                    current(),
                    controller.clone(),
                    node.clone(),
                );
                async move {
                    let request = Arc::clone(goal.goal());
                    let id = goal.goal_id().to_string();
                    let goal = goal.accept().execute();
                    goal.publish_feedback(MoveGroup_Feedback {
                        state: "PLANNING".into(),
                    });
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
                        let error_code = error(
                            MoveItErrorCodes::CRASH,
                            "planning stopped without an answer",
                        );
                        return goal.aborted_with(MoveGroup_Result {
                            error_code,
                            ..Default::default()
                        });
                    };
                    let mut result = MoveGroup_Result {
                        error_code: answer.error_code,
                        trajectory_start: answer.trajectory_start,
                        planned_trajectory: answer.trajectory,
                        executed_trajectory: RobotTrajectory::default(),
                        planning_time: answer.planning_time,
                    };
                    if result.error_code.val == MoveItErrorCodes::SUCCESS
                        && !request.planning_options.plan_only
                    {
                        goal.publish_feedback(MoveGroup_Feedback {
                            state: "MONITOR".into(),
                        });
                        match execute(controller.as_ref(), &result.planned_trajectory).await {
                            Ok(()) => {
                                result.executed_trajectory = result.planned_trajectory.clone()
                            }
                            Err(message) => {
                                result.error_code =
                                    error(MoveItErrorCodes::CONTROL_FAILED, &message)
                            }
                        }
                    }
                    let succeeded = result.error_code.val == MoveItErrorCodes::SUCCESS;
                    let what = if request.planning_options.plan_only {
                        "plan"
                    } else {
                        "plan and execute"
                    };
                    log_info!(
                        node.logger(),
                        "goal {}: {} goal(s) for '{}', {what}: {} ({:.1} ms planning)",
                        &id[..8],
                        request.request.goal_constraints.len(),
                        request.request.group_name,
                        if succeeded {
                            "done"
                        } else {
                            &result.error_code.message
                        },
                        result.planning_time * 1e3
                    );
                    if succeeded {
                        goal.succeeded_with(result)
                    } else {
                        goal.aborted_with(result)
                    }
                }
            },
        )?
    };
    let compute_ik = {
        let (planner, current) = (Arc::clone(&planner), current.clone());
        node.create_service::<GetPositionIK, _>(
            "~/compute_ik",
            move |request: GetPositionIK_Request| {
                let (solution, error_code) = planner.ik(&request.ik_request, current().as_ref());
                GetPositionIK_Response {
                    solution,
                    error_code,
                }
            },
        )?
    };
    let compute_cartesian_path = node.create_service::<GetCartesianPath, _>(
        "~/compute_cartesian_path",
        move |request: GetCartesianPath_Request| {
            planner.cartesian_path(&request, current().as_ref())
        },
    )?;
    Ok(Serving {
        _move_group: move_group,
        _compute_ik: compute_ik,
        _compute_cartesian_path: compute_cartesian_path,
    })
}

async fn execute(
    controller: Option<&ActionClient<FollowJointTrajectory>>,
    trajectory: &RobotTrajectory,
) -> Result<(), String> {
    let controller = controller.ok_or("set controller_action to execute; the node only plans")?;
    let goal = FollowJointTrajectory_Goal {
        trajectory: trajectory.joint_trajectory.clone(),
        ..Default::default()
    };
    let accepted = controller
        .request_goal(goal)
        .await
        .ok_or("the controller rejected the trajectory")?;
    let (status, result) = accepted.result.await;
    if status == GoalStatusCode::Succeeded && result.error_code == 0 {
        Ok(())
    } else {
        Err(format!(
            "the controller ended with {status:?}: {}",
            result.error_string
        ))
    }
}

fn error(val: i32, message: &str) -> MoveItErrorCodes {
    MoveItErrorCodes {
        val,
        message: message.to_string(),
        source: "batchplan".to_string(),
    }
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
