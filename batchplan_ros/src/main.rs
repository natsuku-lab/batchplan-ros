//! The batchplan planning node: `~/move_group` (`moveit_msgs/action/MoveGroup`) plans, and executes
//! through a `FollowJointTrajectory` controller when the goal is not plan-only;
//! `~/sequence_move_group` (`moveit_msgs/action/MoveGroupSequence`) plans and executes a motion
//! sequence as Pilz's capability of that name does, passing joins without stopping; `~/compute_ik`
//! (`moveit_msgs/srv/GetPositionIK`) solves IK; `~/compute_cartesian_path`
//! (`moveit_msgs/srv/GetCartesianPath`) moves the hand in straight lines through waypoints.

use std::path::PathBuf;
use std::collections::BTreeMap;
use std::future::{Future, pending};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use anyhow::{Context as _, Result};
use batchplan_ros::{DeviceKind, Planner, PlannerOptions, SearchOptions};
use rclrs::{
    ActionClient, ActionServer, CancelResponseCode, Context, CreateBasicExecutor, GoalClient,
    GoalStatusCode, IntoPrimitiveOptions, Node, RclrsError, RclrsErrorFilter, RequestedGoal,
    RequestedGoalClient, Service, SpinOptions, log_error, log_info,
};
use ros_env::control_msgs::action::{FollowJointTrajectory, FollowJointTrajectory_Goal};
use ros_env::moveit_msgs::action::{
    MoveGroup, MoveGroup_Feedback, MoveGroup_Result, MoveGroupSequence, MoveGroupSequence_Feedback,
    MoveGroupSequence_Result,
};
use ros_env::moveit_msgs::msg::{MotionSequenceResponse, MoveItErrorCodes, RobotTrajectory};
use ros_env::moveit_msgs::srv::{
    GetCartesianPath, GetCartesianPath_Request, GetPositionIK, GetPositionIK_Request,
    GetPositionIK_Response,
    GetPlannerParams, GetPlannerParams_Request, GetPlannerParams_Response,
    SetPlannerParams, SetPlannerParams_Request, SetPlannerParams_Response,
};
use ros_env::sensor_msgs::msg::JointState;
use ros_env::std_msgs;
use ros_env::visualization_msgs::msg::MarkerArray;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("batchplan_planner")?;

    let mut text_parameters = vec![];
    let mut text = |name: &str| -> Result<String> {
        let parameter = node
            .declare_parameter::<Arc<str>>(name)
            .default("".into())
            .read_only()?;
        let value = parameter.get().to_string();
        text_parameters.push(parameter);
        Ok(value)
    };
    let urdf = text("robot_description")?;
    let srdf = Some(text("robot_description_semantic")?).filter(|s| !s.is_empty());
    let limits_file = text("joint_limits_file")?;
    let device = text("device")?;
    let cache_dir = text("cache_dir")?;
    let controller_action = text("controller_action")?;
    let ee_links_parameter = node
        .declare_parameter("ee_links")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .read_only()?;
    let ee_links: Arc<[Arc<str>]> = ee_links_parameter.get();
    let roots_parameter = node
        .declare_parameter("robot_link_roots")
        .default(Arc::from(Vec::<Arc<str>>::new()))
        .read_only()?;
    let robot_link_roots: Arc<[Arc<str>]> = roots_parameter.get();
    let voxel_parameter = node
        .declare_parameter::<f64>("world_sdf_voxel_size")
        .default(0.01)
        .read_only()?;
    let world_sdf_voxel_size = voxel_parameter.get() as f32;
    anyhow::ensure!(
        world_sdf_voxel_size.is_finite() && world_sdf_voxel_size > 0.0,
        "world_sdf_voxel_size must be finite and positive"
    );
    let threads_parameter = node
        .declare_parameter::<i64>("cpu_threads")
        .default(0)
        .read_only()?;
    let threads = usize::try_from(threads_parameter.get()).context("cpu_threads must be nonnegative")?;
    let dt_parameter = node
        .declare_parameter::<f64>("trajectory_dt")
        .default(0.02)
        .read_only()?;
    let dt = dt_parameter.get();
    let budget_parameter = node
        .declare_parameter::<i64>("sphere_budget")
        .default(64)
        .read_only()?;
    let sphere_budget =
        usize::try_from(budget_parameter.get()).context("sphere_budget must be a positive integer")?;
    anyhow::ensure!(
        sphere_budget > 0,
        "sphere_budget must be a positive integer"
    );
    let joint_limits = if limits_file.is_empty() {
        Default::default()
    } else {
        let yaml = std::fs::read_to_string(&limits_file)
            .with_context(|| format!("reading {limits_file}"))?;
        batchplan_ros::parse_joint_limits(&yaml)?
    };
    let search_parameters = SearchOptions::from_node(&node)?;
    let options = PlannerOptions {
        package_dirs: package_dirs(),
        joint_limits,
        ee_links: ee_links.iter().map(|s| s.to_string()).collect(),
        robot_link_roots: robot_link_roots.iter().map(|s| s.to_string()).collect(),
        world_sdf_voxel_size,
        device: match device.as_str() {
            "" | "cpu" => DeviceKind::Cpu {
                threads,
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
        sphere_budget,
        search: search_parameters.options,
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
    let collision_spheres =
        node.create_publisher::<MarkerArray>("~/collision_spheres".transient_local().reliable())?;

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
            let collision_spheres = collision_spheres.clone();
            std::thread::spawn(move || {
                let begun = Instant::now();
                let planner = match Planner::try_new_with_scene(
                    &urdf,
                    srdf.as_deref(),
                    &options,
                    |model, world| {
                        collision_spheres
                            .publish(batchplan_ros::collision_scene_markers(model, world))?;
                        Ok(())
                    },
                ) {
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
    _sequence_move_group: ActionServer<MoveGroupSequence>,
    _compute_ik: Service<GetPositionIK>,
    _compute_cartesian_path: Service<GetCartesianPath>,
    _get_planner_params: Service<GetPlannerParams>,
    _set_planner_params: Service<SetPlannerParams>,
}

fn serve(
    node: &Node,
    planner: Arc<Planner>,
    controller: Option<ActionClient<FollowJointTrajectory>>,
    latest: Arc<Mutex<Option<JointState>>>,
) -> Result<Serving, RclrsError> {
    let current = move || latest.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let get_planner_params = {
        let (planner, node) = (Arc::clone(&planner), node.clone());
        node.clone().create_service::<GetPlannerParams, _>(
            "~/get_planner_params",
            move |request: GetPlannerParams_Request| {
                match planner.planner_settings(&request.group, &request.planner_config) {
                    Ok(settings) => {
                        let values = settings.parameters();
                        GetPlannerParams_Response {
                            params: ros_env::moveit_msgs::msg::PlannerParams {
                                keys: values.keys().cloned().collect(),
                                values: values.values().cloned().collect(),
                                descriptions: vec![String::new(); values.len()],
                            },
                        }
                    }
                    Err(e) => {
                        log_error!(node.logger(), "cannot read planner settings: {e}");
                        GetPlannerParams_Response::default()
                    }
                }
            },
        )?
    };
    let set_planner_params = {
        let (planner, node) = (Arc::clone(&planner), node.clone());
        node.clone().create_service::<SetPlannerParams, _>(
            "~/set_planner_params",
            move |request: SetPlannerParams_Request| {
                let update = || -> Result<()> {
                    anyhow::ensure!(
                        request.params.keys.len() == request.params.values.len(),
                        "planner parameter keys and values have different lengths"
                    );
                    let values: BTreeMap<_, _> = request.params.keys.iter().cloned()
                        .zip(request.params.values.iter().cloned()).collect();
                    anyhow::ensure!(values.len() == request.params.keys.len(), "duplicate planner parameter keys");
                    planner.set_planner_parameters(
                        &request.group, &request.planner_config, &values, request.replace,
                    )
                };
                if let Err(e) = update() {
                    log_error!(node.logger(), "planner settings rejected: {e}");
                }
                SetPlannerParams_Response::default()
            },
        )?
    };
    let move_group = {
        let (planner, current, node, controller) = (
            Arc::clone(&planner),
            current.clone(),
            node.clone(),
            controller.clone(),
        );
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
                    let asked = Arc::clone(&request);
                    let planned = off_thread(move || {
                        planner.plan(
                            &asked.request,
                            &asked.planning_options.planning_scene_diff,
                            current.as_ref(),
                        )
                    });
                    let Some(answer) = cancellable(
                        planned, goal.unless_cancel_requested(pending::<()>()),
                    ).await else {
                        log_info!(node.logger(), "goal {}: planning cancelled", &id[..8]);
                        return goal.begin_cancelling().cancelled_with(MoveGroup_Result {
                            error_code: error(MoveItErrorCodes::PREEMPTED, "planning cancelled"),
                            ..Default::default()
                        });
                    };
                    let Some(answer) = answer else {
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
                        let mut execution = Execution::default();
                        match cancellable(
                            execution.run(controller.as_ref(), &result.planned_trajectory),
                            goal.unless_cancel_requested(pending::<()>()),
                        ).await {
                            Some(Ok(())) => {
                                result.executed_trajectory = result.planned_trajectory.clone()
                            }
                            Some(Err(message)) => {
                                result.error_code =
                                    error(MoveItErrorCodes::CONTROL_FAILED, &message)
                            }
                            None => {
                                let goal = goal.begin_cancelling();
                                result.error_code = execution.cancel(&node).await;
                                return if result.error_code.val == MoveItErrorCodes::PREEMPTED {
                                    goal.cancelled_with(result)
                                } else {
                                    goal.aborted_with(result)
                                };
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
    let sequence_move_group = {
        let (planner, current, node) = (Arc::clone(&planner), current.clone(), node.clone());
        node.clone().create_action_server::<MoveGroupSequence, _>(
            "~/sequence_move_group",
            move |goal: RequestedGoal<MoveGroupSequence>| {
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
                    goal.publish_feedback(MoveGroupSequence_Feedback {
                        state: "PLANNING".into(),
                    });
                    let asked = Arc::clone(&request);
                    let planned = off_thread(move || {
                        planner.plan_sequence(
                            &asked.request,
                            &asked.planning_options.planning_scene_diff,
                            current.as_ref(),
                        )
                    });
                    let Some(response) = cancellable(
                        planned, goal.unless_cancel_requested(pending::<()>()),
                    ).await else {
                        log_info!(node.logger(), "goal {}: planning cancelled", &id[..8]);
                        return goal.begin_cancelling().cancelled_with(MoveGroupSequence_Result {
                            response: MotionSequenceResponse {
                                error_code: error(MoveItErrorCodes::PREEMPTED, "planning cancelled"),
                                ..Default::default()
                            },
                        });
                    };
                    let Some(mut response) = response else {
                        let error_code = error(
                            MoveItErrorCodes::CRASH,
                            "planning stopped without an answer",
                        );
                        return goal.aborted_with(MoveGroupSequence_Result {
                            response: MotionSequenceResponse {
                                error_code,
                                ..Default::default()
                            },
                        });
                    };
                    let plan_only = request.planning_options.plan_only;
                    if response.error_code.val == MoveItErrorCodes::SUCCESS && !plan_only {
                        goal.publish_feedback(MoveGroupSequence_Feedback {
                            state: "MONITOR".into(),
                        });
                        for trajectory in &response.planned_trajectories {
                            let mut execution = Execution::default();
                            match cancellable(
                                execution.run(controller.as_ref(), trajectory),
                                goal.unless_cancel_requested(pending::<()>()),
                            ).await {
                                Some(Ok(())) => {}
                                Some(Err(message)) => {
                                    response.error_code =
                                        error(MoveItErrorCodes::CONTROL_FAILED, &message);
                                    break;
                                }
                                None => {
                                    let goal = goal.begin_cancelling();
                                    response.error_code = execution.cancel(&node).await;
                                    let result = MoveGroupSequence_Result { response };
                                    return if result.response.error_code.val == MoveItErrorCodes::PREEMPTED {
                                        goal.cancelled_with(result)
                                    } else {
                                        goal.aborted_with(result)
                                    };
                                }
                            }
                        }
                    }
                    let succeeded = response.error_code.val == MoveItErrorCodes::SUCCESS;
                    let what = if plan_only {
                        "plan"
                    } else {
                        "plan and execute"
                    };
                    log_info!(
                        node.logger(),
                        "goal {}: a sequence of {} item(s), {what}: {} ({:.1} ms planning)",
                        &id[..8],
                        request.request.items.len(),
                        if succeeded {
                            "done"
                        } else {
                            &response.error_code.message
                        },
                        response.planning_time * 1e3
                    );
                    let result = MoveGroupSequence_Result { response };
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
        _sequence_move_group: sequence_move_group,
        _compute_ik: compute_ik,
        _compute_cartesian_path: compute_cartesian_path,
        _get_planner_params: get_planner_params,
        _set_planner_params: set_planner_params,
    })
}

/// `f`'s result, worked out on a thread of its own so the executor goes on serving; `None` if the
/// thread panics.
async fn off_thread<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> Option<T> {
    let (tx, rx) = futures::channel::oneshot::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.await.ok()
}

// Poll cancellation first so already-ready work cannot dispatch a trajectory after cancellation.
async fn cancellable<T>(work: impl Future<Output = T>, cancellation: impl Future) -> Option<T> {
    match futures::future::select(Box::pin(cancellation), Box::pin(work)).await {
        futures::future::Either::Left(_) => None,
        futures::future::Either::Right((result, _)) => Some(result),
    }
}

#[derive(Default)]
struct Execution {
    pending: Option<RequestedGoalClient<FollowJointTrajectory>>,
    active: Option<GoalClient<FollowJointTrajectory>>,
}

impl Execution {
    async fn run(
        &mut self,
        controller: Option<&ActionClient<FollowJointTrajectory>>,
        trajectory: &RobotTrajectory,
    ) -> Result<(), String> {
        let controller = controller.ok_or("set controller_action to execute; the node only plans")?;
        let goal = FollowJointTrajectory_Goal {
            trajectory: trajectory.joint_trajectory.clone(),
            ..Default::default()
        };
        let requested = controller.try_request_goal(goal).map_err(|e| e.to_string())?;
        self.active = self.pending.insert(requested).await;
        self.pending = None;
        let accepted = self.active.as_mut().ok_or("the controller rejected the trajectory")?;
        let (status, result) = (&mut accepted.result).await;
        self.active = None;
        if status == GoalStatusCode::Succeeded && result.error_code == 0 {
            Ok(())
        } else {
            Err(format!("the controller ended with {status:?}: {}", result.error_string))
        }
    }

    async fn cancel(&mut self, node: &Node) -> MoveItErrorCodes {
        match self.stop().await {
            Ok(message) => {
                log_info!(node.logger(), "{message}");
                error(MoveItErrorCodes::PREEMPTED, message)
            }
            Err(message) => {
                log_error!(node.logger(), "cannot confirm controller stop: {message}");
                error(MoveItErrorCodes::CONTROL_FAILED, &message)
            }
        }
    }

    async fn stop(&mut self) -> Result<&'static str, String> {
        // Do not drop an in-flight request: it may still be accepted and start moving.
        if let Some(requested) = self.pending.take() {
            self.active = requested.await;
        }
        let Some(active) = self.active.take() else {
            return Ok("goal cancelled; no controller goal is active");
        };
        let response = active.cancellation.try_cancel().map_err(|e| e.to_string())?.await;
        // Some servers return Accept with an empty goals_canceling list when their callback rejects.
        if response.code != CancelResponseCode::GoalTerminated
            && (response.code != CancelResponseCode::Accept || response.stamp.is_none())
        {
            return Err(format!("controller did not accept cancellation: {:?}", response.code));
        }
        let (status, result) = active.result.await;
        match status {
            GoalStatusCode::Cancelled => Ok("goal cancelled; controller stopped"),
            GoalStatusCode::Succeeded if result.error_code == 0 =>
                Ok("goal cancelled after the controller had already completed"),
            _ => Err(format!("the controller ended with {status:?}: {}", result.error_string)),
        }
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
