//! One trajectory through a `FollowJointTrajectory` controller: sent, watched to its end, and
//! stopped when the client cancels or the controller goes quiet. Every wait on the controller is
//! bounded, and a goal the controller may still start is never left without a cancel.

use std::future::Future;
use std::pin::pin;
use std::time::Duration;

use async_std::future::timeout;
use futures::FutureExt;
use futures::future::{Either, select};
use rclrs::{
    ActionClient, CancelResponseCode, GoalClient, GoalStatusCode, Node, RequestedGoalClient,
    log_error, log_warn,
};
use ros_env::control_msgs::action::{FollowJointTrajectory, FollowJointTrajectory_Goal};
use ros_env::moveit_msgs::msg::RobotTrajectory;

/// How long the controller gets before the node stops waiting for it.
#[derive(Clone, Copy, Debug)]
pub struct Waits {
    /// For each answer: its action server to appear, a goal request, a cancel request, and the
    /// result of a goal it was told to cancel.
    pub response: Duration,
    /// For a trajectory's result, past the trajectory's own duration.
    pub margin: Duration,
}

/// How a trajectory's execution ended.
pub enum Outcome {
    /// The controller ran the trajectory to its end.
    Done,
    /// It did not, and no cancel was asked for. The message says why, and whether the controller
    /// is known to have stopped.
    Failed(String),
    /// The client cancelled: `Ok` once the controller confirmed that it stopped (or was never
    /// moving), `Err` when it did not confirm.
    Cancelled(Result<&'static str, String>),
}

/// Runs `trajectory` on `controller` until it ends or `cancelled` finishes. A cancel, or a result
/// that does not come, stops the controller before this returns, and the stop is seen through
/// whatever arrives meanwhile.
pub async fn execute(
    node: &Node,
    controller: Option<&ActionClient<FollowJointTrajectory>>,
    trajectory: &RobotTrajectory,
    waits: Waits,
    cancelled: impl Future,
) -> Outcome {
    let Some(controller) = controller else {
        return Outcome::Failed("set controller_action to execute; the node only plans".into());
    };
    let mut cancelled = pin!(cancelled.fuse());
    let response = waits.response.as_secs_f64();

    // A goal sent before the server is discovered can be lost, so the server comes first.
    let ready = async {
        controller.server_is_available().unwrap_or(false)
            || timeout(waits.response, controller.notify_on_server_ready())
                .await
                .is_ok()
    };
    match unless(&mut cancelled, ready).await {
        None => return Outcome::Cancelled(Ok("goal cancelled; no trajectory was sent")),
        Some(false) => {
            return Outcome::Failed(format!(
                "the controller's action server did not appear within {response} s"
            ));
        }
        Some(true) => {}
    }
    let goal = FollowJointTrajectory_Goal {
        trajectory: trajectory.joint_trajectory.clone(),
        ..Default::default()
    };
    let mut requested = match controller.try_request_goal(goal) {
        Ok(requested) => requested,
        Err(e) => return Outcome::Failed(e.to_string()),
    };
    let unanswered =
        || format!("the controller did not answer the goal request within {response} s");
    let accepted = match unless(&mut cancelled, timeout(waits.response, &mut requested)).await {
        Some(Ok(Some(accepted))) => accepted,
        Some(Ok(None)) => return Outcome::Failed("the controller rejected the trajectory".into()),
        Some(Err(_)) => {
            stop_late(node, requested, waits);
            return Outcome::Failed(unanswered());
        }
        // The request is out, so the controller may still accept it and move.
        None => {
            return Outcome::Cancelled(match timeout(waits.response, &mut requested).await {
                Ok(Some(accepted)) => stop(&accepted, waits).await,
                Ok(None) => Ok("goal cancelled; the controller had rejected the trajectory"),
                Err(_) => {
                    stop_late(node, requested, waits);
                    Err(unanswered())
                }
            });
        }
    };

    let duration = trajectory
        .joint_trajectory
        .points
        .last()
        .map_or(Duration::ZERO, |p| {
            Duration::new(
                p.time_from_start.sec.max(0) as u64,
                p.time_from_start.nanosec,
            )
        });
    let result = timeout(duration + waits.margin, accepted.result.clone());
    match unless(&mut cancelled, result).await {
        Some(Ok((GoalStatusCode::Succeeded, result))) if result.error_code == 0 => Outcome::Done,
        Some(Ok((status, result))) => Outcome::Failed(format!(
            "the controller ended with {status:?}: {}",
            result.error_string
        )),
        Some(Err(_)) => {
            let quiet = format!(
                "the controller gave no result within {} s of a {:.1} s trajectory's end",
                waits.margin.as_secs_f64(),
                duration.as_secs_f64()
            );
            Outcome::Failed(match stop(&accepted, waits).await {
                Ok(_) => format!("{quiet}; it was stopped"),
                Err(e) => format!("{quiet}; cannot confirm that it stopped: {e}"),
            })
        }
        None => Outcome::Cancelled(stop(&accepted, waits).await),
    }
}

/// `work`'s output, or `None` if `cancelled` finishes first. A cancel that is already in wins
/// over work that is already done.
async fn unless<T>(
    cancelled: &mut (impl Future + Unpin),
    work: impl Future<Output = T>,
) -> Option<T> {
    match select(cancelled, pin!(work)).await {
        Either::Left(_) => None,
        Either::Right((output, _)) => Some(output),
    }
}

/// Cancels `goal` and waits for the controller to report its end: `Ok` only once it has.
async fn stop(
    goal: &GoalClient<FollowJointTrajectory>,
    waits: Waits,
) -> Result<&'static str, String> {
    let response = waits.response.as_secs_f64();
    let asked = goal.cancellation.try_cancel().map_err(|e| e.to_string())?;
    let answer = timeout(waits.response, asked).await.map_err(|_| {
        format!("the controller did not answer the cancel request within {response} s")
    })?;
    // An accepted cancel carries the time it took effect; one that cancels no goal does not.
    let cancelling = answer.code == CancelResponseCode::GoalTerminated
        || (answer.code == CancelResponseCode::Accept && answer.stamp.is_some());
    if !cancelling {
        return Err(format!(
            "the controller is not cancelling the goal (it answered {:?})",
            answer.code
        ));
    }
    let (status, result) = timeout(waits.response, goal.result.clone())
        .await
        .map_err(|_| {
            format!("the controller did not report the cancelled goal's end within {response} s")
        })?;
    match status {
        GoalStatusCode::Cancelled => Ok("goal cancelled; the controller stopped"),
        GoalStatusCode::Succeeded if result.error_code == 0 => {
            Ok("goal cancelled as the controller finished the trajectory")
        }
        _ => Err(format!(
            "the controller ended with {status:?}: {}",
            result.error_string
        )),
    }
}

/// Stops a goal whose request got no answer in time, if the controller accepts it after all. The
/// watch is a task on the executor: a controller that never answers costs its memory, no thread.
fn stop_late(node: &Node, requested: RequestedGoalClient<FollowJointTrajectory>, waits: Waits) {
    let logger = node.clone();
    let watch = node.commands().run(async move {
        let Some(accepted) = requested.await else {
            return;
        };
        match stop(&accepted, waits).await {
            Ok(_) => log_warn!(
                logger.logger(),
                "the controller accepted a trajectory late; it was stopped"
            ),
            Err(e) => log_error!(
                logger.logger(),
                "the controller accepted a trajectory late; cannot confirm that it stopped: {e}"
            ),
        }
    });
    // The task runs to its end without its promise.
    drop(watch);
}
