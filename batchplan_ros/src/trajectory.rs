//! Sampled batchplan trajectories as ROS trajectories.

use batchplan::{JointTrajectory, Robot};
use ros_env::builtin_interfaces::msg::Duration;
use ros_env::moveit_msgs::msg::RobotTrajectory;
use ros_env::trajectory_msgs::msg::JointTrajectoryPoint;

/// The trajectory of the planned joints `joints`, a point per sample: sample `k` is at
/// `min(k * dt, duration)`, so the last is exactly where the motion ends.
pub fn robot_trajectory(
    sampled: &JointTrajectory,
    robot: &Robot,
    joints: &[usize],
) -> RobotTrajectory {
    let samples = sampled.positions.len() / sampled.dof;
    let column = |values: &[f32], k: usize| {
        joints
            .iter()
            .map(|&j| f64::from(values[k * sampled.dof + j]))
            .collect()
    };
    let mut out = RobotTrajectory::default();
    out.joint_trajectory.joint_names = joints
        .iter()
        .map(|&j| robot.joint_names()[j].clone())
        .collect();
    out.joint_trajectory.points = (0..samples)
        .map(|k| JointTrajectoryPoint {
            positions: column(&sampled.positions, k),
            velocities: column(&sampled.velocities, k),
            accelerations: column(&sampled.accelerations, k),
            effort: vec![],
            time_from_start: duration((k as f32 * sampled.dt).min(sampled.duration)),
        })
        .collect();
    out
}

pub fn duration(seconds: f32) -> Duration {
    let nanos = (f64::from(seconds) * 1e9).round() as i64;
    Duration {
        sec: (nanos / 1_000_000_000) as i32,
        nanosec: (nanos % 1_000_000_000) as u32,
    }
}
