//! Sampled batchplan trajectories as ROS trajectories.

use batchplan::{JointTrajectory, Robot};
use ros_env::builtin_interfaces::msg::Duration;
use ros_env::moveit_msgs::msg::RobotTrajectory;
use ros_env::trajectory_msgs::msg::JointTrajectoryPoint;

/// The trajectory of the planned joints `joints` through `pieces` one after another, each starting
/// at rest where the one before ends: a point per sample, sample `k` of a piece `min(k * dt,
/// duration)` after it starts, so its last is exactly where it ends, and the next piece's first,
/// the same point, is left out.
pub fn robot_trajectory(
    pieces: &[JointTrajectory],
    robot: &Robot,
    joints: &[usize],
) -> RobotTrajectory {
    let mut out = RobotTrajectory::default();
    out.joint_trajectory.joint_names = joints
        .iter()
        .map(|&j| robot.joint_names()[j].clone())
        .collect();
    let mut begins = 0.0;
    for (i, piece) in pieces.iter().enumerate() {
        let column = |values: &[f32], k: usize| {
            joints
                .iter()
                .map(|&j| f64::from(values[k * piece.dof + j]))
                .collect()
        };
        let samples = piece.positions.len() / piece.dof;
        let points = (usize::from(i > 0)..samples).map(|k| JointTrajectoryPoint {
            positions: column(&piece.positions, k),
            velocities: column(&piece.velocities, k),
            accelerations: column(&piece.accelerations, k),
            effort: vec![],
            time_from_start: duration(begins + (k as f32 * piece.dt).min(piece.duration)),
        });
        out.joint_trajectory.points.extend(points);
        begins += piece.duration;
    }
    out
}

pub fn duration(seconds: f32) -> Duration {
    let nanos = (f64::from(seconds) * 1e9).round() as i64;
    Duration {
        sec: (nanos / 1_000_000_000) as i32,
        nanosec: (nanos % 1_000_000_000) as u32,
    }
}
