//! Joint states by name, to and from batchplan's configurations.

use batchplan::Robot;
use ros_env::sensor_msgs::msg::JointState;

/// The robot's planned joints, each from the last of `states` that names it. Joints the robot does
/// not plan (mimic joints, other robots') are ignored.
pub fn configuration(robot: &Robot, states: &[&JointState]) -> Result<Vec<f32>, String> {
    let mut q: Vec<Option<f32>> = vec![None; robot.dof()];
    for state in states {
        for (name, &value) in state.name.iter().zip(&state.position) {
            if let Some(j) = robot.joint_names().iter().position(|n| n == name) {
                q[j] = Some(value as f32);
            }
        }
    }
    let missing: Vec<&str> =
        robot.joint_names().iter().zip(&q).filter(|(_, v)| v.is_none()).map(|(n, _)| n.as_str()).collect();
    if !missing.is_empty() {
        return Err(format!("no position for {}", missing.join(", ")));
    }
    Ok(q.into_iter().flatten().collect())
}

/// Moves positions within `tolerance` of a limit onto it, since hardware reports positions a hair
/// outside; a position farther out is an error.
pub fn clamp(robot: &Robot, q: &mut [f32], tolerance: f32) -> Result<(), String> {
    for (j, v) in q.iter_mut().enumerate().filter(|&(j, _)| !robot.continuous()[j]) {
        let (lo, hi) = (robot.lower()[j], robot.upper()[j]);
        if *v < lo - tolerance || *v > hi + tolerance {
            return Err(format!("joint '{}' is at {v}, outside [{lo}, {hi}]", robot.joint_names()[j]));
        }
        *v = v.clamp(lo, hi);
    }
    Ok(())
}

pub fn joint_state(robot: &Robot, q: &[f32]) -> JointState {
    JointState {
        name: robot.joint_names().to_vec(),
        position: q.iter().map(|&v| f64::from(v)).collect(),
        ..Default::default()
    }
}
