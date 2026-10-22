//! MoveIt goal constraints as joint goals or poses for IK.

use batchplan::glam::Vec3;
use batchplan::{Pose, Robot};
use ros_env::moveit_msgs::msg::{Constraints, MoveItErrorCodes as Code};
use ros_env::shape_msgs::msg::SolidPrimitive;

use crate::Refusal;
use crate::scene::{self, Frames};

pub enum Goal {
    /// Every planned joint's goal value.
    Joints(Vec<f32>),
    /// Where hand `hand`'s frame should be, and how closely.
    Pose {
        hand: usize,
        pose: Pose,
        position_tolerance: f32,
        rotation_tolerance: f32,
    },
}

/// The tightest tolerances batchplan's IK reaches by default.
const POSITION_TOLERANCE: f32 = 1e-3;
const ROTATION_TOLERANCE: f32 = 1e-2;

/// Each goal constraint as a goal. `group` lists the joints that move; a joint goal must give each
/// of them, and may name others only at their start values. `hands` are the IK frames' names.
pub fn goals(
    constraints: &[Constraints],
    robot: &Robot,
    group: &[usize],
    start: &[f32],
    hands: &[String],
    frames: &Frames,
) -> Result<Vec<Goal>, Refusal> {
    let refuse = |message: String| Refusal::new(Code::INVALID_GOAL_CONSTRAINTS, message);
    if constraints.is_empty() {
        return Err(refuse("the request has no goal constraints".into()));
    }
    constraints
        .iter()
        .map(|c| {
            if !c.visibility_constraints.is_empty() {
                return Err(refuse("visibility constraints are not supported".into()));
            }
            let joints = !c.joint_constraints.is_empty();
            let pose_parts = c.position_constraints.len() + c.orientation_constraints.len();
            match (joints, pose_parts) {
                (true, 0) => joint_goal(c, robot, group, start).map_err(refuse),
                (false, 2) => pose_goal(c, hands, frames).map_err(refuse),
                _ => Err(refuse(
                    "each goal must be joint constraints, or one position and one orientation constraint".into(),
                )),
            }
        })
        .collect()
}

fn joint_goal(
    c: &Constraints,
    robot: &Robot,
    group: &[usize],
    start: &[f32],
) -> Result<Goal, String> {
    let mut q = start.to_vec();
    let mut given = vec![false; robot.dof()];
    for jc in &c.joint_constraints {
        let Some(j) = robot.joint_names().iter().position(|n| *n == jc.joint_name) else {
            continue; // mimic joints follow their leader
        };
        let value = jc.position as f32;
        if group.contains(&j) {
            (q[j], given[j]) = (value, true);
        } else if (value - start[j]).abs()
            > jc.tolerance_above.max(jc.tolerance_below).max(1e-4) as f32
        {
            return Err(format!(
                "the goal moves '{}', which the group does not plan",
                jc.joint_name
            ));
        }
    }
    if let Some(&j) = group.iter().find(|&&j| !given[j]) {
        return Err(format!(
            "the joint goal gives no value for '{}'",
            robot.joint_names()[j]
        ));
    }
    Ok(Goal::Joints(q))
}

fn pose_goal(c: &Constraints, hands: &[String], frames: &Frames) -> Result<Goal, String> {
    let (Some(p), Some(o)) = (
        c.position_constraints.first(),
        c.orientation_constraints.first(),
    ) else {
        return Err("a pose goal needs one position and one orientation constraint".into());
    };
    if p.link_name != o.link_name {
        return Err(format!(
            "position constrains '{}' but orientation '{}'",
            p.link_name, o.link_name
        ));
    }
    let hand = hands
        .iter()
        .position(|h| *h == p.link_name)
        .ok_or_else(|| {
            format!(
                "'{}' is not an IK frame (IK frames: {})",
                p.link_name,
                hands.join(", ")
            )
        })?;
    let region = &p.constraint_region;
    let (Some(shape), Some(at)) = (region.primitives.first(), region.primitive_poses.first())
    else {
        return Err("the position constraint has no region".into());
    };
    let size = match (shape.type_, shape.dimensions.as_slice()) {
        (SolidPrimitive::SPHERE, [r]) => *r as f32,
        (SolidPrimitive::BOX, [x, y, z]) => (x.min(*y).min(*z) * 0.5) as f32,
        _ => POSITION_TOLERANCE,
    };
    let frame = |name: &str| frames.pose_of(name).map_err(|r| r.message);
    let center = frame(&p.header.frame_id)?
        .mul_pose(scene::pose(at))
        .position;
    let rotation = frame(&o.header.frame_id)?.rotation * scene::rotation(&o.orientation);
    // The constrained point sits at `target_point_offset` in the link's frame.
    let offset = &p.target_point_offset;
    let position = center - rotation * Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32);
    let angle = o
        .absolute_x_axis_tolerance
        .min(o.absolute_y_axis_tolerance)
        .min(o.absolute_z_axis_tolerance) as f32;
    Ok(Goal::Pose {
        hand,
        pose: Pose { position, rotation },
        position_tolerance: size.clamp(1e-5, POSITION_TOLERANCE),
        rotation_tolerance: if angle > 0.0 {
            angle.clamp(1e-4, ROTATION_TOLERANCE)
        } else {
            ROTATION_TOLERANCE
        },
    })
}
