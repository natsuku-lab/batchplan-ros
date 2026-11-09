//! MoveIt goal constraints as joint goals or poses for IK.

use batchplan::glam::Vec3;
use batchplan::{IkOptions, Pose, Robot};
use ros_env::moveit_msgs::msg::{Constraints, MoveItErrorCodes as Code};
use ros_env::shape_msgs::msg::SolidPrimitive;

use crate::Refusal;
use crate::scene::{self, Frames};

pub enum Goal {
    /// Every planned joint's goal value.
    Joints(Vec<f32>),
    /// Where some hands' frames should be, each at most once, and how closely: one IK problem.
    Poses {
        targets: Vec<(usize, Pose)>,
        position_tolerance: f32,
        rotation_tolerance: f32,
    },
}

/// Each goal constraint as a goal. `group` lists the joints that move; a joint goal must give each
/// of them, and may name others only at their start values. `hands` are the IK frames' names.
pub fn goals(
    constraints: &[Constraints],
    robot: &Robot,
    group: &[usize],
    start: &[f32],
    hands: &[String],
    frames: &Frames,
    ik: &IkOptions,
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
                (false, 1..) => pose_goal(c, hands, frames, ik).map_err(refuse),
                _ => Err(refuse(
                    "each goal must be joint constraints, or a position and an orientation constraint for each link it places".into(),
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

/// A goal placing each link a position and an orientation constraint name, at most one of each
/// per link, as one IK problem.
fn pose_goal(
    c: &Constraints,
    hands: &[String],
    frames: &Frames,
    ik: &IkOptions,
) -> Result<Goal, String> {
    if c.position_constraints.len() != c.orientation_constraints.len() {
        return Err(
            "a pose goal needs a position and an orientation constraint for each link".into(),
        );
    }
    let (mut targets, mut position_tolerance, mut rotation_tolerance) =
        (vec![], ik.position_tolerance, ik.rotation_tolerance);
    for p in &c.position_constraints {
        let mut matching = c
            .orientation_constraints
            .iter()
            .filter(|o| o.link_name == p.link_name);
        let (Some(o), None) = (matching.next(), matching.next()) else {
            return Err(format!(
                "'{}' needs exactly one orientation constraint beside its position constraint",
                p.link_name
            ));
        };
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
        if targets.iter().any(|&(h, _)| h == hand) {
            return Err(format!("the goal places '{}' twice", p.link_name));
        }
        let region = &p.constraint_region;
        let (Some(shape), Some(at)) = (region.primitives.first(), region.primitive_poses.first())
        else {
            return Err(format!(
                "the position constraint on '{}' has no region",
                p.link_name
            ));
        };
        let size = match (shape.type_, shape.dimensions.as_slice()) {
            (SolidPrimitive::SPHERE, [r]) => *r as f32,
            (SolidPrimitive::BOX, [x, y, z]) => (x.min(*y).min(*z) * 0.5) as f32,
            _ => ik.position_tolerance,
        };
        let frame = |name: &str| frames.pose_of(name).map_err(|r| r.message);
        let center = frame(&p.header.frame_id)?
            .mul_pose(scene::pose(at))
            .position;
        let rotation = frame(&o.header.frame_id)?.rotation * scene::rotation(&o.orientation);
        // The constrained point sits at `target_point_offset` in the link's frame.
        let offset = &p.target_point_offset;
        let position =
            center - rotation * Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32);
        let angle = o
            .absolute_x_axis_tolerance
            .min(o.absolute_y_axis_tolerance)
            .min(o.absolute_z_axis_tolerance) as f32;
        position_tolerance = position_tolerance.min(size.max(1e-5).min(ik.position_tolerance));
        if angle > 0.0 {
            rotation_tolerance = rotation_tolerance.min(angle.max(1e-4).min(ik.rotation_tolerance));
        }
        targets.push((hand, Pose { position, rotation }));
    }
    Ok(Goal::Poses {
        targets,
        position_tolerance,
        rotation_tolerance,
    })
}
