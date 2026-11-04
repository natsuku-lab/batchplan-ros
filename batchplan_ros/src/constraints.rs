//! MoveIt path constraints as batchplan hand constraints, kept at every point of a path.
//!
//! An orientation constraint bounds a link's rotation from `desired`, `desired · Rx(a) Ry(b) Rz(c)`
//! in MoveIt's default XYZ Euler angles, each angle within its axis' tolerance. The link's z axis
//! then depends on `a` and `b` alone (the cosine of its tilt is `cos a cos b`), so a tilt cone as
//! wide as the smaller of the x and y tolerances keeps both, whatever `c` is, up to a quarter turn:
//! the bound batchplan keeps. Past a quarter turn every rotation has two such angle triples, so
//! the bound is ambiguous; a half turn or more leaves the tilt free. The z tolerance bounds the turn about the z axis left once the tilt is taken out, which
//! is `c` to within the square of the tilt. A position constraint is a box, axis-aligned in its
//! frame, around the link's origin. Constraints in an IK frame's frame move with that hand, as two
//! hands holding one object keep their places on it. Bounds tighter than IK solves to (1 mm and
//! 0.01 rad, twice that relative to another hand) count as those.

use std::f32::consts::PI;

use batchplan::glam::Vec3;
use batchplan::{HandConstraint, Pose};
use ros_env::moveit_msgs::msg::{
    Constraints, MoveItErrorCodes as Code, OrientationConstraint, PositionConstraint,
};
use ros_env::shape_msgs::msg::SolidPrimitive;

use crate::Refusal;
use crate::scene::{self, Frames};

/// Rotations closer than this to the identity (the angle, radians) count as it.
const ALIGNED: f32 = 1e-5;

/// A quarter turn, past which XYZ Euler angle tolerances no longer bound a tilt.
const QUARTER: f32 = std::f32::consts::FRAC_PI_2;

/// `path`'s position and orientation constraints as hand constraints (`Planner::asked` refuses the
/// others). `hands` are the IK frames' names.
pub fn path(
    path: &Constraints,
    hands: &[String],
    frames: &Frames,
) -> Result<Vec<HandConstraint>, Refusal> {
    let refuse = |message: String| Refusal::new(Code::INVALID_GOAL_CONSTRAINTS, message);
    let placed = |link: &str, frame: &str| placed(link, frame, hands, frames);
    let orientations = path.orientation_constraints.iter().map(|o| {
        let (hand, relative_to, frame) = placed(&o.link_name, &o.header.frame_id)?;
        orientation(o, hand, relative_to, frame).map_err(refuse)
    });
    let positions = path.position_constraints.iter().map(|p| {
        let (hand, relative_to, frame) = placed(&p.link_name, &p.header.frame_id)?;
        position(p, hand, relative_to, frame).map_err(refuse)
    });
    orientations.chain(positions).collect()
}

/// The hand a constraint bounds, the hand it moves with if its frame is another's IK frame, and
/// its frame's pose: in the world, or in that hand's frame.
fn placed(
    link: &str,
    frame: &str,
    hands: &[String],
    frames: &Frames,
) -> Result<(usize, Option<usize>, Pose), Refusal> {
    let hand_of = |name: &str| hands.iter().position(|h| h == name);
    let hand = hand_of(link).ok_or_else(|| {
        Refusal::new(
            Code::INVALID_LINK_NAME,
            format!(
                "path constraints bound IK frames; '{link}' is not one (IK frames: {})",
                hands.join(", ")
            ),
        )
    })?;
    if let Some(other) = hand_of(frame) {
        if other == hand {
            return Err(Refusal::new(
                Code::INVALID_GOAL_CONSTRAINTS,
                format!("a path constraint on '{link}' is in its own frame"),
            ));
        }
        return Ok((hand, Some(other), Pose::IDENTITY));
    }
    // batchplan keeps constraints in the world or in a hand's frame.
    if frames.moves(frame) {
        return Err(Refusal::new(
            Code::FRAME_TRANSFORM_FAILURE,
            format!(
                "path constraints are in the planning frame, a fixed frame or an IK frame, not \
                 '{frame}', which the robot's joints move"
            ),
        ));
    }
    Ok((hand, None, frames.pose_of(frame)?))
}

fn orientation(
    o: &OrientationConstraint,
    hand: usize,
    relative_to: Option<usize>,
    frame: Pose,
) -> Result<HandConstraint, String> {
    if o.parameterization != OrientationConstraint::XYZ_EULER_ANGLES {
        return Err(format!(
            "the orientation constraint on '{}' needs XYZ Euler angle tolerances: rotation \
             vectors do not bound a tilt",
            o.link_name
        ));
    }
    let tolerances = [
        o.absolute_x_axis_tolerance,
        o.absolute_y_axis_tolerance,
        o.absolute_z_axis_tolerance,
    ];
    if !tolerances.iter().all(|t| *t >= 0.0) {
        return Err(format!(
            "the orientation constraint on '{}' has a negative or NaN tolerance: {tolerances:?}",
            o.link_name
        ));
    }
    let [x, y, z] = tolerances.map(|t| (t as f32).min(PI));
    if x.min(y) > QUARTER && x.min(y) < PI {
        return Err(format!(
            "the orientation constraint on '{}' allows x and y within {:.2} rad: past a quarter \
             turn, XYZ Euler angles do not bound a tilt; use π or more to leave it free",
            o.link_name,
            x.min(y)
        ));
    }
    let rotation = frame.rotation * scene::rotation(&o.orientation);
    Ok(HandConstraint {
        relative_to,
        tilt: x.min(y),
        twist: z,
        ..HandConstraint::new(hand, Pose::new(Vec3::ZERO, rotation))
    })
}

fn position(
    p: &PositionConstraint,
    hand: usize,
    relative_to: Option<usize>,
    frame: Pose,
) -> Result<HandConstraint, String> {
    let link = &p.link_name;
    let offset = &p.target_point_offset;
    if [offset.x, offset.y, offset.z] != [0.0; 3] {
        return Err(format!(
            "the position constraint on '{link}' bounds a point off the link's origin, which \
             batchplan cannot keep"
        ));
    }
    let region = &p.constraint_region;
    let (Some(shape), Some(at), true, true) = (
        region.primitives.first(),
        region.primitive_poses.first(),
        region.primitives.len() == 1 && region.primitive_poses.len() == 1,
        region.meshes.is_empty(),
    ) else {
        return Err(format!(
            "the position constraint on '{link}' needs one box as its region"
        ));
    };
    let [x, y, z] = match (shape.type_, shape.dimensions.as_slice()) {
        (SolidPrimitive::BOX, &[x, y, z]) if x >= 0.0 && y >= 0.0 && z >= 0.0 => {
            [x, y, z].map(|d| 0.5 * d as f32)
        }
        _ => {
            return Err(format!(
                "the position constraint on '{link}' needs one box as its region"
            ));
        }
    };
    let center = frame.mul_pose(scene::pose(at));
    // The angle from the identity, twice the sine of half of it, which keeps its precision near 0.
    if 2.0 * center.rotation.xyz().length() > ALIGNED {
        return Err(format!(
            "the box bounding '{link}' must be aligned with the axes of the {}",
            if relative_to.is_some() {
                "IK frame it is in"
            } else {
                "robot's root link"
            }
        ));
    }
    Ok(HandConstraint {
        relative_to,
        position: Vec3::new(x, y, z),
        ..HandConstraint::new(hand, Pose::new(center.position, center.rotation))
    })
}
