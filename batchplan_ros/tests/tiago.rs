//! The planner on PAL Robotics' TIAGo Pro, as `scripts/fetch_tiago_pro.py` fetches it: a base on
//! planar joints and two arms, planned as one.

use std::path::{Path, PathBuf};
use std::sync::LazyLock;

use batchplan::Pose;
use batchplan::glam::{Quat, Vec3};
use batchplan_ros::{Planner, PlannerOptions};
use ros_env::geometry_msgs::msg as geometry;
use ros_env::moveit_msgs::msg::{
    Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes as Code,
    OrientationConstraint, PlanningScene, PositionConstraint, RobotTrajectory,
};
use ros_env::sensor_msgs::msg::JointState;
use ros_env::shape_msgs::msg::SolidPrimitive;

fn examples() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../batchplan_examples")
}

static TIAGO: LazyLock<Planner> = LazyLock::new(|| {
    let model = examples().join("tiago_pro");
    let read = |name: &str| {
        std::fs::read_to_string(model.join(name))
            .unwrap_or_else(|e| panic!("{name}: {e}; run `pixi run fetch-tiago-pro` first"))
    };
    let limits =
        std::fs::read_to_string(examples().join("config/tiago_pro_joint_limits.yaml")).unwrap();
    let options = PlannerOptions {
        package_dirs: vec![examples().join("..")],
        joint_limits: batchplan_ros::parse_joint_limits(&limits).unwrap(),
        cache_dir: Some(std::env::temp_dir().join("batchplan-ros-tests")),
        ..Default::default()
    };
    Planner::try_new(
        &read("tiago_pro.urdf"),
        Some(&read("tiago_pro.srdf")),
        &options,
    )
    .unwrap()
});

/// TIAGo Pro with its arms straight (the model's folded home pose is in collision with the torso's
/// spheres), the torso up 0.2 m, the head tilted down and the grippers open: every joint the
/// planner moves named.
fn start() -> JointState {
    let names = TIAGO.robot().joint_names();
    let value = |name: &str| match name {
        "torso_lift_joint" => 0.2,
        "head_2_joint" => -0.5,
        "gripper_left_finger_joint" | "gripper_right_finger_joint" => 0.035,
        _ => 0.0,
    };
    JointState {
        name: names.iter().map(|n| n.to_string()).collect(),
        position: names.iter().map(|n| value(n)).collect(),
        ..Default::default()
    }
}

const LEFT: &str = "gripper_left_tray_link";
const RIGHT: &str = "gripper_right_tray_link";

fn pose_msg(p: Pose) -> geometry::Pose {
    let [x, y, z, w] = p.rotation.to_array().map(f64::from);
    let mut msg = geometry::Pose::default();
    (msg.position.x, msg.position.y, msg.position.z) = (
        p.position.x.into(),
        p.position.y.into(),
        p.position.z.into(),
    );
    msg.orientation = geometry::Quaternion { x, y, z, w };
    msg
}

/// `link` within `tolerances` (XYZ Euler angles) of `rotation`, in `frame`.
fn oriented(
    link: &str,
    frame: &str,
    rotation: Quat,
    tolerances: [f64; 3],
) -> OrientationConstraint {
    let mut o = OrientationConstraint {
        link_name: link.into(),
        orientation: pose_msg(Pose::new(Vec3::ZERO, rotation)).orientation,
        absolute_x_axis_tolerance: tolerances[0],
        absolute_y_axis_tolerance: tolerances[1],
        absolute_z_axis_tolerance: tolerances[2],
        weight: 1.0,
        ..Default::default()
    };
    o.header.frame_id = frame.into();
    o
}

/// `link`'s origin within a box `size` around `center`, in `frame`.
fn boxed(link: &str, frame: &str, center: Vec3, size: f64) -> PositionConstraint {
    let mut p = PositionConstraint {
        link_name: link.into(),
        weight: 1.0,
        ..Default::default()
    };
    p.header.frame_id = frame.into();
    p.constraint_region.primitives.push(SolidPrimitive {
        type_: SolidPrimitive::BOX,
        dimensions: [size; 3].as_slice().try_into().unwrap(),
        ..Default::default()
    });
    p.constraint_region
        .primitive_poses
        .push(pose_msg(Pose::new(center, Quat::IDENTITY)));
    p
}

/// The planned joints at waypoint `k`, the rest where `rest` has them.
fn at(trajectory: &RobotTrajectory, k: usize, rest: &[f32]) -> Vec<f32> {
    let names = TIAGO.robot().joint_names();
    let t = &trajectory.joint_trajectory;
    let mut q = rest.to_vec();
    for (name, v) in t.joint_names.iter().zip(&t.points[k].positions) {
        q[names.iter().position(|n| n == name).unwrap()] = *v as f32;
    }
    q
}

/// The tray carried between the hands at `left` and `right`: the left within 0.05 rad of level,
/// turning freely about the vertical, and the right pinned where it is in the left's frame.
fn carrying(left: Pose, right: Pose) -> Constraints {
    let place = left.inverse().mul_pose(right);
    Constraints {
        orientation_constraints: vec![
            oriented(
                LEFT,
                "world",
                left.rotation,
                [0.05, 0.05, std::f64::consts::PI],
            ),
            oriented(RIGHT, LEFT, place.rotation, [1e-3; 3]),
        ],
        position_constraints: vec![boxed(RIGHT, LEFT, place.position, 1e-3)],
        ..Default::default()
    }
}

/// Both hands carry a tray between them, level, to a place 1.1 m away and 15 cm up, so the base
/// drives and the arms lift: one goal places both hands, solved as one IK problem, and every
/// waypoint keeps the right hand within 2 mm and 0.02 rad of its place in the left hand's frame
/// (the floors relative to another hand) and the left within 0.05 rad of level. The same request
/// without the path constraints moves the right hand past its 2 mm.
#[test]
fn two_hands_carry_a_tray_level_with_the_base() {
    let robot = TIAGO.robot();
    let current = start();
    let start: Vec<f32> = current.position.iter().map(|&v| v as f32).collect();
    let (left, right) = (robot.ee_pose(&start, 0), robot.ee_pose(&start, 1));
    let place = left.inverse().mul_pose(right);
    let moved = |hand: Pose| Pose::new(hand.position + Vec3::new(1.0, 0.4, 0.15), hand.rotation);
    let mut goal = Constraints::default();
    for (link, hand) in [(LEFT, left), (RIGHT, right)] {
        let target = moved(hand);
        goal.position_constraints
            .push(boxed(link, "world", target.position, 2e-3));
        goal.orientation_constraints
            .push(oriented(link, "world", target.rotation, [1e-2; 3]));
    }
    let free = MotionPlanRequest {
        group_name: "base_arms".into(),
        goal_constraints: vec![goal],
        allowed_planning_time: 10.0,
        ..Default::default()
    };
    let mut carried = free.clone();
    carried.path_constraints = carrying(left, right);
    // How far the right hand strays from its place, and the left from level, at any waypoint.
    let most = |request: &MotionPlanRequest| {
        let answer = TIAGO.plan(request, &PlanningScene::default(), Some(&current));
        assert_eq!(
            answer.error_code.val,
            Code::SUCCESS,
            "{}",
            answer.error_code.message
        );
        // Two of the goal's IK solutions sit on other branches of the arms, where a constrained
        // search finds no path within the budget: the nearest is planned first.
        assert!(
            answer.planning_time < 0.5 * request.allowed_planning_time,
            "planning took {:.1} s",
            answer.planning_time
        );
        let points = answer.trajectory.joint_trajectory.points.len();
        let (mut away, mut turned, mut tilted, mut drove) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for k in 0..points {
            let q = at(&answer.trajectory, k, &start);
            let (l, r) = (robot.ee_pose(&q, 0), robot.ee_pose(&q, 1));
            let now = l.inverse().mul_pose(r);
            away = away.max((now.position - place.position).abs().max_element());
            turned = turned.max(now.rotation.angle_between(place.rotation));
            tilted = tilted.max((l.rotation * Vec3::Z).angle_between(left.rotation * Vec3::Z));
            drove = drove.max((q[0] - start[0]).hypot(q[1] - start[1]));
        }
        (away, turned, tilted, drove)
    };
    let (away, turned, tilted, drove) = most(&carried);
    assert!(drove > 0.5, "the base drives only {drove:.2} m");
    assert!(
        away <= 2e-3 + 1e-4 && turned <= 0.02 + 1e-3 && tilted <= 0.05 + 1e-3,
        "the right hand strays {:.2} mm and {turned:.4} rad from its place, the left tilts {tilted:.4} rad",
        away * 1e3
    );
    let (away, _, _, _) = most(&free);
    assert!(
        away > 2e-3 + 1e-4,
        "without path constraints the hands stay within {:.2} mm",
        away * 1e3
    );
}

/// Two IK solutions for the carry's goal, both hands 1.1 m away and 15 cm up from `start()`'s,
/// every joint as `Robot::joint_names` orders them: one on other branches of the arms, which no
/// search keeping the carry's constraints reaches, and a farther one that plans in milliseconds.
const OTHER_BRANCH: [f32; 22] = [
    1.0333171,
    0.5444486,
    -0.15000147,
    0.2,
    2.5073674,
    -1.0474691,
    0.0,
    -0.6599927,
    -0.03912298,
    -0.5,
    2.594619,
    0.90466225,
    1.1344638,
    -0.5223519,
    -0.57770777,
    0.34063163,
    -0.76097065,
    0.5081917,
    1.4946154,
    -0.22770315,
    0.035,
    0.035,
];
const REACHED: [f32; 22] = [
    1.0936147,
    0.21566382,
    -0.0979822,
    0.2,
    1.0878061,
    0.44641322,
    0.0,
    -0.19177136,
    -0.19385773,
    -0.5,
    1.3619229,
    -1.1651474,
    0.7599985,
    -1.3241539,
    -3.3140962,
    3.0251489,
    0.7539968,
    -1.1942823,
    0.7661224,
    -2.443461,
    0.035,
    0.035,
];

/// Each constrained goal is planned with an even share of the time left: given the carry's two IK
/// solutions as joint goals, the nearer of which no constrained search reaches, the request plans
/// to the farther within its time.
#[test]
fn a_constrained_goal_out_of_reach_leaves_time_for_the_next() {
    let robot = TIAGO.robot();
    let current = start();
    let q: Vec<f32> = current.position.iter().map(|&v| v as f32).collect();
    let names = robot.joint_names();
    let joints = |values: &[f32; 22]| Constraints {
        joint_constraints: names
            .iter()
            .zip(values)
            .filter(|(name, _)| name.starts_with("base_joint_") || name.starts_with("arm_"))
            .map(|(name, &v)| JointConstraint {
                joint_name: name.to_string(),
                position: v.into(),
                tolerance_above: 1e-3,
                tolerance_below: 1e-3,
                weight: 1.0,
            })
            .collect(),
        ..Default::default()
    };
    let distance = |g: &[f32; 22]| {
        g.iter()
            .zip(&q)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f32>()
            .sqrt()
    };
    assert!(
        distance(&OTHER_BRANCH) < distance(&REACHED),
        "the unreachable goal is the nearer"
    );
    let request = MotionPlanRequest {
        group_name: "base_arms".into(),
        goal_constraints: vec![joints(&REACHED), joints(&OTHER_BRANCH)],
        path_constraints: carrying(robot.ee_pose(&q, 0), robot.ee_pose(&q, 1)),
        allowed_planning_time: 4.0,
        ..Default::default()
    };
    let answer = TIAGO.plan(&request, &PlanningScene::default(), Some(&current));
    assert_eq!(
        answer.error_code.val,
        Code::SUCCESS,
        "{}",
        answer.error_code.message
    );
    let last = answer.trajectory.joint_trajectory.points.len() - 1;
    let end = at(&answer.trajectory, last, &q);
    assert!(
        (end[0] - REACHED[0]).abs() < 1e-3,
        "the plan ends with the base at x {}",
        end[0]
    );
}
