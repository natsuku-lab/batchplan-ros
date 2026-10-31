//! The planner through its interface: MoveIt requests and scenes in, trajectories and error codes
//! out, on MoveIt's own Panda description. Run inside the pixi environment, which provides it.

use std::path::PathBuf;
use std::sync::LazyLock;

use batchplan::glam::{Quat, Vec3};
use batchplan::{MeshModel, Obstacle, Pose, World};
use batchplan_ros::{Answer, Planner, PlannerOptions};
use ros_env::geometry_msgs::msg as geometry;
use ros_env::moveit_msgs::msg::{
    AllowedCollisionEntry, AllowedCollisionMatrix, AttachedCollisionObject, CollisionObject,
    Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes as Code,
    OrientationConstraint, PlanningScene, PositionConstraint, PositionIKRequest, RobotTrajectory,
};
use ros_env::moveit_msgs::srv::GetCartesianPath_Request;
use ros_env::sensor_msgs::msg::JointState;
use ros_env::shape_msgs::msg::{Mesh, MeshTriangle, SolidPrimitive};

const ARM: [&str; 7] = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
];
const READY: [f64; 7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];

const SRDF: &str = r#"<robot name="panda">
  <group name="panda_arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
  <group name="hand"><link name="panda_hand"/><link name="panda_leftfinger"/><link name="panda_rightfinger"/></group>
  <group name="panda_arm_hand"><group name="panda_arm"/><group name="hand"/></group>
  <end_effector name="hand" parent_link="panda_link8" group="hand" parent_group="panda_arm"/>
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="panda_link0"/>
  <disable_collisions link1="panda_hand" link2="panda_leftfinger" reason="Adjacent"/>
</robot>"#;

fn share() -> PathBuf {
    let prefix = std::env::var("CONDA_PREFIX").expect("run inside the pixi environment");
    PathBuf::from(prefix).join("share")
}

fn panda(srdf: &str) -> Planner {
    let urdf =
        std::fs::read_to_string(share().join("moveit_resources_panda_description/urdf/panda.urdf"))
            .unwrap();
    let cache_dir = Some(std::env::temp_dir().join("batchplan-ros-tests"));
    let options = PlannerOptions {
        package_dirs: vec![share()],
        cache_dir,
        ..Default::default()
    };
    Planner::try_new(&urdf, Some(srdf), &options).unwrap()
}

static PLANNER: LazyLock<Planner> = LazyLock::new(|| panda(SRDF));
/// As MoveIt's Panda configuration has it: the root on a floating virtual joint from `world`.
static FLOATING: LazyLock<Planner> =
    LazyLock::new(|| panda(&SRDF.replace(r#"type="fixed""#, r#"type="floating""#)));

fn state(names: &[&str], positions: &[f64]) -> JointState {
    JointState {
        name: names.iter().map(|n| n.to_string()).collect(),
        position: positions.to_vec(),
        ..Default::default()
    }
}

/// The arm at `arm`, the fingers half open.
fn current(arm: &[f64]) -> JointState {
    let mut names = ARM.to_vec();
    names.extend(["panda_finger_joint1", "panda_finger_joint2"]);
    state(&names, &[arm, &[0.02, 0.02]].concat())
}

fn joint_goal(names: &[&str], values: &[f64]) -> Constraints {
    let joint_constraints = names
        .iter()
        .zip(values)
        .map(|(n, &v)| JointConstraint {
            joint_name: n.to_string(),
            position: v,
            tolerance_above: 1e-3,
            tolerance_below: 1e-3,
            weight: 1.0,
        })
        .collect();
    Constraints {
        joint_constraints,
        ..Default::default()
    }
}

fn pose_msg(p: Pose) -> geometry::Pose {
    let [x, y, z, w] = p.rotation.to_array().map(f64::from);
    geometry::Pose {
        position: geometry::Point {
            x: p.position.x.into(),
            y: p.position.y.into(),
            z: p.position.z.into(),
        },
        orientation: geometry::Quaternion { x, y, z, w },
    }
}

fn pose_goal(link: &str, frame: &str, target: Pose) -> Constraints {
    let mut position = PositionConstraint {
        link_name: link.into(),
        weight: 1.0,
        ..Default::default()
    };
    position.header.frame_id = frame.into();
    let sphere = SolidPrimitive {
        type_: SolidPrimitive::SPHERE,
        dimensions: [1e-3].as_slice().try_into().unwrap(),
        ..Default::default()
    };
    position.constraint_region.primitives.push(sphere);
    position
        .constraint_region
        .primitive_poses
        .push(pose_msg(Pose {
            position: target.position,
            rotation: Quat::IDENTITY,
        }));
    let mut orientation = OrientationConstraint {
        link_name: link.into(),
        orientation: pose_msg(target).orientation,
        absolute_x_axis_tolerance: 1e-2,
        absolute_y_axis_tolerance: 1e-2,
        absolute_z_axis_tolerance: 1e-2,
        weight: 1.0,
        ..Default::default()
    };
    orientation.header.frame_id = frame.into();
    Constraints {
        position_constraints: vec![position],
        orientation_constraints: vec![orientation],
        ..Default::default()
    }
}

fn request(group: &str, goals: Vec<Constraints>) -> MotionPlanRequest {
    MotionPlanRequest {
        group_name: group.into(),
        goal_constraints: goals,
        ..Default::default()
    }
}

fn cuboid(id: &str, frame: &str, center: [f64; 3], size: [f64; 3]) -> CollisionObject {
    let mut object = CollisionObject {
        id: id.into(),
        operation: CollisionObject::ADD,
        ..Default::default()
    };
    object.header.frame_id = frame.into();
    object.primitives.push(SolidPrimitive {
        type_: SolidPrimitive::BOX,
        dimensions: size.as_slice().try_into().unwrap(),
        ..Default::default()
    });
    let mut at = geometry::Pose::default();
    (at.position.x, at.position.y, at.position.z) = (center[0], center[1], center[2]);
    object.primitive_poses.push(at);
    object
}

fn scene(objects: Vec<CollisionObject>) -> PlanningScene {
    let mut scene = PlanningScene {
        is_diff: true,
        ..Default::default()
    };
    scene.world.collision_objects = objects;
    scene
}

fn plan(request: &MotionPlanRequest, scene: &PlanningScene, start: &[f64]) -> Answer {
    PLANNER.plan(request, scene, Some(&current(start)))
}

fn succeeded(answer: &Answer) {
    assert_eq!(
        answer.error_code.val,
        Code::SUCCESS,
        "{}",
        answer.error_code.message
    );
}

/// The planned joints at waypoint `k`, with the joints the trajectory leaves out at `rest`.
fn waypoint(answer: &Answer, k: usize, rest: &[f32]) -> Vec<f32> {
    joints_at(&answer.trajectory, k, rest)
}

fn joints_at(trajectory: &RobotTrajectory, k: usize, rest: &[f32]) -> Vec<f32> {
    let names = PLANNER.robot().joint_names();
    let t = &trajectory.joint_trajectory;
    let mut q = rest.to_vec();
    for (name, v) in t.joint_names.iter().zip(&t.points[k].positions) {
        q[names.iter().position(|n| n == name).unwrap()] = *v as f32;
    }
    q
}

/// Where the hand's IK frame (panda_link8) is at each waypoint, and when, from a start at `arm`.
fn hand_path(trajectory: &RobotTrajectory, arm: &[f64]) -> Vec<(f64, Vec3)> {
    let rest = full(arm);
    let points = &trajectory.joint_trajectory.points;
    (0..points.len())
        .map(|k| {
            let q = joints_at(trajectory, k, &rest);
            let at = PLANNER.robot().ee_pose(&q, 0).position;
            (seconds(&points[k].time_from_start), at)
        })
        .collect()
}

/// The hand's fastest speed between waypoints, m/s.
fn fastest(path: &[(f64, Vec3)]) -> f64 {
    path.windows(2)
        .map(|w| f64::from((w[1].1 - w[0].1).length()) / (w[1].0 - w[0].0))
        .fold(0.0, f64::max)
}

/// How far `p` is from the segment from `a` to `b`.
fn off_segment(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let along = (p - a).dot(b - a) / (b - a).length_squared();
    (p - (a + (b - a) * along.clamp(0.0, 1.0))).length()
}

/// A matrix in which `object` and each of `links` may touch as `allowed` says, and nothing else.
fn allowing(object: &str, links: &[&str], allowed: bool) -> AllowedCollisionMatrix {
    let names: Vec<String> = [object]
        .iter()
        .chain(links)
        .map(|n| n.to_string())
        .collect();
    let entry_values = (0..names.len())
        .map(|i| AllowedCollisionEntry {
            enabled: (0..names.len())
                .map(|j| (i == 0) != (j == 0) && allowed)
                .collect(),
        })
        .collect();
    AllowedCollisionMatrix {
        entry_names: names,
        entry_values,
        ..Default::default()
    }
}

/// A straight move of panda_link8 to `goal`, as Pilz's LIN asks for one.
fn lin(goal: Pose) -> MotionPlanRequest {
    MotionPlanRequest {
        planner_id: "LIN".into(),
        ..request("panda_arm", vec![pose_goal("panda_link8", "world", goal)])
    }
}

fn seconds(d: &ros_env::builtin_interfaces::msg::Duration) -> f64 {
    f64::from(d.sec) + f64::from(d.nanosec) * 1e-9
}

/// Joints in the robot's order: the arm, then the fingers.
fn full(arm: &[f64]) -> Vec<f32> {
    arm.iter().map(|&v| v as f32).chain([0.02]).collect()
}

#[test]
fn joint_goals_return_the_groups_trajectory_from_start_to_goal() {
    let goal = [0.8, -0.3, 0.2, -1.8, 0.1, 1.9, 0.5];
    let answer = plan(
        &request("panda_arm", vec![joint_goal(&ARM, &goal)]),
        &scene(vec![]),
        &READY,
    );
    succeeded(&answer);
    let t = &answer.trajectory.joint_trajectory;
    assert_eq!(t.joint_names, ARM);
    let first = &t.points[0].positions;
    let last = &t.points.last().unwrap().positions;
    for j in 0..7 {
        assert!(
            (first[j] - READY[j]).abs() < 1e-5,
            "start {j}: {}",
            first[j]
        );
        assert!((last[j] - goal[j]).abs() < 1e-4, "goal {j}: {}", last[j]);
    }
    // Waypoints every 20 ms, the last exactly at the end, at rest.
    let times: Vec<f64> = t
        .points
        .iter()
        .map(|p| seconds(&p.time_from_start))
        .collect();
    assert!(
        times.len() > 10 && times.windows(2).all(|w| w[1] > w[0]),
        "{times:?}"
    );
    for (k, &time) in times[..times.len() - 1].iter().enumerate() {
        assert!((time - 0.02 * k as f64).abs() < 1e-6, "point {k} at {time}");
    }
    // The motion ends between two 20 ms marks, so the last gap is shorter.
    let gap = times.last().unwrap() - times[times.len() - 2];
    assert!(gap > 0.0 && gap < 0.02 - 1e-6, "{gap}");
    assert!(
        t.points
            .last()
            .unwrap()
            .velocities
            .iter()
            .all(|v| v.abs() < 1e-4)
    );
    assert_eq!(
        answer.trajectory_start.joint_state.name,
        PLANNER.robot().joint_names()
    );
}

#[test]
fn groups_resolve_through_the_srdf() {
    let names = |group: &str, goal: Constraints| {
        let answer = plan(&request(group, vec![goal]), &scene(vec![]), &READY);
        succeeded(&answer);
        answer.trajectory.joint_trajectory.joint_names
    };
    assert_eq!(
        names("hand", joint_goal(&["panda_finger_joint1"], &[0.035])),
        ["panda_finger_joint1"]
    );
    let fingers = plan(
        &request("hand", vec![joint_goal(&["panda_finger_joint1"], &[0.035])]),
        &scene(vec![]),
        &READY,
    );
    let last = &fingers
        .trajectory
        .joint_trajectory
        .points
        .last()
        .unwrap()
        .positions;
    assert!((last[0] - 0.035).abs() < 1e-5, "{last:?}");
    let mut all = ARM.to_vec();
    all.push("panda_finger_joint1");
    let values = [READY.as_slice(), &[0.035]].concat();
    let both = names(
        "panda_arm_hand",
        joint_goal(&all, &[values[..6].to_vec(), vec![1.0, 0.035]].concat()),
    );
    assert_eq!(both, all);
    // A group's goal may name other joints only where they are.
    let mut arm_and_finger = joint_goal(&ARM, &READY);
    arm_and_finger
        .joint_constraints
        .extend(joint_goal(&["panda_finger_joint1"], &[0.02]).joint_constraints);
    arm_and_finger.joint_constraints[6].position = 1.2;
    assert_eq!(names("panda_arm", arm_and_finger), ARM);
}

#[test]
fn pose_goals_end_at_the_pose() {
    let target_q = full(&[0.4, -0.2, 0.1, -2.0, 0.2, 1.8, 0.3]);
    let target = PLANNER.robot().ee_pose(&target_q, 0);
    for frame in ["world", "panda_link0", ""] {
        let answer = plan(
            &request("panda_arm", vec![pose_goal("panda_link8", frame, target)]),
            &scene(vec![]),
            &READY,
        );
        succeeded(&answer);
        let last = answer.trajectory.joint_trajectory.points.len() - 1;
        let reached = PLANNER
            .robot()
            .ee_pose(&waypoint(&answer, last, &full(&READY)), 0);
        assert!(
            (reached.position - target.position).length() < 2e-3,
            "{frame}: {:?}",
            reached.position
        );
        assert!(
            reached.rotation.angle_between(target.rotation) < 2e-2,
            "{frame}"
        );
    }
    // A point 10 cm along the link's z axis reaches the target: the link stops short of it.
    let mut offset = pose_goal("panda_link8", "world", target);
    offset.position_constraints[0].target_point_offset.z = 0.1;
    let answer = plan(&request("panda_arm", vec![offset]), &scene(vec![]), &READY);
    succeeded(&answer);
    let last = answer.trajectory.joint_trajectory.points.len() - 1;
    let reached = PLANNER
        .robot()
        .ee_pose(&waypoint(&answer, last, &full(&READY)), 0);
    let point = reached.position + reached.rotation * Vec3::new(0.0, 0.0, 0.1);
    assert!((point - target.position).length() < 2e-3, "{point:?}");
}

#[test]
fn several_goals_end_at_the_nearest() {
    let near = [0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];
    let far = [-2.0, 0.3, 1.0, -1.0, -1.5, 2.5, -1.0];
    let goals = vec![joint_goal(&ARM, &far), joint_goal(&ARM, &near)];
    let answer = plan(&request("panda_arm", goals), &scene(vec![]), &READY);
    succeeded(&answer);
    let last = &answer
        .trajectory
        .joint_trajectory
        .points
        .last()
        .unwrap()
        .positions;
    assert!((last[0] - near[0]).abs() < 1e-4, "{last:?}");
}

#[test]
fn obstacles_in_the_scene_are_avoided() {
    let start = [-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785];
    let goal = [1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785];
    // A wall in front of the robot, between the two reaches.
    let wall = cuboid("wall", "world", [0.55, 0.0, 0.3], [0.1, 0.3, 0.6]);
    let robot = PLANNER.robot();
    let meshes = MeshModel::try_new(robot).unwrap();
    let world = World::new(vec![Obstacle::Cuboid {
        center: Vec3::new(0.55, 0.0, 0.3),
        half_extents: Vec3::new(0.05, 0.15, 0.3),
        rotation: Quat::IDENTITY,
    }]);
    let clear = |q: &[f32]| {
        meshes
            .clearance(std::slice::from_ref(&world), &[0], q)
            .unwrap()[0]
            .world
    };
    // The straight line between them goes through the wall.
    let (a, b) = (full(&start), full(&goal));
    let middle: Vec<f32> = a.iter().zip(&b).map(|(x, y)| 0.5 * (x + y)).collect();
    // Clearance is zero where meshes touch or overlap.
    assert!(clear(&a) > 0.0 && clear(&b) > 0.0 && clear(&middle) == 0.0);

    let answer = plan(
        &request("panda_arm", vec![joint_goal(&ARM, &goal)]),
        &scene(vec![wall]),
        &start,
    );
    succeeded(&answer);
    for k in 0..answer.trajectory.joint_trajectory.points.len() {
        let q = waypoint(&answer, k, &a);
        assert!(clear(&q) > 0.0, "waypoint {k} touches the wall");
    }
}

#[test]
fn closed_meshes_are_obstacles_and_open_ones_are_refused() {
    let start = [-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785];
    let goal = [1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785];
    // The same wall as a closed triangle mesh: a box's 8 corners, 12 triangles facing out.
    let (h, c) = ([0.05, 0.15, 0.3], [0.55, 0.0, 0.3]);
    let vertices = (0..8)
        .map(|i| geometry::Point {
            x: c[0] + if i & 1 == 0 { -h[0] } else { h[0] },
            y: c[1] + if i & 2 == 0 { -h[1] } else { h[1] },
            z: c[2] + if i & 4 == 0 { -h[2] } else { h[2] },
        })
        .collect();
    let faces = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let triangles: Vec<MeshTriangle> = faces
        .iter()
        .flat_map(|&[a, b, c, d]| [[a, b, c], [a, c, d]])
        .map(|vertex_indices| MeshTriangle { vertex_indices })
        .collect();
    let mesh = |triangles: Vec<MeshTriangle>| {
        let mut object = CollisionObject {
            id: "wall".into(),
            operation: CollisionObject::ADD,
            ..Default::default()
        };
        object.header.frame_id = "world".into();
        object.meshes.push(Mesh {
            triangles,
            vertices: vec![],
        });
        object.meshes[0].vertices.clone_from(&vertices);
        object.mesh_poses.push(geometry::Pose::default());
        object
    };
    let blocked = |objects| {
        plan(
            &request("panda_arm", vec![joint_goal(&ARM, &goal)]),
            &scene(objects),
            &start,
        )
    };
    let free = blocked(vec![]);
    succeeded(&free);
    let with_mesh = blocked(vec![mesh(triangles.clone())]);
    succeeded(&with_mesh);
    // The wall makes the robot go around it: the path gets longer.
    let length = |a: &Answer| {
        let p = &a.trajectory.joint_trajectory.points;
        p.windows(2)
            .map(|w| {
                w[0].positions
                    .iter()
                    .zip(&w[1].positions)
                    .map(|(x, y)| (x - y).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum::<f64>()
    };
    assert!(
        length(&with_mesh) > length(&free) + 0.1,
        "{} vs {}",
        length(&with_mesh),
        length(&free)
    );

    let open = blocked(vec![mesh(triangles[..10].to_vec())]);
    assert_eq!(
        open.error_code.val,
        Code::INVALID_OBJECT_NAME,
        "{}",
        open.error_code.message
    );
}

#[test]
fn frames_resolve_to_links_and_fixed_transforms() {
    // 11 cm behind the hand, on the wrist; taken as world coordinates, below the robot's base and
    // clear.
    let (behind, size) = ([0.0, 0.0, -0.11], [0.1, 0.1, 0.1]);
    let goal = joint_goal(&ARM, &[0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]);
    let outcome =
        |s: &PlanningScene| plan(&request("panda_arm", vec![goal.clone()]), s, &READY).error_code;
    let in_world = outcome(&scene(vec![cuboid("box", "world", behind, size)]));
    assert_eq!(in_world.val, Code::SUCCESS, "{}", in_world.message);
    let on_arm = outcome(&scene(vec![cuboid("box", "panda_hand", behind, size)]));
    assert_eq!(
        on_arm.val,
        Code::START_STATE_IN_COLLISION,
        "{}",
        on_arm.message
    );

    // The same box in a frame the scene places at the hand.
    let hand = PLANNER
        .robot()
        .link_pose(&full(&READY), "panda_hand")
        .unwrap();
    let mut placed = scene(vec![cuboid("box", "table", behind, size)]);
    let mut table = geometry::TransformStamped {
        child_frame_id: "table".into(),
        ..Default::default()
    };
    table.header.frame_id = "world".into();
    let p = pose_msg(hand);
    table.transform.translation = geometry::Vector3 {
        x: p.position.x,
        y: p.position.y,
        z: p.position.z,
    };
    table.transform.rotation = p.orientation;
    placed.fixed_frame_transforms.push(table);
    let answer = outcome(&placed);
    assert_eq!(
        answer.val,
        Code::START_STATE_IN_COLLISION,
        "{}",
        answer.message
    );

    // A box and a cylinder 10 cm beside the hand (along its fingers' axis) leave the start free.
    let side = hand.rotation * Vec3::Y;
    let near = |d: f32| hand.position + side * d;
    let block = near(0.3);
    let beside_box = cuboid(
        "box",
        "world",
        [block.x, block.y, block.z].map(f64::from),
        [0.2, 0.2, 0.2],
    );
    let mut beside_cylinder = cuboid("can", "world", [0.0; 3], [0.1; 3]);
    let can = near(0.25);
    beside_cylinder.primitives[0] = SolidPrimitive {
        type_: SolidPrimitive::CYLINDER,
        dimensions: [0.3, 0.05].as_slice().try_into().unwrap(),
        ..Default::default()
    };
    beside_cylinder.primitive_poses[0] = pose_msg(Pose {
        position: can,
        rotation: Quat::IDENTITY,
    });
    for object in [beside_box, beside_cylinder] {
        let answer = plan(
            &request("panda_arm", vec![goal.clone()]),
            &scene(vec![object.clone()]),
            &READY,
        );
        assert_ne!(
            answer.error_code.val,
            Code::START_STATE_IN_COLLISION,
            "{}: {}",
            object.id,
            answer.error_code.message
        );
    }

    let unknown = plan(
        &request("panda_arm", vec![goal]),
        &scene(vec![cuboid("box", "nowhere", [0.0; 3], [0.1; 3])]),
        &READY,
    );
    assert_eq!(unknown.error_code.val, Code::FRAME_TRANSFORM_FAILURE);
}

#[test]
fn a_moving_base_places_the_planning_frame() {
    let hand = FLOATING
        .robot()
        .link_pose(&full(&READY), "panda_hand")
        .unwrap()
        .position;
    let request = request(
        "panda_arm",
        vec![joint_goal(
            &ARM,
            &[0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        )],
    );
    // A box 1 m along x from the hand, given in the world frame.
    let box_scene = scene(vec![cuboid(
        "box",
        "world",
        [hand.x + 1.0, hand.y, hand.z].map(f64::from),
        [0.1; 3],
    )]);
    succeeded(&FLOATING.plan(&request, &box_scene, Some(&current(&READY))));
    // With the robot standing 1 m along x in the world, the box is at its hand.
    let mut moved = box_scene;
    let mut base = geometry::Transform::default();
    base.translation.x = 1.0;
    moved.robot_state.multi_dof_joint_state.joint_names = vec!["virtual_joint".into()];
    moved.robot_state.multi_dof_joint_state.transforms = vec![base];
    let hit = FLOATING.plan(&request, &moved, Some(&current(&READY)));
    assert_eq!(
        hit.error_code.val,
        Code::START_STATE_IN_COLLISION,
        "{}",
        hit.error_code.message
    );
}

#[test]
fn attached_objects_move_with_their_link() {
    // A long bar held in the hand; a post beside the hand that the bare hand clears but the bar
    // hits.
    let hand = PLANNER
        .robot()
        .link_pose(&full(&READY), "panda_hand")
        .unwrap();
    let beside = hand.position + hand.rotation * Vec3::new(0.0, 0.3, 0.1);
    let post = cuboid(
        "post",
        "world",
        [beside.x, beside.y, beside.z].map(f64::from),
        [0.05, 0.05, 0.05],
    );
    let goal = joint_goal(&ARM, &[0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]);
    let free = plan(
        &request("panda_arm", vec![goal.clone()]),
        &scene(vec![post.clone()]),
        &READY,
    );
    succeeded(&free);

    let mut holding = scene(vec![post]);
    let bar = cuboid("bar", "panda_hand", [0.0, 0.0, 0.1], [0.04, 0.7, 0.04]);
    holding
        .robot_state
        .attached_collision_objects
        .push(AttachedCollisionObject {
            link_name: "panda_hand".into(),
            object: bar,
            touch_links: vec!["panda_leftfinger".into(), "panda_rightfinger".into()],
            ..Default::default()
        });
    let held = plan(&request("panda_arm", vec![goal]), &holding, &READY);
    assert_eq!(
        held.error_code.val,
        Code::START_STATE_IN_COLLISION,
        "{}",
        held.error_code.message
    );
}

#[test]
fn velocity_scaling_slows_the_motion() {
    let goal = [0.8, -0.3, 0.2, -1.8, 0.1, 1.9, 0.5];
    let duration = |scale: f64| {
        let mut r = request("panda_arm", vec![joint_goal(&ARM, &goal)]);
        (
            r.max_velocity_scaling_factor,
            r.max_acceleration_scaling_factor,
        ) = (scale, scale * scale);
        let answer = plan(&r, &scene(vec![]), &READY);
        succeeded(&answer);
        seconds(
            &answer
                .trajectory
                .joint_trajectory
                .points
                .last()
                .unwrap()
                .time_from_start,
        )
    };
    let (full_speed, half) = (duration(1.0), duration(0.5));
    assert!(
        half > 1.9 * full_speed && half < 2.1 * full_speed,
        "{half} vs {full_speed}"
    );
}

#[test]
fn refusals_carry_moveit_codes() {
    let goal = joint_goal(&ARM, &[0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]);
    let code = |r: &MotionPlanRequest, start: &[f64]| plan(r, &scene(vec![]), start).error_code.val;

    assert_eq!(
        code(&request("legs", vec![goal.clone()]), &READY),
        Code::INVALID_GROUP_NAME
    );
    assert_eq!(
        code(&request("panda_arm", vec![]), &READY),
        Code::INVALID_GOAL_CONSTRAINTS
    );
    let partial = joint_goal(&ARM[..6], &READY[..6]);
    assert_eq!(
        code(&request("panda_arm", vec![partial]), &READY),
        Code::INVALID_GOAL_CONSTRAINTS
    );
    let beyond = joint_goal(&ARM, &[3.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]);
    assert_eq!(
        code(&request("panda_arm", vec![beyond]), &READY),
        Code::INVALID_GOAL_CONSTRAINTS
    );
    let mut moves_finger = goal.clone();
    moves_finger
        .joint_constraints
        .extend(joint_goal(&["panda_finger_joint1"], &[0.04]).joint_constraints);
    assert_eq!(
        code(&request("panda_arm", vec![moves_finger]), &READY),
        Code::INVALID_GOAL_CONSTRAINTS
    );

    let mut constrained = request("panda_arm", vec![goal.clone()]);
    constrained.path_constraints = joint_goal(&ARM, &READY);
    assert_eq!(code(&constrained, &READY), Code::INVALID_GOAL_CONSTRAINTS);

    // Far out of reach.
    let away = Pose {
        position: Vec3::new(3.0, 0.0, 0.5),
        rotation: Quat::IDENTITY,
    };
    assert_eq!(
        code(
            &request("panda_arm", vec![pose_goal("panda_link8", "world", away)]),
            &READY
        ),
        Code::NO_IK_SOLUTION
    );
    let wrong_link = pose_goal("panda_link3", "world", away);
    assert_eq!(
        code(&request("panda_arm", vec![wrong_link]), &READY),
        Code::INVALID_GOAL_CONSTRAINTS
    );

    // Starts a hair outside a limit are moved onto it; farther out they are invalid.
    let mut hair = READY;
    hair[3] = 0.0873 + 5e-4;
    assert_eq!(
        code(&request("panda_arm", vec![goal.clone()]), &hair),
        Code::SUCCESS
    );
    hair[3] = 0.0873 + 0.05;
    assert_eq!(
        code(&request("panda_arm", vec![goal.clone()]), &hair),
        Code::INVALID_ROBOT_STATE
    );
    // A goal inside a box.
    let turned = [1.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];
    let p = PLANNER
        .robot()
        .link_pose(&full(&turned), "panda_hand")
        .unwrap()
        .position;
    let at_goal = scene(vec![cuboid(
        "box",
        "world",
        [p.x, p.y, p.z].map(f64::from),
        [0.1; 3],
    )]);
    let into = plan(
        &request("panda_arm", vec![joint_goal(&ARM, &turned)]),
        &at_goal,
        &READY,
    );
    assert_eq!(
        into.error_code.val,
        Code::GOAL_IN_COLLISION,
        "{}",
        into.error_code.message
    );
    // No time to search round a wall.
    let start = [-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785];
    let mut hurried = request(
        "panda_arm",
        vec![joint_goal(&ARM, &[1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785])],
    );
    hurried.allowed_planning_time = 1e-6;
    let wall = scene(vec![cuboid(
        "wall",
        "world",
        [0.55, 0.0, 0.3],
        [0.1, 0.3, 0.6],
    )]);
    assert_eq!(
        plan(&hurried, &wall, &start).error_code.val,
        Code::TIMED_OUT
    );

    let missing = PLANNER.plan(
        &request("panda_arm", vec![goal]),
        &scene(vec![]),
        Some(&state(&ARM[..3], &READY[..3])),
    );
    assert_eq!(
        missing.error_code.val,
        Code::INVALID_ROBOT_STATE,
        "{}",
        missing.error_code.message
    );
}

#[test]
fn ik_finds_a_configuration_at_the_pose() {
    let target = PLANNER
        .robot()
        .ee_pose(&full(&[0.4, -0.2, 0.1, -2.0, 0.2, 1.8, 0.3]), 0);
    let mut ik = PositionIKRequest {
        group_name: "panda_arm".into(),
        ik_link_name: "panda_link8".into(),
        ..Default::default()
    };
    ik.pose_stamped.header.frame_id = "world".into();
    ik.pose_stamped.pose = pose_msg(target);
    let (solution, error) = PLANNER.ik(&ik, Some(&current(&READY)));
    assert_eq!(error.val, Code::SUCCESS, "{}", error.message);
    let q: Vec<f32> = solution
        .joint_state
        .position
        .iter()
        .map(|&v| v as f32)
        .collect();
    let reached = PLANNER.robot().ee_pose(&q, 0);
    assert!((reached.position - target.position).length() < 2e-3);

    ik.ik_link_name = "panda_link3".into();
    assert_eq!(
        PLANNER.ik(&ik, Some(&current(&READY))).1.val,
        Code::INVALID_LINK_NAME
    );
}

#[test]
fn joint_limits_files_give_the_limits_they_enable() {
    let limits = batchplan_ros::parse_joint_limits(
        "default_velocity_scaling_factor: 0.1\njoint_limits:\n  panda_joint1:\n    has_velocity_limits: true\n    max_velocity: 2.0\n    has_acceleration_limits: false\n    max_acceleration: 0.0\n  panda_joint2:\n    has_acceleration_limits: true\n    max_acceleration: 3.75\n",
    )
    .unwrap();
    assert_eq!(
        (
            limits["panda_joint1"].velocity,
            limits["panda_joint1"].acceleration
        ),
        (Some(2.0), None)
    );
    assert_eq!(
        (
            limits["panda_joint2"].velocity,
            limits["panda_joint2"].acceleration
        ),
        (None, Some(3.75))
    );
}

#[test]
fn lin_moves_the_hand_in_a_straight_line() {
    let start = PLANNER.robot().ee_pose(&full(&READY), 0);
    let goal = Pose::new(start.position + Vec3::new(0.1, 0.1, -0.2), start.rotation);
    let answer = plan(&lin(goal), &scene(vec![]), &READY);
    succeeded(&answer);
    let path = hand_path(&answer.trajectory, &READY);
    let off = path
        .iter()
        .map(|&(_, p)| off_segment(p, start.position, goal.position))
        .fold(0.0, f32::max);
    assert!(
        off < 2e-3,
        "the hand strays {:.1} mm from the line",
        off * 1e3
    );
    let end = path.last().unwrap().1;
    assert!((end - goal.position).length() < 2e-3);
    // A planned motion to the same pose leaves the line, so the check above tells them apart.
    let planned = plan(
        &request("panda_arm", vec![pose_goal("panda_link8", "world", goal)]),
        &scene(vec![]),
        &READY,
    );
    succeeded(&planned);
    let strays = hand_path(&planned.trajectory, &READY)
        .iter()
        .map(|&(_, p)| off_segment(p, start.position, goal.position))
        .fold(0.0, f32::max);
    assert!(
        strays > 5e-3,
        "the planned motion strays only {:.1} mm",
        strays * 1e3
    );

    // A box across the line, clear of the start and the goal, stops it; LIN takes no joint goals.
    let on_line = start.position.lerp(goal.position, 0.6);
    let across = cuboid("box", "world", on_line.to_array().map(f64::from), [0.04; 3]);
    let blocked = plan(&lin(goal), &scene(vec![across]), &READY);
    assert_eq!(blocked.error_code.val, Code::PLANNING_FAILED);
    assert!(
        blocked.error_code.message.contains("collide"),
        "{}",
        blocked.error_code.message
    );
    let joints = MotionPlanRequest {
        planner_id: "LIN".into(),
        ..request("panda_arm", vec![joint_goal(&ARM, &READY)])
    };
    assert_eq!(
        plan(&joints, &scene(vec![]), &READY).error_code.val,
        Code::INVALID_GOAL_CONSTRAINTS
    );
}

#[test]
fn the_allowed_collision_matrix_lets_links_touch_objects() {
    let at = PLANNER
        .robot()
        .link_pose(&full(&READY), "panda_leftfinger")
        .unwrap()
        .position;
    let block = cuboid("block", "world", at.to_array().map(f64::from), [0.02; 3]);
    let away = request(
        "panda_arm",
        vec![joint_goal(&ARM, &[0.3, -0.5, 0.0, -2.2, 0.0, 1.7, 0.785])],
    );
    let mut touching = scene(vec![block]);
    let code = |s: &PlanningScene| plan(&away, s, &READY).error_code.val;
    assert_eq!(code(&touching), Code::START_STATE_IN_COLLISION);
    let hand = ["panda_hand", "panda_leftfinger", "panda_rightfinger"];
    touching.allowed_collision_matrix = allowing("block", &hand, true);
    assert_eq!(code(&touching), Code::SUCCESS);
    // MoveIt reads a pair from the matrix's upper triangle: the block's row here, as it comes first.
    let mut upper = allowing("block", &hand, true);
    upper.entry_values[1..]
        .iter_mut()
        .for_each(|row| row.enabled[0] = false);
    touching.allowed_collision_matrix = upper;
    assert_eq!(code(&touching), Code::SUCCESS);
    let mut lower = allowing("block", &hand, true);
    lower.entry_values[0].enabled.fill(false);
    touching.allowed_collision_matrix = lower;
    assert_eq!(code(&touching), Code::START_STATE_IN_COLLISION);
    // A default entry lets every link touch the block...
    touching.allowed_collision_matrix = AllowedCollisionMatrix {
        default_entry_names: vec!["block".into()],
        default_entry_values: vec![true],
        ..Default::default()
    };
    assert_eq!(code(&touching), Code::SUCCESS);
    // ...unless an entry for the pair says otherwise.
    let defaults = touching.allowed_collision_matrix.clone();
    touching.allowed_collision_matrix = AllowedCollisionMatrix {
        default_entry_names: defaults.default_entry_names,
        default_entry_values: defaults.default_entry_values,
        ..allowing("block", &hand, false)
    };
    assert_eq!(code(&touching), Code::START_STATE_IN_COLLISION);
}

#[test]
fn a_cartesian_speed_cap_slows_the_hand() {
    let goal = [0.8, -0.3, 0.2, -1.8, 0.1, 1.9, 0.5];
    let free = request("panda_arm", vec![joint_goal(&ARM, &goal)]);
    let run = |r: &MotionPlanRequest| {
        let answer = plan(r, &scene(vec![]), &READY);
        succeeded(&answer);
        answer
    };
    let uncapped = fastest(&hand_path(&run(&free).trajectory, &READY));
    assert!(
        uncapped > 0.3,
        "the hand moves at only {uncapped} m/s uncapped"
    );
    let mut capped = MotionPlanRequest {
        cartesian_speed_limited_link: "panda_link8".into(),
        max_cartesian_speed: 0.1,
        ..free
    };
    let speed = fastest(&hand_path(&run(&capped).trajectory, &READY));
    assert!(speed < 0.1 * 1.02, "the hand reaches {speed} m/s");
    // With the joints slowed as well, both hold.
    (
        capped.max_velocity_scaling_factor,
        capped.max_acceleration_scaling_factor,
    ) = (0.3, 0.09);
    let slowed = run(&capped);
    let speed = fastest(&hand_path(&slowed.trajectory, &READY));
    assert!(speed < 0.1 * 1.02, "the hand reaches {speed} m/s");
    let robot = PLANNER.robot();
    let t = &slowed.trajectory.joint_trajectory;
    for point in &t.points {
        for (name, v) in t.joint_names.iter().zip(&point.velocities) {
            let j = robot.joint_names().iter().position(|n| n == name).unwrap();
            let limit = 0.3 * f64::from(robot.max_velocity()[j]);
            assert!(
                v.abs() <= limit * 1.001,
                "{name} at {v} rad/s, over {limit}"
            );
        }
    }
    capped.cartesian_speed_limited_link = "panda_link3".into();
    assert_eq!(
        plan(&capped, &scene(vec![]), &READY).error_code.val,
        Code::INVALID_LINK_NAME
    );
}

#[test]
fn cartesian_paths_follow_the_waypoints_as_far_as_they_reach() {
    let start = PLANNER.robot().ee_pose(&full(&READY), 0);
    let down = Pose::new(start.position + Vec3::new(0.0, 0.0, -0.15), start.rotation);
    let across = Pose::new(down.position + Vec3::new(0.0, 0.15, 0.0), start.rotation);
    let mut asked = GetCartesianPath_Request {
        group_name: "panda_arm".into(),
        link_name: "panda_link8".into(),
        waypoints: vec![pose_msg(down), pose_msg(across)],
        max_cartesian_speed: 0.1,
        ..Default::default()
    };
    asked.header.frame_id = "world".into();
    let answer = PLANNER.cartesian_path(&asked, Some(&current(&READY)));
    assert_eq!(
        answer.error_code.val,
        Code::SUCCESS,
        "{}",
        answer.error_code.message
    );
    assert_eq!(answer.fraction, 1.0);
    let path = hand_path(&answer.solution, &READY);
    for &(_, p) in &path {
        let off = off_segment(p, start.position, down.position).min(off_segment(
            p,
            down.position,
            across.position,
        ));
        assert!(
            off < 2e-3,
            "the hand strays {:.1} mm from the lines",
            off * 1e3
        );
    }
    assert!((path.last().unwrap().1 - across.position).length() < 2e-3);
    assert!(fastest(&path) < 0.1 * 1.02);

    // A last waypoint out of reach: the trajectory covers what the arm reaches, and says how much.
    asked.waypoints.push(pose_msg(Pose::new(
        across.position + Vec3::new(2.0, 0.0, 0.0),
        start.rotation,
    )));
    let partial = PLANNER.cartesian_path(&asked, Some(&current(&READY)));
    assert_eq!(
        partial.error_code.val,
        Code::SUCCESS,
        "{}",
        partial.error_code.message
    );
    assert!(
        partial.fraction > 0.1 && partial.fraction < 1.0,
        "fraction {}",
        partial.fraction
    );
    let reached = hand_path(&partial.solution, &READY);
    assert!(
        reached
            .iter()
            .any(|&(_, p)| (p - across.position).length() < 2e-3)
    );
}
