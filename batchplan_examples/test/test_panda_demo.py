"""The Panda demo over ROS: the planner node, its parameters, and the controller it executes through.

Run with `pixi run test-demo`; it launches the demo without the browser scene on a domain of its own.
"""

import io
import os
import random
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
import trimesh
import yourdfpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from batchplan_viser.app import resolve
from batchplan_viser.messages import APPROACH, FINGER, box_object, cylinder_object, grasp_poses, pose_goal, pose_msg

ARM = [f"panda_joint{i}" for i in range(1, 8)]


@pytest.fixture(scope="module")
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    demo = subprocess.Popen(
        ["ros2", "launch", "batchplan_examples", "panda_demo.launch.py", "ui:=false"],
        stdout=open(os.environ.get("DEMO_LOG", os.devnull), "w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("panda_demo_test")
    joints = {}
    node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
    planner = ActionClient(node, MoveGroup, "/batchplan_planner/move_group")
    assert planner.wait_for_server(timeout_sec=60.0), "the planner did not start"
    # Discovery finishes after the server first shows up; a goal sent at once can lose its answer.
    deadline, settled = time.time() + 30, time.time() + 1.0
    while (len(joints) < 8 or time.time() < settled) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    yield node, planner, joints
    rclpy.shutdown()
    os.killpg(demo.pid, signal.SIGINT)
    demo.wait(timeout=30)


def run(ros, goal):
    node, planner, _ = ros
    sent = planner.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=30)
    result = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=60)
    return result.result().result


def settle(ros):
    node = ros[0]
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def request(goal_constraints, plan_only=True, objects=()):
    goal = MoveGroup.Goal()
    goal.request.group_name = "panda_arm"
    goal.request.goal_constraints = goal_constraints
    goal.planning_options.plan_only = plan_only
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.world.collision_objects = list(objects)
    return goal


def joint_goal(values):
    return Constraints(joint_constraints=[JointConstraint(joint_name=n, position=v, weight=1.0) for n, v in zip(ARM, values)])


def test_an_executed_plan_ends_at_its_goal(ros):
    target = [0.6, -0.4, 0.2, -2.0, 0.1, 1.8, 0.6]
    result = run(ros, request([joint_goal(target)], plan_only=False))
    assert result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.message
    assert result.executed_trajectory.joint_trajectory.points
    settle(ros)
    joints = ros[2]
    assert max(abs(joints[n] - v) for n, v in zip(ARM, target)) < 1e-3


def test_a_plan_only_goal_leaves_the_arm_where_it_is(ros):
    settle(ros)
    before = dict(ros[2])
    result = run(ros, request([joint_goal([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])]))
    assert result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.message
    assert not result.executed_trajectory.joint_trajectory.points
    settle(ros)
    assert max(abs(ros[2][n] - before[n]) for n in ARM) < 1e-6


def test_pose_goals_plan_around_boxes(ros):
    box = box_object("box", "panda_link0", (0.5, 0.0, 0.15), (1.0, 0.0, 0.0, 0.0), (0.12, 0.12, 0.3))
    goal = pose_goal("panda_link8", "world", (0.4, 0.3, 0.35), (0.0, 1.0, 0.0, 0.0))
    result = run(ros, request([goal], objects=[box]))
    assert result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.message
    assert len(result.planned_trajectory.joint_trajectory.points) > 2


def test_unreachable_goals_are_refused(ros):
    goal = pose_goal("panda_link8", "world", (2.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.0))
    result = run(ros, request([goal]))
    assert result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION, result.error_code.message


def test_ik_solves_for_the_link(ros):
    node = ros[0]
    ik = node.create_client(GetPositionIK, "/batchplan_planner/compute_ik")
    assert ik.wait_for_service(timeout_sec=10.0)
    request = GetPositionIK.Request()
    request.ik_request.ik_link_name = "panda_link8"
    request.ik_request.pose_stamped.header.frame_id = "world"
    request.ik_request.pose_stamped.pose = pose_msg((0.4, 0.1, 0.5), (0.0, 1.0, 0.0, 0.0))
    answer = ik.call_async(request)
    rclpy.spin_until_future_complete(node, answer, timeout_sec=10)
    assert answer.result().error_code.val == MoveItErrorCodes.SUCCESS, answer.result().error_code.message
    assert "panda_joint1" in answer.result().solution.joint_state.name


def clearance_to_cylinder(ros, trajectory, center, radius: float, height: float) -> float:
    """The least distance from the robot's collision meshes to a vertical cylinder along a
    trajectory, measured at points sampled on the meshes."""
    node, _, joints = ros
    description = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/robot_description", lambda m: description.append(m.data), latched)
    while not description:
        rclpy.spin_once(node, timeout_sec=0.1)
    robot = yourdfpy.URDF.load(
        io.StringIO(description[0]),
        filename_handler=resolve,
        load_meshes=False,
        build_collision_scene_graph=True,
        load_collision_meshes=True,
        force_collision_mesh=False,
    )
    scene = robot.collision_scene
    samples = []
    for name in scene.graph.nodes_geometry:
        mesh = scene.geometry[scene.graph[name][1]]
        points, _ = trimesh.sample.sample_surface(mesh, 4000, seed=1)
        samples.append((name, np.vstack([points, mesh.vertices])))
    least = np.inf
    for point in trajectory.points:
        robot.update_cfg({**joints, **dict(zip(trajectory.joint_names, point.positions))})
        for name, local in samples:
            transform = scene.graph.get(name)[0]
            world = local @ transform[:3, :3].T + transform[:3, 3]
            beside = np.abs(world[:, 2] - center[2]) < height / 2
            if beside.any():
                least = min(least, np.min(np.hypot(*(world[beside, :2] - center[:2]).T) - radius))
    return least


def test_the_hand_moves_in_around_the_can_and_the_fingers_close_on_it(ros):
    """The browser demo's best grasp, for one grasp: to the pose before it with the can in the scene,
    then in to the grasp without the can, as the fingers may touch what they grasp. The robot's
    meshes keep clear of the can and end close around it, and the fingers close on it."""
    center, radius, height = np.array([0.45, 0.25, 0.1]), 0.03, 0.2
    (grasp,), (approach,) = grasp_poses(center, 1), grasp_poses(center, 1, back=APPROACH)
    can = cylinder_object("can", "world", center, radius, height)
    before = run(ros, request([pose_goal("panda_link8", "world", *approach)], plan_only=False, objects=[can]))
    assert before.error_code.val == MoveItErrorCodes.SUCCESS, before.error_code.message
    settle(ros)
    inward = run(ros, request([pose_goal("panda_link8", "world", *grasp)], plan_only=False))
    assert inward.error_code.val == MoveItErrorCodes.SUCCESS, inward.error_code.message
    clearance = clearance_to_cylinder(ros, inward.planned_trajectory.joint_trajectory, center, radius, height)
    assert 0.0 < clearance < 0.015, f"the robot's meshes come within {clearance * 1e3:.1f} mm of the can"

    node, _, joints = ros
    hand = ActionClient(node, FollowJointTrajectory, "/panda_hand_controller/follow_joint_trajectory")
    assert hand.wait_for_server(timeout_sec=10.0), "no gripper controller"
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [FINGER]
    goal.trajectory.points = [JointTrajectoryPoint(positions=[radius], time_from_start=Duration(nanosec=500_000_000))]
    sent = hand.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
    done = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, done, timeout_sec=10)
    assert done.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    settle(ros)
    assert abs(joints[FINGER] - radius) < 1e-3, f"the fingers stopped at {joints[FINGER]:.4f} m"


def test_the_planner_outlives_clients_that_come_and_go(ros):
    """Clients that send a goal and leave, as scripts do when they end. On Lyrical, rclrs 0.8.0 took
    a wake-up that followed for an error and ended the node (ros2-rust/ros2_rust#730)."""
    goal = request([joint_goal([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])])
    for k in range(3):
        visitor = rclpy.create_node(f"panda_demo_visitor_{k}")
        client = ActionClient(visitor, MoveGroup, "/batchplan_planner/move_group")
        assert client.wait_for_server(timeout_sec=10.0)
        settled = time.time() + 1.0
        while time.time() < settled:
            rclpy.spin_once(visitor, timeout_sec=0.1)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(visitor, sent, timeout_sec=10)
        assert sent.result(), f"client {k} got no answer"
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(visitor, result, timeout_sec=10)
        assert result.result(), f"client {k} got no answer"
        client.destroy()
        visitor.destroy_node()
