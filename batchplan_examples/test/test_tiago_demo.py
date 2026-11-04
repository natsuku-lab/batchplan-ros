"""The TIAGo Pro demo over ROS: its base and arms planned as one, a tray carried level between both
hands, and the whole-body and gripper controllers it executes through.

Run with `pixi run test-demo`, which fetches the robot first; it launches the demo without the
browser scene on a domain of its own.
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
import yourdfpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from viser.transforms import SO3

from batchplan_viser import tray
from batchplan_viser.app import resolve
from batchplan_viser.messages import pose_goal

FRAME = "odom"


@pytest.fixture(scope="module")
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    demo = subprocess.Popen(
        ["ros2", "launch", "batchplan_examples", "demo.launch.py", "robot:=tiago_pro", "ui:=false"],
        stdout=open(os.environ.get("DEMO_LOG", os.devnull), "w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("tiago_demo_test")
    joints, description = {}, []
    node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/robot_description", lambda m: description.append(m.data), latched)
    planner = ActionClient(node, MoveGroup, "/batchplan_planner/move_group")
    assert planner.wait_for_server(timeout_sec=60.0), "the planner did not start"
    # Discovery finishes after the server first shows up; a goal sent at once can lose its answer.
    deadline, settled = time.time() + 30, time.time() + 1.0
    while (len(joints) < 22 or not description or time.time() < settled) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    robot = yourdfpy.URDF.load(io.StringIO(description[0]), filename_handler=resolve, load_meshes=False, build_collision_scene_graph=False)
    yield node, planner, joints, robot
    rclpy.shutdown()
    os.killpg(demo.pid, signal.SIGINT)
    demo.wait(timeout=30)


def run(ros, client, goal):
    node = ros[0]
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=30)
    result = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=120)
    return result.result().result


def transform(robot: yourdfpy.URDF, joints: dict, link: str) -> np.ndarray:
    robot.update_cfg({name: joints[name] for name in robot.actuated_joint_names if name in joints})
    return robot.get_transform(link, FRAME)


def test_the_tray_is_carried_level_from_table_to_table(ros):
    """Every leg plans, the base driving round the wall; along the legs that carry the tray, the
    left hand stays within 3° of level and the right within 2 mm and 0.02 rad of its place in the
    left's frame (the planner's floors relative to another hand) at every waypoint."""
    node, planner, joints, robot = ros

    def send(request: MotionPlanRequest, objects, allowed):
        goal = MoveGroup.Goal(request=request)
        goal.planning_options.plan_only = True
        scene = goal.planning_options.planning_scene_diff
        scene.is_diff, scene.world.collision_objects = True, objects
        if allowed is not None:
            scene.allowed_collision_matrix = allowed
        result = run(ros, planner, goal)
        assert result.error_code.val == MoveItErrorCodes.SUCCESS, f"{result.error_code.val}: {result.error_code.message}"
        return result

    touch = [link for link in robot.link_map if link.startswith("gripper_")]
    legs = tray.carry(send, lambda _: True, FRAME, touch, dict(joints))
    assert [name for name, _ in legs] == ["reach", "approach", "lift", "carry", "lower", "retreat"]
    place = np.linalg.inv(tray.hands(tray.PICK)[0].as_matrix()) @ tray.hands(tray.PICK)[1].as_matrix()
    drove = 0.0
    for name, result in legs:
        trajectory = result.planned_trajectory.joint_trajectory
        start = dict(zip(result.trajectory_start.joint_state.name, result.trajectory_start.joint_state.position))
        for point in trajectory.points:
            q = {**start, **dict(zip(trajectory.joint_names, point.positions))}
            drove = max(drove, np.hypot(q["base_joint_x"], q["base_joint_y"]))
            if name not in ("lift", "carry", "lower"):
                continue
            left, right = transform(robot, q, tray.LEFT), transform(robot, q, tray.RIGHT)
            tilt = np.arccos(np.clip(left[2, 2], -1.0, 1.0))
            now = np.linalg.inv(left) @ right
            away = np.abs(now[:3, 3] - place[:3, 3]).max()
            turned = np.linalg.norm(SO3.from_matrix(place[:3, :3].T @ now[:3, :3]).log())
            assert tilt <= tray.LEVEL + 1e-3, f"{name}: the tray tilts {np.degrees(tilt):.2f}°"
            assert away <= 2e-3 + 1e-4 and turned <= 0.02 + 1e-3, f"{name}: the right hand strays {away * 1e3:.2f} mm, {turned:.4f} rad"
    assert drove > 2.0, f"the base drives only {drove:.2f} m from where it starts"


def test_whole_body_moves_and_the_grippers_run_on_the_robot(ros):
    """A move of the left hand that the base and left arm make together, executed through the
    whole-body controller, which takes the group's joints alone, ends at the goal; each gripper
    closes through its own controller."""
    node, planner, joints, robot = ros
    hand = transform(robot, joints, tray.LEFT)
    target = hand[:3, 3] + np.array([0.6, 0.2, 0.0])
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest(group_name="base_left_arm", allowed_planning_time=5.0)
    goal.request.goal_constraints = [pose_goal(tray.LEFT, FRAME, target, SO3.from_matrix(hand[:3, :3]).wxyz)]
    result = run(ros, planner, goal)
    assert result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.message
    assert len(result.planned_trajectory.joint_trajectory.joint_names) == 10
    time.sleep(0.5)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    reached = transform(robot, joints, tray.LEFT)[:3, 3]
    assert np.linalg.norm(reached - target) < 5e-3, f"the hand ends {np.linalg.norm(reached - target) * 1e3:.1f} mm from the goal"
    assert np.hypot(joints["base_joint_x"], joints["base_joint_y"]) > 0.05, "the base did not move"
    for controller, joint in tray.GRIPPERS:
        client = ActionClient(node, FollowJointTrajectory, f"{controller}/follow_joint_trajectory")
        assert client.wait_for_server(timeout_sec=10.0)
        close = FollowJointTrajectory.Goal()
        close.trajectory.joint_names = [joint]
        close.trajectory.points = [JointTrajectoryPoint(positions=[tray.CLOSED], time_from_start=Duration(nanosec=500_000_000))]
        assert run(ros, client, close).error_code == FollowJointTrajectory.Result.SUCCESSFUL
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert abs(joints[joint] - tray.CLOSED) < 1e-3, f"{joint} at {joints[joint]}"
