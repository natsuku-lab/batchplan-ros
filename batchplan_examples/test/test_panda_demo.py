"""The Panda demo over ROS: the planner node, its parameters, and the controller it executes through.

Run with `pixi run test-demo`; it launches the demo without the browser scene on a domain of its own.
"""

import os
import random
import signal
import subprocess
import time

import pytest
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from batchplan_viser.app import box_object, pose_goal, pose_msg

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
