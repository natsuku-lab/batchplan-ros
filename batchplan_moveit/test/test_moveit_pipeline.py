"""batchplan as a MoveIt planner: the stock Panda demo with the batchplan pipeline, driven through
move_group's own interfaces. Run with `pixi run -e moveit test-moveit`."""

import math
import os
import random
import signal
import subprocess
import time

import pytest
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, QueryPlannerInterfaces
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from batchplan_viser.messages import box_object, pose_goal

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BESIDE_WALL = [-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785]
BEYOND_WALL = [1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785]


@pytest.fixture(scope="module")
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    demo = subprocess.Popen(
        ["ros2", "launch", "batchplan_moveit", "panda_moveit_demo.launch.py", "rviz:=false"],
        stdout=open(os.environ.get("DEMO_LOG", os.devnull), "w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("moveit_pipeline_test")
    joints = {}
    node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
    planning = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    assert planning.wait_for_service(timeout_sec=90.0), "move_group did not start"
    # Discovery finishes after the service first shows up; a request sent at once can lose its answer.
    settled = time.time() + 1.0
    while time.time() < settled:
        rclpy.spin_once(node, timeout_sec=0.1)
    yield node, planning, joints
    rclpy.shutdown()
    os.killpg(demo.pid, signal.SIGINT)
    demo.wait(timeout=30)


def call(node, client, request):
    answer = client.call_async(request)
    rclpy.spin_until_future_complete(node, answer, timeout_sec=30)
    return answer.result()


def joint_goal(values):
    return Constraints(
        joint_constraints=[
            JointConstraint(joint_name=n, position=v, weight=1.0, tolerance_above=1e-3, tolerance_below=1e-3)
            for n, v in zip(ARM, values)
        ]
    )


def plan_request(goal, start=BESIDE_WALL):
    request = GetMotionPlan.Request()
    m = request.motion_plan_request
    m.group_name, m.pipeline_id, m.allowed_planning_time = "panda_arm", "batchplan", 5.0
    m.start_state.joint_state.name, m.start_state.joint_state.position = ARM, start
    m.goal_constraints = [goal]
    return request


def test_batchplan_is_a_planning_pipeline(ros):
    node = ros[0]
    query = node.create_client(QueryPlannerInterfaces, "/query_planner_interface")
    assert query.wait_for_service(timeout_sec=10.0)
    interfaces = call(node, query, QueryPlannerInterfaces.Request()).planner_interfaces
    assert ("batchplan", "batchplan") in [(i.pipeline_id, i.name) for i in interfaces]


def test_plans_go_around_the_scenes_objects(ros):
    node, planning, _ = ros
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    assert apply.wait_for_service(timeout_sec=10.0)
    wall = PlanningScene(is_diff=True)
    wall.world.collision_objects = [box_object("wall", "world", (0.55, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0), (0.1, 0.3, 0.6))]
    assert call(node, apply, ApplyPlanningScene.Request(scene=wall)).success
    response = call(node, planning, plan_request(joint_goal(BEYOND_WALL))).motion_plan_response
    assert response.error_code.val == MoveItErrorCodes.SUCCESS, response.error_code.message
    points = response.trajectory.joint_trajectory.points
    straight = math.dist(BESIDE_WALL, BEYOND_WALL)
    length = sum(math.dist(a.positions, b.positions) for a, b in zip(points, points[1:]))
    # The straight line goes through the wall; MoveIt checked the plan, which goes round it.
    assert length > straight + 0.05, (length, straight)
    clear = PlanningScene(is_diff=True)
    clear.world.collision_objects = [box_object("wall", "world", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.1, 0.1, 0.1))]
    clear.world.collision_objects[0].operation = clear.world.collision_objects[0].REMOVE
    assert call(node, apply, ApplyPlanningScene.Request(scene=clear)).success


def test_move_group_executes_batchplans_pose_goals(ros):
    node, _, joints = ros
    move = ActionClient(node, MoveGroup, "/move_action")
    assert move.wait_for_server(timeout_sec=10.0)
    goal = MoveGroup.Goal()
    goal.request.group_name, goal.request.pipeline_id, goal.request.allowed_planning_time = "panda_arm", "batchplan", 5.0
    goal.request.goal_constraints = [pose_goal("panda_link8", "world", (0.4, 0.25, 0.4), (0.0, 1.0, 0.0, 0.0))]
    sent = move.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=30)
    result = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=60)
    outcome = result.result().result
    assert outcome.error_code.val == MoveItErrorCodes.SUCCESS, outcome.error_code.message
    last = outcome.planned_trajectory.joint_trajectory
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    final = dict(zip(last.joint_names, last.points[-1].positions))
    assert max(abs(joints[n] - v) for n, v in final.items()) < 1e-2


def test_path_constraints_are_refused_with_a_reason(ros):
    node, planning, _ = ros
    request = plan_request(joint_goal(BEYOND_WALL))
    request.motion_plan_request.path_constraints = joint_goal(BESIDE_WALL)
    response = call(node, planning, request).motion_plan_response
    assert response.error_code.val != MoveItErrorCodes.SUCCESS
    assert "path and trajectory constraints are not supported" in response.error_code.message
