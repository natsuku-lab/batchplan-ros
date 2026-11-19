"""The planner node against a trajectory controller that misbehaves: one that shows up late, answers
late, never finishes or refuses to stop. The node must bound every wait, cancel whatever the
controller may still run, and report a stop only once the controller confirmed it.

Run with `pixi run test-demo`; it starts the node alone, on a domain of its own, with a controller
this file plays.
"""

import os
import random
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import pytest
import rclpy
import xacro
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

JOINTS = [f"panda_joint{i}" for i in range(1, 8)] + ["panda_finger_joint1", "panda_finger_joint2"]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02]
CONTROLLER = "/played_controller/follow_joint_trajectory"
# The node's waits, in seconds: for each answer of the controller, and for a result past the
# trajectory's own duration. Long enough for a loaded CI runner, short enough to test.
RESPONSE, MARGIN = 2.0, 1.0


class Controller:
    """A FollowJointTrajectory server that behaves as `play` says, and records what it saw."""

    def __init__(self, node):
        self.node, self.server, self.events = node, None, []
        self.play()

    def play(self, accept_after=0.0, finish=True, cancel=CancelResponse.ACCEPT, cancel_after=0.0):
        """How it treats the goals to come: how long it takes to accept one, whether it finishes
        one by itself, and what it answers to a cancel request, after how long. Goals still
        running from an earlier test end."""
        self.generation = getattr(self, "generation", 0) + 1
        self.accept_after, self.finish, self.cancel, self.cancel_after = accept_after, finish, cancel, cancel_after
        self.events.clear()

    def start(self):
        if self.server is None:
            self.server = ActionServer(
                self.node,
                FollowJointTrajectory,
                CONTROLLER,
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=self._cancel,
                callback_group=ReentrantCallbackGroup(),
            )

    def saw(self, event, within):
        deadline = time.time() + within
        while event not in self.events and time.time() < deadline:
            time.sleep(0.01)
        return event in self.events

    def _goal(self, _request):
        self.events.append("requested")
        time.sleep(self.accept_after)
        self.events.append("accepted")
        return GoalResponse.ACCEPT

    def _cancel(self, _goal):
        self.events.append("cancel requested")
        time.sleep(self.cancel_after)
        return self.cancel

    def _execute(self, goal):
        generation = self.generation
        while not self.finish and generation == self.generation:
            if goal.is_cancel_requested:
                self.events.append("cancelled")
                goal.canceled()
                return FollowJointTrajectory.Result()
            time.sleep(0.01)
        goal.succeed()
        return FollowJointTrajectory.Result()


@pytest.fixture(scope="module")
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    share = Path(get_package_share_directory("batchplan_examples"))
    parameters = {
        "robot_description": xacro.process_file(str(share / "urdf/panda.urdf.xacro")).toxml(),
        "robot_description_semantic": (share / "config/panda.srdf").read_text(),
        "controller_action": CONTROLLER,
        "controller_response_timeout": RESPONSE,
        "controller_result_margin": MARGIN,
    }
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as file:
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}, file)
    planner_node = subprocess.Popen(
        ["ros2", "run", "batchplan_ros", "batchplan_planner", "--ros-args", "--params-file", file.name],
        stdout=open(os.environ.get("DEMO_LOG", os.devnull), "w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("controller_faults_test")
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    planner = ActionClient(node, MoveGroup, "/batchplan_planner/move_group")
    assert planner.wait_for_server(timeout_sec=60.0), "the planner did not start"
    time.sleep(1.0)  # Discovery finishes after the server first shows up.
    controller = Controller(node)
    yield planner, controller
    controller.play()  # Ends the goals it still runs, so that the executor can stop.
    time.sleep(0.5)
    executor.shutdown(timeout_sec=1.0)
    rclpy.shutdown()
    os.killpg(planner_node.pid, signal.SIGINT)
    planner_node.wait(timeout=30)
    os.unlink(file.name)


def wait(future, seconds):
    deadline = time.time() + seconds
    while not future.done() and time.time() < deadline:
        time.sleep(0.01)
    assert future.done(), f"no answer within {seconds} s"
    return future.result()


def send(planner):
    """A short move of the arm, planned and executed: the goal's handle."""
    goal = MoveGroup.Goal()
    goal.request.group_name = "panda_arm"
    goal.request.start_state.joint_state.name = JOINTS
    goal.request.start_state.joint_state.position = READY
    target = [READY[0] + 0.3] + READY[1:7]
    goal.request.goal_constraints = [
        Constraints(joint_constraints=[JointConstraint(joint_name=n, position=v, weight=1.0) for n, v in zip(JOINTS, target)])
    ]
    goal.planning_options.planning_scene_diff.is_diff = True
    handle = wait(planner.send_goal_async(goal), 30)
    assert handle.accepted
    return handle


def result(handle, within):
    """How the planner's goal ended: its action status and its result."""
    wrapped = wait(handle.get_result_async(), within)
    return wrapped.status, wrapped.result


def test_a_controller_that_shows_up_late_still_runs_the_trajectory(ros):
    planner, controller = ros
    handle = send(planner)
    time.sleep(RESPONSE / 4)
    controller.start()
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.SUCCESS, answer.error_code.message
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert answer.executed_trajectory.joint_trajectory.points


def test_a_cancel_stops_the_controller_and_is_reported_once_it_stopped(ros):
    planner, controller = ros
    controller.start()
    controller.play(finish=False)
    handle = send(planner)
    assert controller.saw("accepted", 30)
    handle.cancel_goal_async()
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.PREEMPTED, answer.error_code.message
    assert status == GoalStatus.STATUS_CANCELED
    assert "cancelled" in controller.events


def test_a_cancel_before_the_controller_accepts_stops_the_goal_it_then_accepts(ros):
    planner, controller = ros
    controller.start()
    controller.play(accept_after=RESPONSE / 2, finish=False)
    handle = send(planner)
    assert controller.saw("requested", 30)
    handle.cancel_goal_async()
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.PREEMPTED, answer.error_code.message
    assert status == GoalStatus.STATUS_CANCELED
    assert "cancelled" in controller.events


def test_a_goal_the_controller_accepts_too_late_fails_and_is_stopped(ros):
    planner, controller = ros
    controller.start()
    controller.play(accept_after=2 * RESPONSE, finish=False)
    handle = send(planner)
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.CONTROL_FAILED, answer.error_code.message
    assert "did not answer the goal request" in answer.error_code.message
    assert status == GoalStatus.STATUS_ABORTED
    # The planner has answered before the controller accepts; the goal must not be left running.
    assert "accepted" not in controller.events
    assert controller.saw("cancelled", 4 * RESPONSE)


def test_a_controller_that_never_finishes_is_stopped_after_the_margin(ros):
    planner, controller = ros
    controller.start()
    controller.play(finish=False)
    handle = send(planner)
    status, answer = result(handle, 60)
    assert answer.error_code.val == MoveItErrorCodes.CONTROL_FAILED, answer.error_code.message
    assert "gave no result" in answer.error_code.message and "it was stopped" in answer.error_code.message
    assert status == GoalStatus.STATUS_ABORTED
    assert "cancelled" in controller.events


def test_a_stop_the_controller_refuses_is_not_reported_as_a_stop(ros):
    planner, controller = ros
    controller.start()
    controller.play(finish=False, cancel=CancelResponse.REJECT)
    handle = send(planner)
    assert controller.saw("accepted", 30)
    handle.cancel_goal_async()
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.CONTROL_FAILED, answer.error_code.message
    assert "is not cancelling the goal" in answer.error_code.message
    assert status == GoalStatus.STATUS_ABORTED


def test_a_cancel_while_a_quiet_controller_is_being_stopped_does_not_pass_for_a_stop(ros):
    planner, controller = ros
    controller.start()
    # The controller never finishes, and takes its time to refuse the stop that follows.
    controller.play(finish=False, cancel=CancelResponse.REJECT, cancel_after=RESPONSE / 2)
    handle = send(planner)
    assert controller.saw("cancel requested", 60)
    handle.cancel_goal_async()
    status, answer = result(handle, 30)
    assert answer.error_code.val == MoveItErrorCodes.CONTROL_FAILED, answer.error_code.message
    assert "cannot confirm that it stopped" in answer.error_code.message
    assert status == GoalStatus.STATUS_ABORTED
