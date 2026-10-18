#!/usr/bin/env python3
"""Best of N grasps: one request with N approach poses against N requests with one each.

Run next to the Panda demo (`pixi run demo`, or with ui:=false). For each N it times one MoveGroup
request carrying N pose goals, which batchplan plans in one batched call, against N requests of one
goal each, and reports the planner's time, the wall-clock time and the motion of the path kept.
"""

import statistics
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient

from batchplan_viser.messages import cylinder_object, grasp_poses, pose_goal, seconds

CAN = ((0.45, 0.25, 0.1), 0.04, 0.2)
REPEATS = 5


def request(goals):
    goal = MoveGroup.Goal()
    goal.request.group_name = "panda_arm"
    goal.request.goal_constraints = goals
    goal.planning_options.plan_only = True
    scene = goal.planning_options.planning_scene_diff
    scene.is_diff = True
    center, radius, height = CAN
    scene.world.collision_objects = [cylinder_object("can", "world", center, radius, height)]
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node("best_grasp")
    planner = ActionClient(node, MoveGroup, "/batchplan_planner/move_group")
    if not planner.wait_for_server(timeout_sec=30.0):
        sys.exit("no planner at /batchplan_planner/move_group: start the demo first")
    time.sleep(1.0)

    def run(goal):
        sent = planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, result)
        return result.result().result

    print(f"{'grasps':>6} | {'one request: planner':>21} | {'wall':>8} | {'motion':>7} | {'one per grasp: planner':>23} | {'wall':>8} | {'solved':>6}")
    for count in (1, 8, 64):
        center, radius, _ = CAN
        goals = [pose_goal("panda_link8", "world", p, q) for p, q in grasp_poses(center, radius, count)]
        batched, batched_wall, separate, separate_wall = [], [], [], []
        for _ in range(REPEATS):
            begun = time.perf_counter()
            result = run(request(goals))
            batched_wall.append(time.perf_counter() - begun)
            batched.append(result.planning_time)
            ok = result.error_code.val == MoveItErrorCodes.SUCCESS
            motion = f"{seconds(result.planned_trajectory.joint_trajectory.points[-1].time_from_start):6.2f}s" if ok else "-"
            begun, total, solved = time.perf_counter(), 0.0, 0
            for goal in goals:
                single = run(request([goal]))
                total += single.planning_time
                solved += single.error_code.val == MoveItErrorCodes.SUCCESS
            separate_wall.append(time.perf_counter() - begun)
            separate.append(total)
        ms = lambda xs: f"{statistics.median(xs) * 1e3:7.1f} ms"
        print(f"{count:>6} | {ms(batched):>21} | {ms(batched_wall):>8} | {motion:>7} | {ms(separate):>23} | {ms(separate_wall):>8} | {solved:>3}/{count:<2}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
