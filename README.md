# batchplan-ros

ROS 2 motion planning with [batchplan](https://github.com/natsuku-lab/batchplan): batched IK and collision-aware planning on any GPU (Vulkan, Metal, DX12) or plain CPU threads, with no CUDA.

`batchplan_planner` is a ROS 2 node that speaks MoveIt's messages without needing MoveIt:

- `~/move_group` (`moveit_msgs/action/MoveGroup`): plans a `MotionPlanRequest` against the scene in `planning_options.planning_scene_diff`, and executes through a `FollowJointTrajectory` controller unless the goal is plan-only.
- `~/compute_ik` (`moveit_msgs/srv/GetPositionIK`).

Several goal constraints in one request are planned in one batched call, and the shortest valid path wins.

## Quick start

```bash
pixi run demo                                    # Linux or macOS (Apple silicon); ROS 2 Jazzy from RoboStack
```

Then open http://localhost:8080: a simulated Panda on ros2_control mock hardware. Drag the gizmo to set a goal (IK runs as you drag), add boxes, and plan or plan and execute. No MoveIt and no GPU are needed; `pixi run demo device:=gpu` plans on the GPU.

**Best of N grasps.** "Best grasp" sends one request with N approach poses around a can as its goal constraints. The node solves IK for all of them and plans to every solution in one batched call, keeping the shortest path. `ros2 run batchplan_examples best_grasp.py` times that against one request per grasp, next to a running demo. On an M4 Pro CPU (medians of 5, other apps running):

| Grasps | One request | One request per grasp |
|---|---|---|
| 1 | 15 ms | 15 ms |
| 8 | 10 ms | 51 ms |
| 64 | 28 ms | 554 ms |

To plan for your own robot, run the node next to its `robot_state_publisher`:

```bash
pixi run ros2 launch batchplan_ros planner.launch.py robot_description_semantic:="$(cat my_robot.srdf)" \
    controller_action:=/arm_controller/follow_joint_trajectory
```

In an existing ROS 2 Jazzy workspace, with `colcon-cargo`, `colcon-ros-cargo` and `cargo-ament-build` installed: `colcon build --packages-select batchplan_ros`.

## Status

Early: Jazzy only. An optional MoveIt planner plugin is on the way.
