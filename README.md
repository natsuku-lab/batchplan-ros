# batchplan-ros

ROS 2 motion planning with [batchplan](https://github.com/natsuku-lab/batchplan): batched IK and collision-aware planning on any GPU (Vulkan, Metal, DX12) or plain CPU threads, with no CUDA.

`batchplan_planner` is a ROS 2 node that speaks MoveIt's messages without needing MoveIt:

- `~/move_group` (`moveit_msgs/action/MoveGroup`): plans a `MotionPlanRequest` against the scene in `planning_options.planning_scene_diff`, and executes through a `FollowJointTrajectory` controller unless the goal is plan-only.
- `~/sequence_move_group` (`moveit_msgs/action/MoveGroupSequence`), as Pilz's capability of that name: plans a motion sequence's items one after another, each from where the one before ends, and returns one trajectory per run of items in one group. Each join with a `blend_radius` above 0 passes without stopping, the hand within that radius of where the motions meet, where batchplan's [`join`](https://github.com/natsuku-lab/batchplan/blob/main/docs/planning.md#joining-motions) finds that sooner and clear of collision and the items either side have the same scaling and speed cap; a sequence never takes longer than its motions one after another. The last radius is 0, and only the first item gives a start state.
- `~/compute_ik` (`moveit_msgs/srv/GetPositionIK`): it moves the request's group, and keeps the other joints where the request's state has them, as MoveIt does.
- `~/compute_cartesian_path` (`moveit_msgs/srv/GetCartesianPath`): the hand in straight lines through waypoints, as far as it reaches, with `fraction` saying how far. The service carries no scene (as with `compute_ik`), so it checks the robot and what its start state holds.

Several goal constraints in one request are planned in one batched call, and the shortest valid path wins. Planning requests also take:

- `planner_id: "LIN"`, as Pilz's planner names it: the hand moves in a straight line to its one pose goal, in the request's scene.
- The scene's `allowed_collision_matrix`: the robot's links may touch the objects it allows, as a hand touches what it grasps. Pairs of the robot's own links are the SRDF's.
- `max_cartesian_speed` on `cartesian_speed_limited_link` (an IK frame): the trajectory keeps that link under the speed, together with the scaling factors.

<p align="center">
  <img src="docs/best_grasp.gif" alt="A simulated Panda in the browser: the poses before 24 grasps of a can, planned in one motion sequence; the arm's preview moves past a box to the nearest one and on in to the grasp, and closes its fingers on the can." width="600">
</p>

## Quick start

```bash
git clone https://github.com/natsuku-lab/batchplan-ros
cd batchplan-ros
pixi run demo                                    # Linux or macOS (Apple silicon); ROS 2 Lyrical from RoboStack
pixi run -e jazzy demo                           # the same on ROS 2 Jazzy
```

Then open http://localhost:8080: a simulated Panda and its gripper on ros2_control mock hardware. Drag the gizmo to set a goal (IK runs as you drag), add boxes, and plan or plan and execute. No MoveIt and no GPU are needed; `pixi run demo device:=gpu` plans on the GPU.

**Best of N grasps.** "Best grasp" sends one motion sequence. Its first item's goal constraints are the poses 8 cm before N grasps of a can, each around a different side: the node solves IK for all of them and plans to every solution in one batched call, keeping the shortest path. The second moves the hand 8 cm in along its own z to the grasp in a straight line (`LIN`, its goal in `panda_link8`'s frame), and a 4 cm blend radius lets the arm pass into it without stopping where that is sooner. The can is in the scene and the hand may touch it. The fingers then close on the can through their own trajectory controller; on the robot, the hand then lets go and backs out the same way. `ros2 run batchplan_examples best_grasp.py` times the first item, as one `MoveGroup` request with every pose, against one request per grasp, next to a running demo. Wall-clock times on an M4 Pro CPU, on Lyrical (the middle of three runs, each the median of 5 requests; other apps running):

| Grasps | One request | One request per grasp |
|---|---|---|
| 1 | 12 ms | 10 ms |
| 8 | 8 ms | 62 ms |
| 64 | 18 ms | 620 ms |

## With MoveIt (optional)

```bash
pixi run -e moveit demo-moveit                   # -e jazzy-moveit on Jazzy
```

This is MoveIt's stock Panda demo, with a `batchplan` pipeline next to OMPL. In RViz's MotionPlanning panel, pick "batchplan" under Context → Planning Library, then Plan or Plan & Execute. The `batchplan_moveit` plugin forwards each request, with MoveIt's planning scene (its allowed collision matrix included), to the planner node, waits within the request's planning time, and hands MoveIt a timed trajectory; MoveIt's own checks and controllers take it from there. Everything without MoveIt keeps working without it: the plugin is its own package and pixi environment.

Through MoveIt's `/plan_kinematic_path`, 20 alternating identical requests per planner on an M4 Pro CPU, on Lyrical (other apps running), medians:

| Goal | Planner | Solved | Planning | Path | Motion |
|---|---|---|---|---|---|
| Round a wall | OMPL (RRTConnect) | 15/20 | 16.4 ms | 3.72 rad | 2.85 s |
| Round a wall | batchplan | 20/20 | 3.8 ms | 2.69 rad | 2.23 s |
| Free space | OMPL (RRTConnect) | 18/20 | 13.4 ms | 2.51 rad | 2.27 s |
| Free space | batchplan | 20/20 | 4.2 ms | 1.88 rad | 2.52 s |

batchplan keeps the configuration's jerk limits, which MoveIt's default time parameterization ignores, so its free-space motion is slower.

To use it with your robot's MoveIt configuration, add `batchplan_planning.yaml` (from `batchplan_moveit/config`) to your config package's planning pipelines, and run `batchplan_planner` with the robot's SRDF and joint limits as `panda_moveit_demo.launch.py` does.

## Your own robot

To plan for your own robot without MoveIt, run the node next to its `robot_state_publisher`:

```bash
pixi run ros2 launch batchplan_ros planner.launch.py robot_description_semantic:="$(cat my_robot.srdf)" \
    controller_action:=/arm_controller/follow_joint_trajectory
```

In an existing ROS 2 Lyrical or Jazzy workspace, with Rust 1.90 or later, `colcon-cargo`, `colcon-ros-cargo` and `cargo-ament-build` installed: `colcon build --packages-select batchplan_ros`.

## Status

Early: ROS 2 Lyrical Luth (the 2026 LTS) and Jazzy Jalisco (the 2024 LTS). CI builds and tests the node and the browser demo on both, on Linux (x86-64) and macOS (Apple silicon), and the MoveIt plugin on both, on Linux. rclrs 0.8.0 ends the node on Lyrical when a client leaves ([ros2-rust/ros2_rust#730](https://github.com/ros2-rust/ros2_rust/pull/730)), so the node builds rclrs from that fix's commit until a release includes it. The Rust node follows [Google's Rust style guide](https://google.github.io/styleguide/rustguide.html), as batchplan does; its [contributing guide](https://github.com/natsuku-lab/batchplan/blob/main/CONTRIBUTING.md) has the commands and the choices.
