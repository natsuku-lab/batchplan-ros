<h1 align="center">batchplan-ros</h1>

<p align="center"><em>Batched motion planning for ROS 2.</em></p>

<p align="center">
  <img src="docs/best_grasp.gif" alt="A simulated Panda in the browser: the poses before 24 grasps of a can, planned in one motion sequence; the arm's preview moves past a box to the nearest one and on in to the grasp, and closes its fingers on the can." width="600">
</p>

**ROS 2 integration for [batchplan](https://github.com/natsuku-lab/batchplan).** A standalone planner node using MoveIt messages, plus an optional MoveIt planning plugin. Planning runs on CPU threads, with optional GPU acceleration for IK and trajectory optimization through Vulkan, Metal or DX12. No CUDA required.

- **Plan:** batched pose goals, multi-arm IK, collision-aware paths and timed trajectories.
- **Sequence:** straight-line moves, blended joins, attached objects and constrained motions.
- **Execute:** send trajectories to a `FollowJointTrajectory` controller, or return a plan without executing it.

See the [batchplan README](https://github.com/natsuku-lab/batchplan#readme) for the library overview, [planning guide](https://github.com/natsuku-lab/batchplan/blob/main/docs/planning.md) for algorithms and constraints, and [benchmarks](https://github.com/natsuku-lab/batchplan/blob/main/docs/benchmarks.md) for performance comparisons.

## Quick start

With [pixi](https://pixi.sh) installed:

```bash
git clone https://github.com/natsuku-lab/batchplan-ros
cd batchplan-ros
pixi run demo                    # ROS 2 Lyrical
pixi run -e jazzy demo           # ROS 2 Jazzy
```

Open http://localhost:8080 to control a simulated Panda on ros2_control mock hardware. Drag a goal, add obstacles, then plan or execute. **Best grasp** batches candidate grasps and selects the shortest valid path before approaching and closing the gripper.

The demo runs on Linux or macOS (Apple silicon), without MoveIt or a GPU. Use `pixi run demo device:=gpu` for GPU acceleration.

`pixi run demo-tiago-pro` runs TIAGo Pro with its base and arms planned together. **Carry the tray** keeps a tray level between both hands while moving around a wall. The demo fetches PAL Robotics' model from a pinned, checksum-verified source and retains its license notices.

## ROS interfaces

`batchplan_planner` provides:

| Endpoint | Type | Purpose |
|---|---|---|
| `~/move_group` | `moveit_msgs/action/MoveGroup` | Plan and optionally execute a motion. |
| `~/sequence_move_group` | `moveit_msgs/action/MoveGroupSequence` | Plan a sequence with optional blended joins. |
| `~/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Solve IK for the requested group. |
| `~/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | Follow straight-line waypoints and report the completed fraction. |

Planning requests supply the scene through `planning_options.planning_scene_diff`. Multiple unconstrained goal alternatives are batched; the shortest valid path wins. Set `planner_id: "LIN"` for a straight-line move to a pose goal.

The node supports allowed robot-object contacts, Cartesian speed limits, and position/orientation constraints on IK frames, including constraints relative to another hand. Joint and visibility path constraints, rotation-vector tolerances, and path constraints on `LIN` requests are unsupported. IK and Cartesian-path services carry no scene; collision checks use the robot and objects attached to their start state.

## With MoveIt

```bash
pixi run -e moveit demo-moveit   # use -e jazzy-moveit on Jazzy
```

In RViz, select **batchplan** under **Context → Planning Library**. The plugin forwards MoveIt's planning scene to the node and returns a timed trajectory; MoveIt handles execution.

For your own configuration, add [`batchplan_planning.yaml`](batchplan_moveit/config/batchplan_planning.yaml) to its planning pipelines and launch the node with your robot's SRDF and joint limits. See [`panda_moveit_demo.launch.py`](batchplan_moveit/launch/panda_moveit_demo.launch.py).


## Your own robot

To plan for your own robot without MoveIt, run the node next to its `robot_state_publisher`:

```bash
pixi run ros2 launch batchplan_ros planner.launch.py robot_description_semantic:="$(cat my_robot.srdf)" \
    controller_action:=/arm_controller/follow_joint_trajectory
```

In an existing ROS 2 Lyrical or Jazzy workspace, with Rust 1.90 or later, `colcon-cargo`, `colcon-ros-cargo` and `cargo-ament-build` installed: `colcon build --packages-select batchplan_ros`.

## Configuration

- **Workcells:** `robot_link_roots` keeps fixed tables and fixtures out of the robot's sphere fit and plans around them as world geometry.
- **Search settings:** IK, planning and sampling settings are ROS parameters at startup and MoveIt planner parameters while running.
- **Cancellation:** canceling an action stops planning or the controller goal, and reports `CONTROL_FAILED` when the stop cannot be confirmed.
- **Visualization:** `~/collision_spheres` publishes the collision model as RViz markers.

See [configuration](docs/configuration.md) for details.

## Development

Early-stage software. CI covers the node and browser demo on Linux and macOS, and the MoveIt plugin on Linux, for Lyrical and Jazzy.

```bash
pixi run test
pixi run test-demo
pixi run -e moveit test-moveit
```

See batchplan's [contributing guide](https://github.com/natsuku-lab/batchplan/blob/main/CONTRIBUTING.md) for conventions and its [limitations](https://github.com/natsuku-lab/batchplan#limitations) before using it on a robot.

## License

[Apache-2.0](LICENSE-APACHE) or [MIT](LICENSE-MIT), at your option. Robot models retain their own licenses.
