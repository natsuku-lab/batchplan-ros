# Configuring the planner node

How `batchplan_planner` divides a workcell URDF between robot and world collision geometry, which search settings it exposes, and how it executes and cancels trajectories.

- [Workcells](#workcells)
- [Collision visualization](#collision-visualization)
- [Search settings](#search-settings)
- [Execution and cancellation](#execution-and-cancellation)

## Workcells

A workcell URDF can hold both a moving robot and fixed equipment such as tables and fixtures. By default the whole URDF is the robot: every link shares the collision-sphere fit, and large fixed objects take spheres from the arm. Set `robot_link_roots` to keep the fixed equipment out of the fit and plan around it as world geometry:

```bash
ros2 launch batchplan_ros planner.launch.py 'robot_link_roots:=[ur/base_link]'
```

Only the **strict descendants** of these links are the robot; the named roots themselves are not, unless they descend from another root. Several roots select the union of their descendants. Every other link must be fixed to the URDF's root, and each IK frame must be a robot link; otherwise the node refuses the description at startup.

All links, joints and frames are kept. Only collision ownership changes:

- **Robot links** are checked as fitted spheres, and each trajectory once more against their meshes.
- **Other links** become world obstacles in their link's frame: boxes, spheres, cylinders, and `<capsule radius="..." length="..."/>`, where `length` is the distance between the caps' centers. A mesh becomes a distance grid, so it must be a closed surface.

These obstacles are part of every request, alongside the request's planning scene, including IK and Cartesian-path requests that carry no scene. A scene object cannot take a world link's name. A robot link may touch the world link it is mounted on and any world link fixed to the same body; SRDF `disable_collisions` pairs that name a world link apply too, and a request's allowed collision matrix overrides both.

| Parameter | Default | Meaning |
|---|---|---|
| `robot_link_roots` | `[]` | Links whose strict descendants are the robot; empty for the whole URDF. |
| `sphere_budget` | `64` | Collision spheres for the robot links. Large robots need more: coarse spheres reach past the geometry and can refuse goals MoveIt accepts. Fits are cached per budget. |
| `world_sdf_voxel_size` | `0.01` | Grid spacing of a world mesh's distance grid, in meters. |

batchplan decides which values, shapes and meshes it can plan with, and the node reports its refusal at startup.

## Collision visualization

`~/collision_spheres` publishes the collision model as a `visualization_msgs/msg/MarkerArray` with transient-local QoS: add an RViz MarkerArray display with matching durability. Robot spheres are cyan and world obstacles orange, each in its link's frame and frame-locked, so they follow TF. A world mesh is drawn as its triangles. The markers show the URDF's model, not the objects of individual planning scenes.

## Search settings

The node exposes batchplan's IK, planning, RRT, shortcut, Cartesian and join options, and a few of its own:

| Prefix | Settings |
|---|---|
| `ik.*` | `seeds`, `iterations`, `damping`, `rotation_weight`, `max_step`, `collision_step`, `position_tolerance`, `rotation_tolerance`, `rng_seed` |
| `ik.collision.*`, `planning.collision.*` | `world`, `self_collision`, `margin`, `self_margin` |
| `planning.*` | `seeds`, `control_points`, `samples_per_span`, `iterations`, `history`, `initial_step`, `acceleration_weight`, `velocity_weight`, `validate_substeps`, `rng_seed` |
| `planning.rrt.*` | `range`, `resolution`, `max_iterations`, `rng_seed`, `clearance`, `clearance_ramp` |
| `planning.shortcut.*` | `resolution`, `passes`, `min_change`, `straighten`, `clearance`, `clearance_ramp` |
| `cartesian.*` | `step`, `rotation_step`, `precision`, `rotation_precision`, `resolution`, `iterations` |
| `join.*` | `resolution` |
| the node's own | `solutions_per_pose` (4 IK solutions planned to per pose goal), `solution_distance` (0.05, how far apart in joint space they must be), `default_planning_time` (5 s, for requests that allow none), `trajectory_dt` (0.02 s between waypoints) |

Defaults are batchplan's; see its [planning guide](https://github.com/natsuku-lab/batchplan/blob/main/docs/planning.md) for what each option does. A pose goal's tolerances can tighten `ik.position_tolerance` and `ik.rotation_tolerance`, and a Cartesian request's positive `max_step` replaces `cartesian.step`.

The node checks its own four settings and asks batchplan about the rest, at startup and at every update, so a value batchplan refuses is refused with batchplan's message.

### At startup

Each setting is a read-only ROS parameter of the same name (`ros2 param get /batchplan_planner ik.seeds`). Set them in a parameter file:

```yaml
# search.yaml
/**:
  ros__parameters:
    ik.seeds: 128
    solutions_per_pose: 8
    planning.rrt.max_iterations: 20000
```

```bash
ros2 launch batchplan_ros planner.launch.py parameters_file:=/absolute/path/search.yaml
```

### While running

`~/get_planner_params` and `~/set_planner_params` (`moveit_msgs/srv/GetPlannerParams`, `SetPlannerParams`) read and change the settings of a group and planner (`batchplan`, or `LIN` for straight-line moves; an empty planner name means `batchplan`). An update without a group applies to every group that has no settings of its own. An update is applied whole or, if any value is refused, not at all; the service has no status, so read the settings back, or see the node's log. A request plans with the settings its group has when it starts. Settings last until the node stops.

Through MoveIt the same settings are move_group's `/get_planner_params` and `/set_planner_params`, and **Planner Parameters** in RViz once the `batchplan` planner is selected under **Context → Planning Library**. The plugin sends each change to the node as it is made and shows what the node then holds. An update without a group reaches every group. If the node is not running when move_group loads the plugin (it waits 60 s), MoveIt shows no parameters and plans use the node's own settings.

### Planning time and attempts

A request's `allowed_planning_time` is shared by its `num_planning_attempts`. An attempt that finds no IK solution or no path is followed by another with different IK, optimization and RRT seeds, until one succeeds or the time is up. A `LIN` request follows one straight line, so it is attempted once.

## Execution and cancellation

A goal that is not plan-only is executed through the `controller_action` `FollowJointTrajectory` server. The node bounds every wait on the controller:

| Parameter | Default | The controller's time |
|---|---|---|
| `controller_response_timeout` | `5.0` s | For its action server to appear, to answer a goal request, to answer a cancel request, and to report the end of a cancelled goal. |
| `controller_result_margin` | `30.0` s | For a trajectory's result, past the trajectory's own duration. |

Cancelling `~/move_group` or `~/sequence_move_group`:

- **While planning:** the plan is dropped and no trajectory is sent.
- **While executing:** the node cancels the controller's goal, including one the controller has not yet accepted, and waits for the controller to report its end. The result is `PREEMPTED` once it has.
- **A stop that cannot be confirmed** is `CONTROL_FAILED`, never `PREEMPTED`: the controller refused or did not answer the cancel, or did not report the goal's end in time.

Without a cancel, a controller that does not appear, does not answer the goal request, or gives no result in time fails the goal with `CONTROL_FAILED`. The node then cancels the controller's goal, also when the controller accepts it only after the node has given up, and the message says whether the controller confirmed that it stopped.

Through MoveIt, a planning context cancels only its own request.
