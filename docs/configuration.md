# Configuring the planner node

How `batchplan_planner` divides a workcell URDF between robot and world collision geometry, which search settings it exposes, and how it cancels requests.

- [Sphere budget](#sphere-budget)
- [Robot subtrees and URDF world geometry](#robot-subtrees-and-urdf-world-geometry)
- [Collision visualization](#collision-visualization)
- [Search settings](#search-settings)
- [Cancellation](#cancellation)

## Sphere budget

The positive integer `sphere_budget` parameter (also an argument to `planner.launch.py`, default `64`) controls the collision-sphere fit for the selected robot links. With no `robot_link_roots`, the whole URDF is fitted, including fixed tables and fixtures.

Large robot models need a larger budget: coarse spheres extend beyond the actual collision geometry and can reject goals that MoveIt accepts. Increasing the budget refines that approximation without disabling collisions or changing the trajectory's final mesh checks.

The budget is a hard maximum, not a target sphere count: the fitter may use coarser, more inflated spheres to meet it, and models that cannot fit it, including cached models, are rejected rather than exceeding it. Sphere fits are cached separately for each budget; restart the planner after changing it.

## Robot subtrees and URDF world geometry

A workcell URDF can hold both a moving robot and fixed equipment. Set `robot_link_roots` to keep the fixed equipment out of the robot's sphere fit and plan around it as world geometry:

```yaml
robot_link_roots: [ur/base_link]
world_sdf_voxel_size: 0.01
```

```bash
ros2 launch batchplan_ros planner.launch.py 'robot_link_roots:=[ur/base_link]'
```

Only the **strict descendants** of these links own robot collision geometry; the named roots themselves are not included unless they descend from another selected root. Multiple roots select the union of their descendants. An empty list retains the all-robot behavior. Unknown roots, empty descendant selections, and non-fixed joints outside the selected subtrees fail explicitly: URDF world links must be rigidly attached to the workcell root.

All kinematic links, joints and frame transforms are retained. Only collision ownership changes:

- **Robot links** use spheres and final mesh checks.
- **Other links** use native oriented boxes, spheres and cylinders. The world loader also accepts `<capsule radius="..." length="..."/>`, where length is the central segment's length. Each collision origin's translation and rotation are composed with its link's transform, including separate origins for multiple collision elements.
- **World meshes** use conservative SDFs directly, not sphere or primitive fitting.

URDF world obstacles are retained alongside the request's planning scene, including for IK and Cartesian services that carry no scene. SRDF and adjacent/rigid-link exemptions survive the robot/world split; explicit request ACM entries override them. Request objects cannot reuse URDF world-link ids to replace or remove them. Robot caches are keyed by the collision-filtered URDF, so an all-robot cache cannot be reused for a different collision partition.

### World mesh requirements

- Meshes must be watertight, consistently oriented, non-self-intersecting solids; nested shells and cavities are rejected. Disconnected shells, including shells touching only at a vertex, are split and oriented independently.
- Repeated Collada `instance_geometry` references and `instance_node` elements are rejected, because the importer does not preserve their instances. Bake those instances into unique mesh geometry first; single-instance Collada remains supported.
- Mesh files larger than 64 MiB are rejected before loading.
- `world_sdf_voxel_size` is the maximum grid spacing in meters. Grids above 67,108,864 points, or exceeding the SDF builder's numeric limits, are rejected before allocating grid data; resolution is never coarsened to make a grid fit. These are fixed resource limits, not estimates of available system memory.

## Collision visualization

`~/collision_spheres` publishes the whole URDF collision model as `visualization_msgs/msg/MarkerArray`, with reliable transient-local QoS. Add an RViz MarkerArray display with matching durability.

- Robot spheres are cyan; native world shapes are orange. World SDF markers show their source triangle surfaces, not a meshed SDF inflation halo.
- Markers use link-local coordinates and are frame-locked, so they follow the robot's TF. Equal-radius spheres are batched for rendering.
- The markers represent the URDF model (including fixed fixtures), not world objects from individual planning requests. They are visualization only, not duplicate MoveIt obstacles.
- Publication failures are logged but do not prevent planner initialization.

## Search settings

The node exposes every native `IkOptions`, `PlanOptions`, `RrtOptions`, `ShortcutOptions`, `ToolPathOptions` and `JoinOptions` search field. `PlanOptions.time_budget` is represented by the request's planning time, with `default_planning_time` (5 seconds) as fallback.

| ROS parameter prefix | Settings |
|---|---|
| `ik.*` | `seeds`, `iterations`, `damping`, `rotation_weight`, `max_step`, `collision_step`, `position_tolerance`, `rotation_tolerance`, `rng_seed` |
| `ik.collision.*` | `world`, `self_collision`, `margin`, `self_margin` |
| `planning.*` | `seeds`, `control_points`, `samples_per_span`, `iterations`, `history`, `initial_step`, `acceleration_weight`, `velocity_weight`, `validate_substeps`, `rng_seed` |
| `planning.collision.*` | `world`, `self_collision`, `margin`, `self_margin` |
| `planning.rrt.*` | `range`, `resolution`, `max_iterations`, `rng_seed`, `clearance`, `clearance_ramp` |
| `planning.shortcut.*` | `resolution`, `passes`, `min_change`, `straighten`, `clearance`, `clearance_ramp` |
| `cartesian.*` | `step`, `rotation_step`, `precision`, `rotation_precision`, `resolution`, `iterations` |
| `join.*` | `resolution` |

Other controls: `solutions_per_pose` (4), `solution_distance` (0.05 in joint space), `trajectory_dt` (0.02 seconds), `cpu_threads` (0 = all cores) and `device` (`cpu`/`gpu`).

- `ik.seeds` defaults to 32. `planning.seeds` defaults to 0 and enables additional trajectory-optimization candidates, not additional RRT attempts.
- Spatial collision resolutions are joint-space distances, not sampling times.
- Pose requests can tighten the configured IK tolerances. A Cartesian request's positive `max_step` overrides `cartesian.step`.
- Finer trajectory sampling is not a substitute for MoveIt's `ValidateSolution`.

### Startup defaults

Startup defaults are read-only ROS parameters. Inspect them with `ros2 param list /batchplan_planner` and `ros2 param get /batchplan_planner ik.seeds`. To set persistent defaults, pass a parameter file, which overrides the launch defaults:

```yaml
# search.yaml
/**:
  ros__parameters:
    trajectory_dt: 0.02
    ik.seeds: 64
    solutions_per_pose: 8
    planning.rrt.max_iterations: 20000
```

```bash
ros2 launch batchplan_ros planner.launch.py parameters_file:=/absolute/path/search.yaml
```

Device, CPU threads, sphere fitting and world geometry are startup settings only.

### Editing while running

Live planner configurations use `~/get_planner_params` and `~/set_planner_params` on the node (MoveIt's `/get_planner_params` and `/set_planner_params` through the plugin); an empty planner name selects `batchplan`. Unknown groups and planner names are rejected rather than using defaults. Valid groups inherit global settings when no group-specific override exists. Invalid edits fail planning explicitly and are logged by the node.

In RViz, select the group and the `batchplan` planner under **Context → Planning Library**: **Planner Parameters** exposes the 50 search and sampling settings for editing. Edits are uploaded to the node when the next MoveIt planning request starts, so they apply to subsequent plans for that group, not in-flight plans. They are session-only; save persistent defaults in YAML.

The node must be ready within 60 seconds when MoveIt loads the plugin, so the plugin can discover the configured defaults. Rebuild and restart both packages together when updating this interface.

### Planning time and attempts

MoveIt's `allowed_planning_time` and `num_planning_attempts` are supported, as are its velocity/acceleration scaling and Cartesian speed caps.

- Attempts share one overall time budget and stop at the first valid result. Each retry changes the IK, optimization and RRT seeds; zero attempts means one.
- `LIN` follows a fixed Cartesian line rather than retrying alternate paths.
- MoveIt's `ValidateSolution` response adapter performs final validation with the actual planning scene. The plugin does not repeat this scan inside the search time budget or rerun successful native searches on adapter rejection. Differences between collision models can still cause validation failures.

## Cancellation

Both standalone actions accept cancellation during planning and execution.

- **Planning:** canceled planning results are discarded without sending a controller goal. Native planning already in progress may finish in its worker thread.
- **Execution:** cancellation is forwarded to the specific controller goal, including a goal whose acceptance is still pending. The action remains canceling until the controller confirms a terminal result.
- **Unconfirmed stops:** a refused cancellation, or one the controller does not confirm in time, is reported as `CONTROL_FAILED`, not as a successful stop.
- **MoveIt:** contexts cancel only their own requests; canceling a queued context does not affect the active context or submit another goal.
