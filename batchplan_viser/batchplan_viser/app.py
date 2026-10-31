"""Plan with batchplan from a browser.

Drag the gizmo to set the goal: IK runs as you drag, and the blue ghost shows where the arm would
be. Add boxes and drag them around, then Plan to preview the motion in green, or Plan and execute to
run it on the robot. Best grasp sends one request with the poses before many grasps of the can, which
the planner plans in one batch, keeping the shortest path; then the arm moves in to that grasp and the
fingers close on the can. Every request carries the boxes and the can as its scene, so the planner
keeps no state.
"""

import io
import os
import threading
import time

import numpy as np
import rclpy
import viser
import yourdfpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from viser.extras import ViserUrdf

from batchplan_viser.messages import (
    APPROACH,
    FINGER,
    FINGERS_OPEN,
    HAND,
    allowing,
    box_object,
    cylinder_object,
    grasp_poses,
    pose_goal,
    pose_msg,
    seconds,
)

GHOST = (0.25, 0.55, 1.0, 0.45)
PREVIEW = (0.3, 0.85, 0.4, 0.55)
BOX_SIZE = (0.12, 0.12, 0.3)
CAN_RADIUS, CAN_HEIGHT = 0.03, 0.2


def resolve(fname: str) -> str:
    """A mesh path (package://, file:// or plain) as a file. yourdfpy passes it as `fname`."""
    if fname.startswith("package://"):
        package, _, rest = fname[len("package://") :].partition("/")
        return os.path.join(get_package_share_directory(package), rest)
    return fname.removeprefix("file://")


def load_urdf(xml: str) -> yourdfpy.URDF:
    return yourdfpy.URDF.load(io.StringIO(xml), filename_handler=resolve)


class App(Node):
    def __init__(self):
        super().__init__("batchplan_viser")
        self.group = self.declare_parameter("group", "").value
        self.tip = self.declare_parameter("tip_link", "").value
        planner = self.declare_parameter("planner", "/batchplan_planner").value
        gripper = self.declare_parameter("gripper", "/panda_hand_controller").value
        self.port = self.declare_parameter("port", 8080).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.description = None
        self.described = threading.Event()
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.planning = ActionClient(self, MoveGroup, f"{planner}/move_group")
        self.ik = self.create_client(GetPositionIK, f"{planner}/compute_ik")
        self.hand = ActionClient(self, FollowJointTrajectory, f"{gripper}/follow_joint_trajectory")

        self.lock = threading.Lock()
        self.ik_busy = self.ik_again = False
        self.boxes = []
        self.can = None
        self.grasps = []
        self.playback = 0

    def on_description(self, msg: String):
        if self.description is None:
            self.description = msg.data
            self.described.set()

    def on_state(self, msg: JointState):
        self.joints.update(zip(msg.name, msg.position))

    def configuration(self, robot: ViserUrdf, joints: dict[str, float]) -> np.ndarray:
        return np.array([joints.get(name, 0.0) for name in robot.get_actuated_joint_names()])

    def start(self):
        """Builds the scene once the robot description and joint states have arrived."""
        self.described.wait()
        while not self.joints:
            time.sleep(0.05)
        self.server = viser.ViserServer(port=self.port)
        camera = self.server.initial_camera
        camera.position, camera.look_at, camera.up = (1.3, 0.9, 0.9), (0.3, 0.0, 0.35), (0.0, 0.0, 1.0)
        self.kinematics = load_urdf(self.description)
        self.frame = self.kinematics.base_link
        # Each robot hangs under a frame of its own, so it can be hidden as a whole.
        frame = lambda name: self.server.scene.add_frame(name, show_axes=False)
        self.ghost_frame, self.preview_frame = frame("/ghost"), frame("/preview")
        self.preview_frame.visible = False
        self.robot = ViserUrdf(self.server, load_urdf(self.description), root_node_name="/robot")
        self.ghost = ViserUrdf(self.server, load_urdf(self.description), root_node_name="/ghost", mesh_color_override=GHOST)
        self.preview = ViserUrdf(self.server, load_urdf(self.description), root_node_name="/preview", mesh_color_override=PREVIEW)
        self.server.scene.add_grid("/grid", width=2.0, height=2.0, cell_size=0.1)

        position, wxyz = self.tip_pose(self.joints)
        self.goal = self.server.scene.add_transform_controls("/goal", scale=0.2, position=position, wxyz=wxyz)
        self.goal.on_update(lambda _: self.solve_ik())

        gui = self.server.gui
        with gui.add_folder("batchplan"):
            self.status = gui.add_markdown("Drag the gizmo to set the goal.")
            gui.add_button("Plan", icon=viser.Icon.ROUTE).on_click(lambda _: self.plan(execute=False))
            gui.add_button("Plan and execute", icon=viser.Icon.PLAYER_PLAY).on_click(lambda _: self.plan(execute=True))
            self.speed = gui.add_slider("Speed", min=0.1, max=1.0, step=0.05, initial_value=1.0)
            gui.add_button("Goal at the hand", icon=viser.Icon.TARGET).on_click(lambda _: self.goal_at_hand())
        with gui.add_folder("Scene"):
            gui.add_button("Add box", icon=viser.Icon.BOX).on_click(lambda _: self.add_box())
            gui.add_button("Remove boxes", icon=viser.Icon.TRASH).on_click(lambda _: self.remove_boxes())
        with gui.add_folder("Best of N grasps"):
            gui.add_markdown(
                "One request with the poses before N grasps of the can: IK for all of them, one batched plan, the shortest "
                "path wins. Then in to the grasp, and the fingers close."
            )
            self.grasp_count = gui.add_slider("Grasps", min=1, max=64, step=1, initial_value=16)
            gui.add_button("Best grasp", icon=viser.Icon.HAND_GRAB).on_click(lambda _: self.best_grasp(execute=False))
            gui.add_button("Best grasp and execute", icon=viser.Icon.PLAYER_PLAY).on_click(lambda _: self.best_grasp(execute=True))

        self.solve_ik()
        while rclpy.ok():
            self.robot.update_cfg(self.configuration(self.robot, self.joints))
            time.sleep(1 / 30)

    def tip_pose(self, joints: dict[str, float]):
        self.kinematics.update_cfg(self.configuration(self.robot, joints))
        transform = self.kinematics.get_transform(self.tip, self.frame)
        return transform[:3, 3], viser.transforms.SO3.from_matrix(transform[:3, :3]).wxyz

    def goal_at_hand(self):
        self.goal.position, self.goal.wxyz = self.tip_pose(self.joints)
        self.solve_ik()

    def solve_ik(self):
        with self.lock:
            if self.ik_busy:
                self.ik_again = True
                return
            self.ik_busy = True
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group
        request.ik_request.ik_link_name = self.tip
        request.ik_request.pose_stamped.header.frame_id = self.frame
        request.ik_request.pose_stamped.pose = pose_msg(self.goal.position, self.goal.wxyz)
        self.ik.call_async(request).add_done_callback(self.on_ik)

    def on_ik(self, future):
        response = future.result()
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            self.ghost.update_cfg(self.configuration(self.ghost, {**self.joints, **solution}))
        self.ghost_frame.visible = response.error_code.val == MoveItErrorCodes.SUCCESS
        with self.lock:
            self.ik_busy, again, self.ik_again = False, self.ik_again, False
        if again:
            self.solve_ik()

    def add_box(self):
        name = f"box{len(self.boxes)}"
        position = (0.5, -0.3 + 0.2 * len(self.boxes), 0.15)
        control = self.server.scene.add_transform_controls(f"/boxes/{name}", scale=0.15, position=position)
        shape = self.server.scene.add_box(f"/boxes/{name}/shape", color=(230, 120, 50), dimensions=BOX_SIZE, opacity=0.85)
        self.boxes.append((name, control, shape))

    def remove_boxes(self):
        for _, control, shape in self.boxes:
            shape.remove()
            control.remove()
        self.boxes = []

    def add_can(self):
        control = self.server.scene.add_transform_controls("/can", scale=0.15, position=(0.45, 0.25, CAN_HEIGHT / 2), disable_rotations=True)
        self.server.scene.add_cylinder("/can/shape", radius=CAN_RADIUS, height=CAN_HEIGHT, color=(200, 60, 60))
        self.can = control

    def best_grasp(self, execute: bool):
        if self.can is None:
            self.add_can()
        threading.Thread(target=self.grasp, args=(execute,), daemon=True).start()

    def grasp(self, execute: bool):
        """Plans to the best of N poses before a grasp of the can in one request, then moves in to
        that grasp in a straight line, and closes the fingers on the can. On the robot, it then lets
        go and backs out the same way."""
        count = int(self.grasp_count.value)
        grasps = grasp_poses(self.can.position, count)
        approaches = grasp_poses(self.can.position, count, back=APPROACH)
        for frame in self.grasps:
            frame.remove()
        self.grasps = [
            self.server.scene.add_frame(f"/grasps/{k}", axes_length=0.04, axes_radius=0.003, position=p, wxyz=q)
            for k, (p, q) in enumerate(approaches)
        ]
        opened = {FINGER: FINGERS_OPEN}
        if execute and not self.grip(FINGERS_OPEN):
            return
        approach = self.move([pose_goal(self.tip, self.frame, p, q) for p, q in approaches], execute, self.objects(), opened)
        if approach is None:
            return
        waypoints, times = self.waypoints(approach, opened)
        reached = self.tip_pose(waypoints[-1])[0]
        k = min(range(count), key=lambda i: np.linalg.norm(approaches[i][0] - reached))
        # In a straight line, with the can in the scene and the hand allowed to touch it.
        start = opened if execute else waypoints[-1]
        touching = allowing("can", HAND)
        inward = self.move([pose_goal(self.tip, self.frame, *grasps[k])], execute, self.objects(), start, straight=True, allowed=touching)
        if inward is None:
            return
        if execute:
            # Close on the can, then let go and back out, so the next request does not start around it.
            if not self.grip(CAN_RADIUS):
                return
            time.sleep(1.0)
            if not self.grip(FINGERS_OPEN):
                return
            if self.move([pose_goal(self.tip, self.frame, *approaches[k])], True, self.objects(), opened, straight=True, allowed=touching) is None:
                return
        more, later = self.waypoints(inward, opened)
        waypoints += more
        times += [times[-1] + t for t in later]
        # The fingers close on the can in half a second.
        for step in range(1, 11):
            waypoints.append({**waypoints[-1], FINGER: FINGERS_OPEN + (CAN_RADIUS - FINGERS_OPEN) * step / 10})
            times.append(times[-1] + 0.05)
        self.status.content = (
            f"Best of {count} grasps in **{approach.planning_time * 1e3:.1f} ms**, in to the can in "
            f"**{inward.planning_time * 1e3:.1f} ms**, {times[-1]:.2f} s of motion with the fingers closing"
            + (", executed, then let go and backed out." if execute else ".")
        )
        self.show(waypoints, times, execute)

    def grip(self, opening: float) -> bool:
        """Moves the fingers to `opening` (each finger's distance from the middle) in half a second."""
        if not self.hand.wait_for_server(timeout_sec=2.0):
            self.status.content = "No gripper controller to close the fingers."
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [FINGER]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[opening], time_from_start=Duration(nanosec=500_000_000))]
        handle = wait(self.hand.send_goal_async(goal))
        return handle.accepted and wait(handle.get_result_async()).result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def objects(self):
        objects = [box_object(n, self.frame, c.position, c.wxyz, BOX_SIZE) for n, c, _ in self.boxes]
        if self.can is not None:
            objects.append(cylinder_object("can", self.frame, self.can.position, CAN_RADIUS, CAN_HEIGHT))
        return objects

    def plan(self, execute: bool):
        goals = [pose_goal(self.tip, self.frame, self.goal.position, self.goal.wxyz)]
        threading.Thread(target=self.plan_to, args=(goals, execute), daemon=True).start()

    def plan_to(self, goals, execute: bool):
        result = self.move(goals, execute, self.objects())
        if result is None:
            return
        waypoints, times = self.waypoints(result)
        self.status.content = (
            f"Planned in **{result.planning_time * 1e3:.1f} ms**, {len(waypoints)} waypoints, {times[-1]:.2f} s of motion"
            + (", executed." if execute else ".")
        )
        self.show(waypoints, times, execute)

    def move(self, goals, execute: bool, objects, start=None, straight=False, allowed=None):
        """Sends one MoveGroup request, from `start` (joint positions over the current ones), and waits
        for it; None, with the reason shown, if it fails. A `straight` request moves the hand in a
        straight line to its one pose goal; `allowed` is the scene's allowed collision matrix."""
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        if straight:
            request.planner_id = "LIN"
        request.allowed_planning_time = 2.0
        speed = float(self.speed.value)
        request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor = speed, speed * speed
        request.goal_constraints = goals
        if start:
            request.start_state.joint_state = JointState(name=list(start), position=[float(v) for v in start.values()])
        goal.planning_options.plan_only = not execute
        scene = goal.planning_options.planning_scene_diff
        scene.is_diff = True
        scene.world.collision_objects = objects
        if allowed is not None:
            scene.allowed_collision_matrix = allowed
        self.status.content = "Planning..."
        handle = wait(self.planning.send_goal_async(goal))
        if not handle.accepted:
            self.status.content = "The planner refused the request."
            return None
        result = wait(handle.get_result_async()).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.status.content = f"**Failed** ({result.error_code.val}): {result.error_code.message}"
            return None
        return result

    def waypoints(self, result, joints=None):
        """The planned trajectory's waypoints, with every joint (the current ones, then `joints`), and
        their times."""
        trajectory = result.planned_trajectory.joint_trajectory
        current = {**self.joints, **(joints or {})}
        waypoints = [{**current, **dict(zip(trajectory.joint_names, p.positions))} for p in trajectory.points]
        return waypoints, [seconds(p.time_from_start) for p in trajectory.points]

    def show(self, waypoints, times, execute: bool):
        """Draws the hand's path, and plays the motion on the preview unless the robot ran it."""
        path = np.array([self.tip_pose(w)[0] for w in waypoints])
        self.server.scene.add_line_segments(
            "/path", points=np.stack([path[:-1], path[1:]], axis=1), colors=(60, 200, 90), thickness=0.004
        )
        if not execute:
            self.play(waypoints, times)

    def play(self, waypoints, times):
        with self.lock:
            self.playback += 1
            mine = self.playback
        self.preview_frame.visible = True
        begun = time.perf_counter()
        for waypoint, at in zip(waypoints, times):
            if self.playback != mine:
                return
            time.sleep(max(0.0, at - (time.perf_counter() - begun)))
            self.preview.update_cfg(self.configuration(self.preview, waypoint))


def wait(future):
    """A future's result, waited for on a thread the executor does not run."""
    while not future.done():
        time.sleep(0.005)
    return future.result()


def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    executor.add_node(app)
    threading.Thread(target=app.start, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
