"""Plan with batchplan from a browser.

Drag the gizmo to set the goal: IK runs as you drag, and the blue ghost shows where the arm would
be. Add boxes and drag them around, then Plan to preview the motion in green, or Plan and execute to
run it on the robot. Best grasp sends one request with many approach poses around the can, which the
planner plans in one batch, keeping the shortest path. Every request carries the boxes and the can as
its scene, so the planner keeps no state.
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
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from viser.extras import ViserUrdf

from batchplan_viser.messages import box_object, cylinder_object, grasp_poses, pose_goal, pose_msg, seconds

GHOST = (0.25, 0.55, 1.0, 0.45)
PREVIEW = (0.3, 0.85, 0.4, 0.55)
BOX_SIZE = (0.12, 0.12, 0.3)
CAN_RADIUS, CAN_HEIGHT = 0.04, 0.2


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
        self.port = self.declare_parameter("port", 8080).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.description = None
        self.described = threading.Event()
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.planning = ActionClient(self, MoveGroup, f"{planner}/move_group")
        self.ik = self.create_client(GetPositionIK, f"{planner}/compute_ik")

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
            gui.add_markdown("One request with N approach poses around the can: IK for all of them, one batched plan, the shortest path wins.")
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
        poses = grasp_poses(self.can.position, CAN_RADIUS, int(self.grasp_count.value))
        for frame in self.grasps:
            frame.remove()
        self.grasps = [
            self.server.scene.add_frame(f"/grasps/{k}", axes_length=0.04, axes_radius=0.003, position=p, wxyz=q)
            for k, (p, q) in enumerate(poses)
        ]
        goals = [pose_goal(self.tip, self.frame, p, q) for p, q in poses]
        self.plan(execute, goals, f"Best of {len(goals)} grasps")

    def objects(self):
        objects = [box_object(n, self.frame, c.position, c.wxyz, BOX_SIZE) for n, c, _ in self.boxes]
        if self.can is not None:
            objects.append(cylinder_object("can", self.frame, self.can.position, CAN_RADIUS, CAN_HEIGHT))
        return objects

    def plan(self, execute: bool, goals=None, label: str = "Planned"):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        request.allowed_planning_time = 2.0
        speed = float(self.speed.value)
        request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor = speed, speed * speed
        request.goal_constraints = goals or [pose_goal(self.tip, self.frame, self.goal.position, self.goal.wxyz)]
        goal.planning_options.plan_only = not execute
        scene = goal.planning_options.planning_scene_diff
        scene.is_diff = True
        scene.world.collision_objects = self.objects()
        self.status.content = "Planning..."
        sent = self.planning.send_goal_async(goal)
        sent.add_done_callback(lambda f: self.on_accepted(f, execute, label))

    def on_accepted(self, future, execute: bool, label: str):
        handle = future.result()
        if not handle.accepted:
            self.status.content = "The planner refused the request."
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f.result().result, execute, label))

    def on_result(self, result, execute: bool, label: str):
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.status.content = f"**Failed** ({result.error_code.val}): {result.error_code.message}"
            return
        trajectory = result.planned_trajectory.joint_trajectory
        points = trajectory.points
        motion = seconds(points[-1].time_from_start)
        self.status.content = (
            f"{label} in **{result.planning_time * 1e3:.1f} ms**, {len(points)} waypoints, {motion:.2f} s of motion"
            + (", executed." if execute else ".")
        )
        waypoints = [{**self.joints, **dict(zip(trajectory.joint_names, p.positions))} for p in points]
        path = np.array([self.tip_pose(w)[0] for w in waypoints])
        self.server.scene.add_line_segments(
            "/path", points=np.stack([path[:-1], path[1:]], axis=1), colors=(60, 200, 90), thickness=0.004
        )
        if not execute:
            threading.Thread(target=self.play, args=(waypoints, [seconds(p.time_from_start) for p in points]), daemon=True).start()

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
