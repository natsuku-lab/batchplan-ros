"""TIAGo Pro carrying a tray level with both hands, from one table, around a wall, to another.

Each leg is one MoveGroup request planning the base and both arms (`GROUP`). The legs that carry
the tray keep it level and the right hand pinned where it holds the tray in the left hand's frame
(`carried`), as path constraints; the tray rides in the left hand as an attached object.
"""

import numpy as np
from moveit_msgs.msg import AttachedCollisionObject, Constraints, MotionPlanRequest
from sensor_msgs.msg import JointState
from viser.transforms import SE3, SO3

from batchplan_viser.messages import allowing, box_object, joint_goal, oriented, positioned

GROUP = "base_arms"
LEFT, RIGHT = "gripper_left_tray_link", "gripper_right_tray_link"
GRIPPERS = (("/gripper_left_controller", "gripper_left_finger_joint"), ("/gripper_right_controller", "gripper_right_finger_joint"))
OPEN, CLOSED = 0.035, 0.0
# The tray: 30 cm deep, 50 cm across, 2 cm thick, resting on 72 cm tables.
TRAY = np.array([0.3, 0.5, 0.02])
TABLE_TOP = 0.72
# Where each hand pinches the tray's rim, in the tray's frame: 8 cm in from its back corners.
GRIP = (np.array([-0.08, 0.24, 0.0]), np.array([-0.08, -0.24, 0.0]))
# How far before the grasp each hand starts, along the tray's -x, and how high the tray is lifted.
BACK, LIFT = 0.15, 0.08
# Level within 3°, and the right hand pinned in the left's frame (to the planner's floors of 2 mm
# and 0.02 rad relative to another hand).
LEVEL = np.radians(3.0)


def tray_on(x: float, y: float, yaw: float) -> SE3:
    """The tray resting on a table at (x, y), turned `yaw`."""
    return SE3.from_rotation_and_translation(SO3.from_z_radians(yaw), np.array([x, y, TABLE_TOP + TRAY[2] / 2 + 0.002]))


# The robot starts at the origin, arms straight out, 40 cm short of the first tray.
PICK, PLACE = tray_on(1.25, 0.0, 0.0), tray_on(-0.3, 3.05, np.pi / 2)
# Tables narrower than the tray, so its sides overhang where the fingers pinch them, and a wall a
# metre tall between them, open only past its end by the first.
TABLES = {"table_a": ((1.38, 0.0), (0.6, 0.28)), "table_b": ((-0.3, 3.18), (0.28, 0.6))}
WALL = ((-1.15, 1.3, 0.5), (2.9, 0.2, 1.0))


def lifted(tray: SE3, by: float) -> SE3:
    return SE3.from_translation(np.array([0.0, 0.0, by])) @ tray


def hands(tray: SE3, back: float = 0.0):
    """Each tray link's pose holding the tray at `tray`, moved `back` along the tray's -x."""
    return [tray @ SE3.from_translation(g - np.array([back, 0.0, 0.0])) for g in GRIP]


def goal(tray: SE3, frame: str, back: float = 0.0) -> Constraints:
    """Both hands holding the tray at `tray`, `back` before the grasp: one goal, one IK problem."""
    c = Constraints()
    for link, at in zip((LEFT, RIGHT), hands(tray, back)):
        c.position_constraints.append(positioned(link, frame, at.translation(), 1e-3))
        c.orientation_constraints.append(oriented(link, frame, at.rotation().wxyz, (1e-2, 1e-2, 1e-2)))
    return c


def carried(frame: str) -> Constraints:
    """The tray level, turning freely about the vertical, and the right hand pinned where it holds
    the tray in the left hand's frame."""
    place = SE3.from_translation(GRIP[0]).inverse() @ SE3.from_translation(GRIP[1])
    c = Constraints()
    c.orientation_constraints = [
        oriented(LEFT, frame, (1.0, 0.0, 0.0, 0.0), (LEVEL, LEVEL, np.pi)),
        oriented(RIGHT, LEFT, place.rotation().wxyz, (1e-3, 1e-3, 1e-3)),
    ]
    c.position_constraints = [positioned(RIGHT, LEFT, place.translation(), 1e-3, box=True)]
    return c


def scene(frame: str, tray_at: SE3 | None, without=()):
    """The tables (but those in `without`), the wall, and the tray where it rests, if it does."""
    objects = [box_object(name, frame, (*xy, TABLE_TOP / 2), (1.0, 0.0, 0.0, 0.0), (*size, TABLE_TOP)) for name, (xy, size) in TABLES.items() if name not in without]
    objects.append(box_object("wall", frame, WALL[0], (1.0, 0.0, 0.0, 0.0), WALL[1]))
    if tray_at is not None:
        objects.append(box_object("tray", frame, tray_at.translation(), tray_at.rotation().wxyz, TRAY))
    return objects


def held(touch_links) -> AttachedCollisionObject:
    """The tray in the left hand, held by its rim, the grippers' links free to touch it."""
    in_left = SE3.from_translation(GRIP[0]).inverse()
    attached = AttachedCollisionObject(link_name=LEFT, touch_links=list(touch_links))
    attached.object = box_object("tray", LEFT, in_left.translation(), in_left.rotation().wxyz, TRAY)
    return attached


def base_for(q_base, tray_from: SE3, tray_to: SE3):
    """The base pose (x, y, turn) that puts the tray at `tray_to` relative to it as a base at
    `q_base` has it at `tray_from`: the arms need not change."""
    x, y, theta = q_base
    base = SE3.from_rotation_and_translation(SO3.from_z_radians(theta), np.array([x, y, 0.0]))
    moved = tray_to @ tray_from.inverse() @ base
    turn = SO3.from_matrix(moved.rotation().as_matrix()).compute_yaw_radians()
    return float(moved.translation()[0]), float(moved.translation()[1]), float(turn)


def steps(frame: str):
    """The carry, step by step: each leg as (name, its goal, or a function of the joints where the
    leg before ends that gives it, its scene, whether it carries the tray, whether the grippers may
    touch the tray), and the grippers' moves as ("grip", opening). The tray leaves the scene while
    the hands hold it, and each table while the tray rests on it, lifted off or set down."""
    up, above = lifted(PICK, LIFT), lifted(PLACE, LIFT)

    def driven(joints: dict) -> Constraints:
        # The base to where the tray sits over the second table as it did over the first.
        base = base_for((joints["base_joint_x"], joints["base_joint_y"], joints["base_joint_theta"]), up, above)
        planned = {n: v for n, v in joints.items() if n.startswith(("base_joint_", "arm_"))}
        planned["base_joint_x"], planned["base_joint_y"], planned["base_joint_theta"] = base
        return joint_goal(planned)

    return [
        ("reach", goal(PICK, frame, BACK), scene(frame, PICK), False, False),
        ("approach", goal(PICK, frame), scene(frame, PICK), False, True),
        ("grip", CLOSED),
        ("lift", goal(up, frame), scene(frame, None, without=("table_a",)), True, False),
        ("carry", driven, scene(frame, None), True, False),
        ("lower", goal(PLACE, frame), scene(frame, None, without=("table_b",)), True, False),
        ("grip", OPEN),
        ("retreat", goal(PLACE, frame, BACK), scene(frame, PLACE), False, True),
    ]


def carry(send, grip, frame: str, touch_links, start: dict):
    """Plans every leg of the carry from `start` (joint name to position), each from where the one
    before ends: `send(request, objects, allowed)` plans one (and runs it, if it executes) and
    returns its result, or None if it fails; `grip(opening)` moves the grippers. Returns each leg's
    name and result, or None at the first that fails."""
    joints, results = dict(start), []
    for step in steps(frame):
        if step[0] == "grip":
            if not grip(step[1]):
                return None
            joints.update({joint: step[1] for _, joint in GRIPPERS})
            continue
        name, target, objects, carrying, touching = step
        request = MotionPlanRequest(group_name=GROUP, allowed_planning_time=5.0)
        request.goal_constraints = [target(joints) if callable(target) else target]
        request.start_state.joint_state = JointState(name=list(joints), position=[float(v) for v in joints.values()])
        if carrying:
            request.path_constraints = carried(frame)
            request.start_state.attached_collision_objects = [held(touch_links)]
        result = send(request, objects, allowing("tray", touch_links) if touching else None)
        if result is None:
            return None
        trajectory = result.planned_trajectory.joint_trajectory
        joints.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
        results.append((name, result))
    return results
