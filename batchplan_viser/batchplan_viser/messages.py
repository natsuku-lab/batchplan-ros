"""MoveIt messages for batchplan requests: poses, objects and goals."""

import numpy as np
from viser.transforms import SO3
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

# panda_hand is panda_link8 turned -45° about z.
HAND_IN_LINK8 = -np.pi / 4
# How far along panda_link8's z axis a grasp puts an object's axis: between the finger pads, which
# reach from 5.84 to 11.2 cm out, and past the palm, which ends at 6.6 cm.
GRASP_DEPTH = 0.105
# The Panda's finger joint, each finger's distance from the middle, and how far it opens.
FINGER = "panda_finger_joint1"
FINGERS_OPEN = 0.04
# How far before a grasp the hand's approach begins, along its z, to move in from there.
APPROACH = 0.08
# How far from that pose the hand may leave its path, to move in without stopping there: half the
# approach, so it is straight for the last half.
BLEND = APPROACH / 2
# The links that may touch what the hand grasps: the fingers, and the palm, whose collision spheres
# reach past its meshes toward what the fingers close on.
HAND = ("panda_hand", "panda_leftfinger", "panda_rightfinger")


def pose_msg(position, wxyz) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    w, x, y, z = (float(v) for v in wxyz)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
    return pose


def solid(name: str, frame: str, position, wxyz, kind: int, dimensions) -> CollisionObject:
    shape = CollisionObject(id=name, operation=CollisionObject.ADD)
    shape.header.frame_id = frame
    shape.primitives = [SolidPrimitive(type=kind, dimensions=[float(v) for v in dimensions])]
    shape.primitive_poses = [pose_msg(position, wxyz)]
    return shape


def box_object(name: str, frame: str, position, wxyz, size) -> CollisionObject:
    return solid(name, frame, position, wxyz, SolidPrimitive.BOX, size)


def cylinder_object(name: str, frame: str, position, radius: float, height: float) -> CollisionObject:
    return solid(name, frame, position, (1.0, 0.0, 0.0, 0.0), SolidPrimitive.CYLINDER, (height, radius))


def positioned(link: str, frame: str, position, size: float, box: bool = False) -> PositionConstraint:
    """`link`'s origin within `size` of `position` in `frame`: a sphere of that radius, or a box of
    that side, as path constraints need."""
    constraint = PositionConstraint(link_name=link, weight=1.0)
    constraint.header.frame_id = frame
    shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size] * 3) if box else SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[size])
    constraint.constraint_region.primitives = [shape]
    constraint.constraint_region.primitive_poses = [pose_msg(position, (1.0, 0.0, 0.0, 0.0))]
    return constraint


def oriented(link: str, frame: str, wxyz, tolerances) -> OrientationConstraint:
    """`link` turned within `tolerances` (x, y, z, as XYZ Euler angles) of `wxyz` in `frame`."""
    constraint = OrientationConstraint(link_name=link, weight=1.0)
    constraint.header.frame_id = frame
    constraint.orientation = pose_msg((0.0, 0.0, 0.0), wxyz).orientation
    x, y, z = (float(t) for t in tolerances)
    constraint.absolute_x_axis_tolerance, constraint.absolute_y_axis_tolerance, constraint.absolute_z_axis_tolerance = x, y, z
    return constraint


def pose_goal(link: str, frame: str, position, wxyz) -> Constraints:
    return Constraints(position_constraints=[positioned(link, frame, position, 1e-3)], orientation_constraints=[oriented(link, frame, wxyz, (1e-2, 1e-2, 1e-2))])


def joint_goal(values: dict) -> Constraints:
    """Each joint at its value in `values` (name to position), within a milliradian."""
    joints = [JointConstraint(joint_name=name, position=float(v), tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0) for name, v in values.items()]
    return Constraints(joint_constraints=joints)


def allowing(obj: str, links) -> AllowedCollisionMatrix:
    """An allowed collision matrix in which `links` may touch the object `obj`, and nothing else."""
    names = [obj, *links]
    rows = [AllowedCollisionEntry(enabled=[(i == 0) != (j == 0) for j in range(len(names))]) for i in range(len(names))]
    return AllowedCollisionMatrix(entry_names=names, entry_values=rows)


def grasp_poses(center, count: int, facing=(0.0, 0.0), back: float = 0.0):
    """`count` grasps of a vertical cylinder at `center` by the Panda's hand, as poses of
    panda_link8: the hand points at the cylinder's axis, which lies between the finger pads
    (`GRASP_DEPTH`), its fingers closing horizontally, the first from the side facing the point
    `facing` (x, y), such as the robot's base. Each pose is moved `back` along the hand's approach,
    for the poses before the grasps. Returns (position, wxyz) pairs."""
    first = np.arctan2(facing[1] - center[1], facing[0] - center[0])
    turn = SO3.from_z_radians(HAND_IN_LINK8).as_matrix()
    poses = []
    for k in range(count):
        angle = first + 2 * np.pi * k / count
        out = np.array([np.cos(angle), np.sin(angle), 0.0])
        z = -out
        y = np.cross(z, [0.0, 0.0, 1.0])
        hand = np.column_stack([np.cross(y, z), y, z])
        link8 = hand @ turn.T
        poses.append((np.asarray(center, dtype=float) + out * (GRASP_DEPTH + back), SO3.from_matrix(link8).wxyz))
    return poses


def seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9
