"""MoveIt messages for batchplan requests: poses, objects and goals."""

import numpy as np
from viser.transforms import SO3
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

# panda_hand is panda_link8 turned -45° about z.
HAND_IN_LINK8 = -np.pi / 4


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


def pose_goal(link: str, frame: str, position, wxyz) -> Constraints:
    position_constraint = PositionConstraint(link_name=link, weight=1.0)
    position_constraint.header.frame_id = frame
    position_constraint.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-3])]
    position_constraint.constraint_region.primitive_poses = [pose_msg(position, (1.0, 0.0, 0.0, 0.0))]
    orientation = OrientationConstraint(link_name=link, weight=1.0)
    orientation.header.frame_id = frame
    orientation.orientation = pose_msg((0.0, 0.0, 0.0), wxyz).orientation
    orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = 1e-2
    orientation.absolute_z_axis_tolerance = 1e-2
    return Constraints(position_constraints=[position_constraint], orientation_constraints=[orientation])


def grasp_poses(center, radius: float, count: int, facing=(0.0, 0.0), standoff: float = 0.15):
    """`count` approach poses of the Panda's panda_link8 around a vertical cylinder at `center`: the
    hand points at the cylinder's axis from `standoff` beyond its surface, its fingers closing
    horizontally, the first approaching from the side facing the point `facing` (x, y), such as the
    robot's base. Returns (position, wxyz) pairs."""
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
        poses.append((np.asarray(center, dtype=float) + out * (radius + standoff), SO3.from_matrix(link8).wxyz))
    return poses


def seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9
