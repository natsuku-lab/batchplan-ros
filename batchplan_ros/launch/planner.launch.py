"""The batchplan planning node.

The robot comes from the latched /robot_description topic unless `robot_description` is given.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = {
    "robot_description_semantic": ("", "SRDF text: planning groups, end effectors, disabled collisions"),
    "joint_limits_file": ("", "MoveIt joint_limits.yaml"),
    "device": ("cpu", "'cpu' or 'gpu'"),
    "controller_action": ("", "FollowJointTrajectory action used to execute plans"),
}


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=doc) for name, (default, doc) in ARGUMENTS.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in ARGUMENTS}
    return LaunchDescription(
        declared
        + [Node(package="batchplan_ros", executable="batchplan_planner", output="screen", parameters=[parameters])]
    )
