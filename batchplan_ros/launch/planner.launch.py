"""The batchplan planning node.

The robot comes from the latched /robot_description topic unless `robot_description` is given.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = {
    "robot_description_semantic": ("", str, "SRDF text: planning groups, end effectors, disabled collisions"),
    "joint_limits_file": ("", str, "MoveIt joint_limits.yaml"),
    "device": ("cpu", str, "'cpu' or 'gpu'"),
    "controller_action": ("", str, "FollowJointTrajectory action used to execute plans"),
    "sphere_budget": ("64", int, "Collision spheres shared by selected robot links"),
    "world_sdf_voxel_size": ("0.01", float, "Maximum URDF world mesh SDF grid spacing in meters"),
    "trajectory_dt": ("0.02", float, "Returned trajectory sample interval in seconds"),
}


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=doc) for name, (default, _, doc) in ARGUMENTS.items()]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, (_, value_type, _) in ARGUMENTS.items()
    }
    declared.append(
        DeclareLaunchArgument(
            "robot_link_roots", default_value="[]",
            description="YAML list of roots whose strict descendants use robot spheres; [] selects all",
        )
    )
    declared.append(
        DeclareLaunchArgument(
            "parameters_file", default_value="",
            description="Optional ROS parameter YAML file for all backend/search settings",
        )
    )
    return LaunchDescription(
        declared
        + [OpaqueFunction(function=_planner_node, args=[parameters])]
    )


def _planner_node(context, parameters):
    parameters = dict(parameters)
    if LaunchConfiguration("robot_link_roots").perform(context).strip() not in ("", "[]"):
        parameters["robot_link_roots"] = ParameterValue(LaunchConfiguration("robot_link_roots"))
    parameter_sources = [parameters]
    parameters_file = LaunchConfiguration("parameters_file", default="").perform(context)
    if parameters_file:
        parameter_sources.append(parameters_file)
    return [Node(package="batchplan_ros", executable="batchplan_planner", output="screen", parameters=parameter_sources)]
