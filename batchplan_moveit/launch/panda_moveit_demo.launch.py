"""MoveIt's Panda demo with batchplan in the planner list.

The stock moveit_resources Panda demo (mock hardware, move_group, RViz), with a batchplan pipeline next
to OMPL and the batchplan_planner node it forwards to. In RViz's MotionPlanning panel, pick
"batchplan" under Context > Planning Library, then Plan or Plan & Execute.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def with_batchplan(moveit_config):
    """Adds the batchplan pipeline to a MoveIt config's planning pipelines."""
    share = Path(get_package_share_directory("batchplan_moveit"))
    pipelines = moveit_config.planning_pipelines
    pipelines["planning_pipelines"].append("batchplan")
    pipelines["batchplan"] = yaml.safe_load((share / "config" / "batchplan_planning.yaml").read_text())
    return moveit_config


def generate_launch_description():
    panda = Path(get_package_share_directory("moveit_resources_panda_moveit_config"))
    moveit_config = with_batchplan(
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    spawn = lambda name: Node(package="controller_manager", executable="spawner", arguments=[name, "-c", "/controller_manager"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument("device", default_value="cpu", description="'cpu' or 'gpu'"),
            Node(
                package="batchplan_ros",
                executable="batchplan_planner",
                parameters=[
                    {
                        "robot_description_semantic": moveit_config.robot_description_semantic["robot_description_semantic"],
                        "joint_limits_file": str(panda / "config" / "joint_limits.yaml"),
                        "device": LaunchConfiguration("device"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict()],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(panda / "launch" / "moveit.rviz")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="log",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
                output="log",
            ),
            Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[moveit_config.robot_description]),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(panda / "config" / "ros2_controllers.yaml")],
                remappings=[("/controller_manager/robot_description", "/robot_description")],
                output="screen",
            ),
            spawn("joint_state_broadcaster"),
            spawn("panda_arm_controller"),
            spawn("panda_hand_controller"),
        ]
    )
