"""A simulated Panda planned with batchplan from a browser, without MoveIt.

ros2_control mock hardware runs the arm and its fingers; batchplan_planner plans and executes through
the arm's trajectory controller; batchplan_viser serves the scene at http://localhost:8080 and moves
the fingers through theirs.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("batchplan_examples"))
    urdf = xacro.process_file(str(share / "urdf" / "panda.urdf.xacro")).toxml()
    srdf = (share / "config" / "panda.srdf").read_text()
    controllers = str(share / "config" / "controllers.yaml")
    spawn = lambda name: Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--param-file", controllers],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="cpu", description="'cpu' or 'gpu'"),
            DeclareLaunchArgument("port", default_value="8080", description="Port of the browser scene"),
            DeclareLaunchArgument("ui", default_value="true", description="Serve the browser scene"),
            Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}]),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers],
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),
            spawn("joint_state_broadcaster"),
            spawn("panda_arm_controller"),
            spawn("panda_hand_controller"),
            Node(
                package="batchplan_ros",
                executable="batchplan_planner",
                parameters=[
                    {
                        "robot_description_semantic": srdf,
                        "device": LaunchConfiguration("device"),
                        "controller_action": "/panda_arm_controller/follow_joint_trajectory",
                    }
                ],
                output="screen",
            ),
            Node(
                package="batchplan_viser",
                executable="batchplan_viser",
                condition=IfCondition(LaunchConfiguration("ui")),
                parameters=[
                    {
                        "group": "panda_arm",
                        "tip_link": "panda_link8",
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                    }
                ],
                output="screen",
            ),
        ]
    )
