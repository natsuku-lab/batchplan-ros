"""A simulated robot planned with batchplan from a browser, without MoveIt: MoveIt's Panda, or PAL
Robotics' TIAGo Pro (`robot:=tiago_pro`, once `pixi run fetch-tiago-pro` has fetched it), whose base
and arms plan as one.

ros2_control mock hardware runs the robot; batchplan_planner plans and executes through its
trajectory controller; batchplan_viser serves the scene at http://localhost:8080 and moves the
grippers through theirs.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Each robot's description, planning setup, controllers, and the group and link the browser's
# gizmo moves.
ROBOTS = {
    "panda": {
        "xacro": "urdf/panda.urdf.xacro",
        "srdf": "config/panda.srdf",
        "controllers": "config/panda_controllers.yaml",
        "spawn": ["panda_arm_controller", "panda_hand_controller"],
        "joint_limits": "",
        "group": "panda_arm",
        "tip": "panda_link8",
    },
    "tiago_pro": {
        "xacro": "urdf/tiago_pro.urdf.xacro",
        "srdf": "tiago_pro/tiago_pro.srdf",
        "controllers": "config/tiago_pro_controllers.yaml",
        "spawn": ["whole_body_controller", "gripper_left_controller", "gripper_right_controller"],
        "joint_limits": "config/tiago_pro_joint_limits.yaml",
        "group": "base_left_arm",
        "tip": "gripper_left_tray_link",
    },
}


def nodes(context):
    name = LaunchConfiguration("robot").perform(context)
    if name not in ROBOTS:
        raise ValueError(f"robot:={name}: choose one of {', '.join(ROBOTS)}")
    robot = ROBOTS[name]
    share = Path(get_package_share_directory("batchplan_examples"))
    urdf = xacro.process_file(str(share / robot["xacro"])).toxml()
    srdf = (share / robot["srdf"]).read_text()
    controllers = str(share / robot["controllers"])
    spawn = lambda controller: Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--param-file", controllers],
        output="screen",
    )
    limits = str(share / robot["joint_limits"]) if robot["joint_limits"] else ""
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}]),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        spawn("joint_state_broadcaster"),
        *map(spawn, robot["spawn"]),
        Node(
            package="batchplan_ros",
            executable="batchplan_planner",
            parameters=[
                {
                    "robot_description_semantic": srdf,
                    "joint_limits_file": limits,
                    "device": LaunchConfiguration("device"),
                    "controller_action": f"/{robot['spawn'][0]}/follow_joint_trajectory",
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
                    "robot": name,
                    "group": robot["group"],
                    "tip_link": robot["tip"],
                    "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="'panda' or 'tiago_pro'"),
            DeclareLaunchArgument("device", default_value="cpu", description="'cpu' or 'gpu'"),
            DeclareLaunchArgument("port", default_value="8080", description="Port of the browser scene"),
            DeclareLaunchArgument("ui", default_value="true", description="Serve the browser scene"),
            OpaqueFunction(function=nodes),
        ]
    )
