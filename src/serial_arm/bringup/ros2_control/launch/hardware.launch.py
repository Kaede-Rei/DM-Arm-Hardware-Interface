from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from profile_utils import load_profile


def resolve_profile(context):
    robot_profile = context.launch_configurations["robot_profile"]
    profile = load_profile(robot_profile)
    use_sim_time = context.launch_configurations.get(
        "use_sim_time", "false"
    ).lower() in ("true", "1", "yes")
    config_file = profile["core_config_path"]
    hardware_plugin = profile["hardware_plugin"]
    hardware_config = profile["hardware_config_path"]
    controllers_file = profile["controllers_path"]
    description_xacro = profile["ros2_control_xacro_path"]
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_xacro,
            " config_file:=",
            config_file,
            " hardware_plugin:=",
            hardware_plugin,
            " hardware_config:=",
            hardware_config,
        ]
    )
    robot_description_param = ParameterValue(robot_description, value_type=str)
    controller_manager_name = context.launch_configurations.get(
        "controller_manager_name", "/controller_manager"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_param,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_param},
            controllers_file,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    return [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        joint_trajectory_controller,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "controller_manager_name", default_value="/controller_manager"
            ),
            OpaqueFunction(function=resolve_profile),
        ]
    )
