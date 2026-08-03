from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from profile_utils import load_profile


def resolve_profile(context):
    robot_profile = context.launch_configurations.get("robot_profile", "gray")
    profile = load_profile(robot_profile)
    use_sim_time = context.launch_configurations.get(
        "use_sim_time", "false"
    ).lower() in ("true", "1", "yes")

    description_xacro = PathJoinSubstitution(
        [
            FindPackageShare("serial_arm_description"),
            "robots",
            "dm_arm",
            "urdf",
            "dm_arm.urdf.xacro",
        ]
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_xacro,
            " robot_profile:=",
            profile["description_profile"],
        ]
    )
    robot_description_param = ParameterValue(robot_description, value_type=str)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description_param,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[
                {
                    "robot_description": robot_description_param,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("serial_arm_bringup"),
                        "rviz",
                        "display.rviz",
                    ]
                ),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="gray"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=resolve_profile),
        ]
    )
