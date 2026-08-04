from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from profile_utils import load_profile, require_moveit_package


def resolve_profile(context):
    robot_profile = context.launch_configurations["robot_profile"]
    profile = load_profile(robot_profile)
    moveit_package = require_moveit_package(profile, robot_profile)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("serial_arm_ros2_control"),
                        "launch",
                        "hardware.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "robot_profile": robot_profile,
                "use_sim_time": context.launch_configurations.get(
                    "use_sim_time", "false"
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(moveit_package),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(moveit_package),
                        "launch",
                        "moveit_rviz.launch.py",
                    ]
                )
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=resolve_profile),
        ]
    )
