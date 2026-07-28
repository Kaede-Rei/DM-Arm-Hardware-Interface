from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def resolve_profile(context):
    robot_profile = context.launch_configurations.get("robot_profile", "gray")
    if robot_profile not in ("gray", "white"):
        raise RuntimeError("robot_profile must be 'gray' or 'white'")

    moveit_package = "dm_arm_no_gripper" if robot_profile == "gray" else "dm_arm_with_gripper"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("dm_arm_bringup"),
                    "launch",
                    "hardware.launch.py",
                ])
            ),
            launch_arguments={
                "robot_profile": robot_profile,
                "use_sim_time": context.launch_configurations.get("use_sim_time", "false"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(moveit_package),
                    "launch",
                    "move_group.launch.py",
                ])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(moveit_package),
                    "launch",
                    "moveit_rviz.launch.py",
                ])
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_profile", default_value="gray"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=resolve_profile),
    ])
