from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolve_profile(context):
    robot_profile = context.launch_configurations.get("robot_profile", "gray")
    if robot_profile not in ("gray", "white"):
        raise RuntimeError("robot_profile must be 'gray' or 'white'")

    description_xacro = PathJoinSubstitution([
        FindPackageShare("dm_arm_description"),
        "urdf",
        "dm_arm.urdf.xacro",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        description_xacro,
        " robot_profile:=",
        robot_profile,
    ])

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": context.launch_configurations.get("use_sim_time", "false"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_profile", default_value="gray"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=resolve_profile),
    ])
