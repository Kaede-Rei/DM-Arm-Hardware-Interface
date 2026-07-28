from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def resolve_profile(context):
    robot_profile = context.launch_configurations.get("robot_profile", "gray")
    if robot_profile not in ("gray", "white"):
        raise RuntimeError("robot_profile must be 'gray' or 'white'")
    use_sim_time = context.launch_configurations.get(
        "use_sim_time", "false"
    ).lower() in ("true", "1", "yes")

    config_file = LaunchConfiguration("config_file")
    ros2_control_xacro = PathJoinSubstitution(
        [
            FindPackageShare("dm_arm_ros2_control"),
            "urdf",
            "dm_arm.ros2_control.xacro",
        ]
    )
    controllers_file = LaunchConfiguration("controllers_file")
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            ros2_control_xacro,
            " robot_profile:=",
            robot_profile,
            " config_file:=",
            config_file,
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
    )

    joint_trajectory_controller_spawner = Node(
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
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="gray"),
            DeclareLaunchArgument("config_file"),
            DeclareLaunchArgument("controllers_file"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "controller_manager_name", default_value="/controller_manager"
            ),
            OpaqueFunction(function=resolve_profile),
        ]
    )
