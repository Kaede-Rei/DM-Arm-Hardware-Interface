from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolve_profile(context):
    robot_profile = context.launch_configurations.get("robot_profile", "gray")
    if robot_profile not in ("gray", "white"):
        raise RuntimeError("robot_profile must be 'gray' or 'white'")

    config_file = PathJoinSubstitution([
        FindPackageShare("dm_arm_bringup"),
        "config",
        f"dm_arm_{robot_profile}.yaml",
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare("dm_arm_bringup"),
        "config",
        "ros2_controllers.yaml",
    ])
    description_xacro = PathJoinSubstitution([
        FindPackageShare("dm_arm_ros2_control"),
        "urdf",
        "dm_arm.ros2_control.xacro",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        description_xacro,
        " robot_profile:=",
        robot_profile,
        " config_file:=",
        config_file,
    ])
    controller_manager_name = context.launch_configurations.get("controller_manager_name", "/controller_manager")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": context.launch_configurations.get("use_sim_time", "false"),
        }],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
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
    return LaunchDescription([
        DeclareLaunchArgument("robot_profile", default_value="gray"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("controller_manager_name", default_value="/controller_manager"),
        OpaqueFunction(function=resolve_profile),
    ])
