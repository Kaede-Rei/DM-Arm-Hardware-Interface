from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    ros2_control_xacro = PathJoinSubstitution([
        FindPackageShare("dm_arm_ros2_control"),
        "urdf",
        "dm_arm.ros2_control.xacro",
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare("dm_arm_ros2_control"),
        "config",
        "ros2_controllers.yaml",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        ros2_control_xacro,
        " name:=dm_arm",
        " config_file:=",
        config_file,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
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

    return LaunchDescription([
        DeclareLaunchArgument("config_file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("controller_manager_name", default_value="/controller_manager"),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
