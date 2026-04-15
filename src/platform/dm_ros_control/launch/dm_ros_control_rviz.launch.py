from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("dm_ros_control")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    kp = LaunchConfiguration("kp")
    kd = LaunchConfiguration("kd")
    max_position_change = LaunchConfiguration("max_position_change")
    max_velocity = LaunchConfiguration("max_velocity")
    enable_write = LaunchConfiguration("enable_write")
    refresh_state_in_read = LaunchConfiguration("refresh_state_in_read")
    startup_read_cycles = LaunchConfiguration("startup_read_cycles")

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([pkg_share, "urdf", "dm_arm_ros_control.urdf.xacro"]),
            " use_fake_hardware:=",
            use_fake_hardware,
            " serial_port:=",
            serial_port,
            " baudrate:=",
            baudrate,
            " kp:=",
            kp,
            " kd:=",
            kd,
            " max_position_change:=",
            max_position_change,
            " max_velocity:=",
            max_velocity,
            " enable_write:=",
            enable_write,
            " refresh_state_in_read:=",
            refresh_state_in_read,
            " startup_read_cycles:=",
            startup_read_cycles,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "config", "view_robot.rviz"]),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument("kp", default_value="30.0"),
            DeclareLaunchArgument("kd", default_value="1.0"),
            DeclareLaunchArgument("max_position_change", default_value="0.5"),
            DeclareLaunchArgument("max_velocity", default_value="3.0"),
            DeclareLaunchArgument("enable_write", default_value="true"),
            DeclareLaunchArgument("refresh_state_in_read", default_value="true"),
            DeclareLaunchArgument("startup_read_cycles", default_value="5"),
            rsp,
            control_node,
            jsb,
            arm,
            rviz,
        ]
    )
