from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

URDF_PATH = "DM-Arm-Description.urdf"


def generate_launch_description():
    # 查找包路径
    pkg_share = FindPackageShare("dm_arm_description").find("dm_arm_description")

    # URDF 文件路径
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", URDF_PATH])

    # 判断是否使用 xacro
    use_xacro = True if URDF_PATH.endswith(".xacro") else False

    return LaunchDescription(
        [
            # 从 URDF 加载并发布 robot_description
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            (
                                Command(["xacro ", urdf_path])
                                if use_xacro
                                else Command(["cat ", urdf_path])
                            ),
                            value_type=str,
                        )
                    }
                ],
            ),
            # 启动 joint_state_publisher_gui
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            # 启动 RViz 并加载配置
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([pkg_share, "config", "view_robot.rviz"]),
                ],
            ),
        ]
    )
