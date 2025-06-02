#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():

    manipulator = get_package_share_directory("manipulator")
    rviz_config_path = os.path.join(manipulator, "config", "manipulator_config.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time" : use_sim_time,
                "robot_description" : Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                    "\"" + manipulator + "/models/manipulator.xacro\""
                ])
            }
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "false",
            description = "Use simulation (Gazebo) clock if true"
        ),
        robot_state_publisher_node,
        rviz_node
    ])