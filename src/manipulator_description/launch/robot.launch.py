#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    package_share_directory = get_package_share_directory('manipulator_description')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description' : Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                    "\"" + package_share_directory + "/urdf/manipulator.xacro\""
                ])
            }
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])