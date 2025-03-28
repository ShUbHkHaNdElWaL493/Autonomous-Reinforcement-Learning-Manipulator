#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    package_share_directory = get_package_share_directory('manipulator_rviz')
    rviz_config_path = os.path.join(package_share_directory, 'config', 'manipulator_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])