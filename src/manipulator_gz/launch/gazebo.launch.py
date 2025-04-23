#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    package_share_directory = get_package_share_directory('manipulator_gz')

    gazebo_node = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(package_share_directory, 'worlds', 'environment.world')],
        output='screen'
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-entity', 'manipulator'],
        output='screen'
    )

    gz_bridge_node = Node(
        package='manipulator_gz',
        executable='gz_bridge_node',
        output='screen'
    )

    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(package_share_directory, 'config', 'manipulator_controller.yaml')],
        output='screen'
    )

    joint_state_broadcaster_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    manipulator_controller_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'manipulator_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_node,
        spawn_entity_node,
        gz_bridge_node,
        controller_node,
        joint_state_broadcaster_node,
        manipulator_controller_node
    ])