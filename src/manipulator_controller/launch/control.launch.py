#   CS22B1090
#   Shubh Khandelwal

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    trainer_node = Node(
        package='manipulator_controller',
        executable='trainer.py',
        output='screen'
    )

    controller_node = Node(
        package='manipulator_controller',
        executable='controller.py',
        output='screen'
    )

    return LaunchDescription([
        trainer_node
    ])