#   CS22B1090
#   Shubh Khandelwal

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("manipulator"), "launch", "robot.launch.py")),
        launch_arguments = {
            "use_sim_time" : use_sim_time
        }.items()
    )

    train_node = Node(
        package = "controller",
        executable = "train_node.py",
        parameters = [{"use_sim_time" : use_sim_time}],
        output = "screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "true",
            description = "Use simulation (Gazebo) clock if true"
        ),
        spawner_node,
        train_node
    ])