#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    manipulator = get_package_share_directory("manipulator")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    xacro_file = os.path.join(manipulator, "models", "manipulator.xacro")
    robot_description = xacro.process_file(xacro_file)
    rviz_config_path = os.path.join(manipulator, "config", "config.rviz")

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            "use_sim_time" : use_sim_time,
            "robot_description" : robot_description.toxml()
        }]
    )

    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments = {
            "gz_args": ["-r -v2 ", world],
            "on_exit_shutdown": "true"
        }.items()
    )

    spawner_node = Node(
        package = "ros_gz_sim",
        executable = "create",
        output = "screen",
        arguments = [
            "-name", "manipulator",
            "-topic", "robot_description",
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose
        ]
    )

    bridge_parameters = os.path.join(
        manipulator,
        "config",
        "ros_gz_bridge.yaml"
    )

    ros_gz_bridge_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        arguments = [
            "--ros-args",
            "-p",
            f"config_file:={bridge_parameters}"
        ],
        output = "screen"
    )

    forward_position_controller_node = ExecuteProcess(
        cmd = ["ros2", "control", "load_controller", "--set-state", "active", "forward_position_controller"],
        output = "screen"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", rviz_config_path],
        output="screen"
    )

    ros_bridge_node = Node(
        package = "controller",
        executable = "ros_bridge_node",
        parameters = [{"use_sim_time" : use_sim_time}],
        output = "screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "true",
            description = "Use simulation (Gazebo) clock if true"
        ),
        DeclareLaunchArgument(
            "world",
            default_value = os.path.join(manipulator, "worlds", "empty_world.sdf"),
            description = "Specify world for Gazebo simulation"
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value = "0.0",
            description = "Specify namespace of the robot"
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value = "0.0",
            description = "Specify namespace of the robot"
        ),
        DeclareLaunchArgument(
            "z_pose",
            default_value = "0.0",
            description = "Specify namespace of the robot"
        ),
        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action = spawner_node,
                on_exit = [ros_gz_bridge_node],
            )
        ),
        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action = ros_gz_bridge_node,
                on_exit = [forward_position_controller_node],
            )
        ),
        robot_state_publisher_node,
        gz_node,
        spawner_node,
        rviz_node,
        ros_bridge_node
    ])