import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

def generate_launch_description():
    xacro = LaunchConfiguration("xacro")

    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace", default="DiffDrive")

    frame_prefix = LaunchConfiguration("frame_prefix")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    use_sim_time = LaunchConfiguration("use_sim_time", default='false')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro, ' robot_name:=', robot_name])},
                    {'frame_prefix': frame_prefix},
                    {'use_sim_time': use_sim_time}]
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        output='screen',
        arguments=['0','0','0','0','0','0',
                   odom_frame, base_frame]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(odom_to_base)

    return ld