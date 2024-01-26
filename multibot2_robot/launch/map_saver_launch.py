import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        namespace='DiffDrive',
        name='map_saver_cli',
        output='screen',
        arguments=[
            '-f', 'my_map'
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace='DiffDrive',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_saver_cli']}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager)

    return ld