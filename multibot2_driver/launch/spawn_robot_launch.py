import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

def generate_launch_description():
    multibot2_driver_dir = get_package_share_directory("multibot2_driver")

    xacro = LaunchConfiguration("xacro")

    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace", default="DiffDrive")

    frame_prefix = LaunchConfiguration("frame_prefix")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    use_sim_time = LaunchConfiguration("use_sim_time", default='false')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    Y = LaunchConfiguration('Y')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(multibot2_driver_dir, 'launch',
                                                   'robot_state_publisher.py')),
        launch_arguments={
            'robot_name': robot_name,
            'namespace': namespace,
            'xacro': xacro,
            'frame_prefix': frame_prefix,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'use_sim_time': use_sim_time,
            'x': x,
            'y': y,
            'Y': Y
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        output='screen',
        arguments=[
            '-robot_namespace', namespace,
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-reference_frame', "map",
            '-x', x,
            '-y', y,
            '-Y', Y
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld