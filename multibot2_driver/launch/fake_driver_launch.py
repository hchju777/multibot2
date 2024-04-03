import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    multibot2_driver_dir = get_package_share_directory("multibot2_driver")

    # xacro = os.path.join(multibot2_driver_dir, 'models', 'DiffDrive', 'model.xacro')

    robot_name = LaunchConfiguration("robot_name")
    robot_type = LaunchConfiguration("robot_type")
    xacro = LaunchConfiguration("robot_model")
    robot_config = LaunchConfiguration("robot_config")

    frame_prefix = LaunchConfiguration("frame_prefix")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    Y = LaunchConfiguration('Y')
    
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(multibot2_driver_dir, 'launch',
                                                   'spawn_robot_launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'namespace': robot_name,
            'xacro': xacro,
            'frame_prefix': frame_prefix,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'use_sim_time': 'True',
            'x': x,
            'y': y,
            'Y': Y
        }.items()
    )

    fake_node = Node(
        package='multibot2_driver',
        namespace=robot_name,
        executable='fake_driver',
        name='fake_driver',
        parameters=[
            robot_config,
            {'namespace': robot_name,
             'type': robot_type,
             'use_gazebo_odom': True,
             'use_sim_time': True}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(fake_node)

    return ld