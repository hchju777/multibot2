import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import yaml

def generate_launch_description():
    target = 'keyboard_controller_params.yaml'

    multibot2_robot_dir = get_package_share_directory("multibot2_robot")

    robotConfig = os.path.join(multibot2_robot_dir, 'robot', target)

    use_sim_time = True

    with open(robotConfig) as robot_params:
        robot_params = yaml.load(robot_params, Loader=yaml.Loader)
        robot_params = robot_params['/**']['ros__parameters']['robot']
    
    # Fake Node
    fake_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot2_driver'), 'launch',
                                                   'fake_driver_launch.py')),
        launch_arguments={
            'robot_name': robot_params['name'],
            'robot_type': robot_params['type'],
            'robot_config': robotConfig,
            'frame_prefix': robot_params['name'] + '/',
            'odom_frame': robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'base_frame': robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'x': str(robot_params['spawn']['x']),
            'y': str(robot_params['spawn']['y']),
            'Y': str(robot_params['spawn']['theta'])
        }.items()
    )
    
    # Robot Node
    keyboard_controller_cmd = Node(
        package='multibot2_robot',
        namespace=robot_params['name'],
        executable='keyboard_controller',
        name='keyboard_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(fake_driver_cmd)
    ld.add_action(keyboard_controller_cmd)

    return ld