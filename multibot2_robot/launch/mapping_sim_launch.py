import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import yaml

def generate_launch_description():
    target = 'mapping_robot_params.yaml'

    multibot2_robot_dir = get_package_share_directory("multibot2_robot")
    
    cartographer_config_dir = os.path.join(multibot2_robot_dir, 'config')
    configuration_basename = 'DiffDrive_lds_2d.lua'

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
    
    # Static Transform Publisher
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_params['name'],
        output='screen',
        arguments=[str(robot_params['spawn']['x']),str(robot_params['spawn']['y']),'0',str(robot_params['spawn']['theta']),'0','0',
                   'map', robot_params['name'] + '/' + robot_params['odometry']['frame_id']]
    )
    
    # Mapping
    mapping = Node(
        package='cartographer_ros',
        namespace=robot_params['name'],
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        namespace=robot_params['name'],
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-resolution', '0.1',
            '-publish_period_sec', '0.5'
        ]
    )
    
    #Rviz
    rviz_config_dir = os.path.join(
        multibot2_robot_dir,
        'rviz',
        'mapping.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_params['name'],
        arguments=['-d', rviz_config_dir],
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
    ld.add_action(map_to_odom)

    ld.add_action(mapping)
    ld.add_action(occupancy_grid_node)
    ld.add_action(start_rviz_cmd)

    return ld