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

    use_sim_time = False

    with open(robotConfig) as robot_params:
        robot_params = yaml.load(robot_params, Loader=yaml.Loader)
        robot_params = robot_params['/**']['ros__parameters']['robot']
    
    # ISR_M2 Driver
    isr_m2_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot2_driver'), 'launch',
                                                   'isr_m2_node_launch.py')),
        launch_arguments={
            'robot_name': robot_params['name'],
            'odom_frame': robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'base_frame': robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'laser_frame': robot_params['name'] + '/' + robot_params['laser']['frame_id'],
            'laser_offset_x': str(robot_params['laser']['offset_x']),
            'laser_offset_y': str(robot_params['laser']['offset_y']),
            'laser_offset_z': str(robot_params['laser']['offset_z'])
        }.items()
    )
    
    # LIDAR
    if (robot_params['laser']['type'] == "sick_tim"):
        lidar_driver = Node(
            package='sick_tim',
            executable='sick_tim551_2050001',
            name='sick_tim_driver',
            namespace=robot_params['name'],
            parameters=[
                {'range_max': '25.0'},
                {'hostname' : robot_params['laser']['hostname']},
                {'port': '2112'},
                {'timelimit': 5},
                {'frame_id': robot_params['name'] + '/' + robot_params['laser']['frame_id']},
                {'use_sim_time': use_sim_time}
            ]
        )
    elif (robot_params['laser']['type'] == "hokuyo"):
        lidar_driver = Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_driver',
            namespace=robot_params['name'],
            output='screen',
            parameters=[
                {'ip_address': robot_params['laser']['hostname']},
                {'laser_frame_id': robot_params['name'] + '/' + robot_params['laser']['frame_id']},
                {'use_sim_time': use_sim_time}
            ]
        )
    else:
        print("WRONG_LIDAR_TYPE")
        os.abort()
    
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
    ld.add_action(isr_m2_node_cmd)
    ld.add_action(keyboard_controller_cmd)
    ld.add_action(lidar_driver)

    return ld