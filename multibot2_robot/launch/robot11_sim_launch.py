import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import yaml

def generate_launch_description():
    target = 'robot11_params.yaml'

    multibot2_robot_dir = get_package_share_directory("multibot2_robot")

    robotConfig = os.path.join(multibot2_robot_dir, 'robot', target)

    use_sim_time = True

    with open(robotConfig) as robot_params:
        robot_params = yaml.load(robot_params, Loader=yaml.Loader)
        robot_params = robot_params['/**']['ros__parameters']['robot']
    
    # Fake Node
    xacro = os.path.join(get_package_share_directory("multibot2_driver"), 'models', robot_params['model'], 'model.xacro')
    
    fake_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot2_driver'), 'launch',
                                                   'fake_driver_launch.py')),
        launch_arguments={
            'robot_name': robot_params['name'],
            'robot_type': robot_params['type'],
            'robot_model': xacro,
            'robot_config': robotConfig,
            'frame_prefix': robot_params['name'] + '/',
            'odom_frame': robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'base_frame': robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'x': str(robot_params['spawn']['x']),
            'y': str(robot_params['spawn']['y']),
            'Y': str(robot_params['spawn']['theta'])
        }.items()
    )
    
    # Rviz
    rviz_config_dir = os.path.join(
        multibot2_robot_dir,
        'rviz',
        'multibot2_robot.rviz'
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
        output='screen',
        remappings=[
            ('/initialpose', '/'+robot_params['name']+'/initialpose'),
            ('/goal_pose', '/'+robot_params['name']+'/goal_pose'),
            ('/global_costmap/costmap', '/'+robot_params['name']+'/global_costmap/costmap'),
            ('/global_costmap/costmap_updates', '/'+robot_params['name']+'/global_costmap/costmap_updates'),
            ('/local_costmap/costmap', '/'+robot_params['name']+'/local_costmap/costmap'),
            ('/local_costmap/costmap', '/'+robot_params['name']+'/local_costmap/costmap'),
            ]
    )

    # AMCL
    amcl_params = RewrittenYaml(
        source_file=os.path.join(multibot2_robot_dir, 'params', 'amcl.yaml'),
        root_key=robot_params['name'],
        param_rewrites={
            'base_frame_id':   robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'odom_frame_id':   robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'scan_topic': '/'+ robot_params['name'] + '/' + robot_params['laser']['scan_topic'],
            'x': str(robot_params['spawn']['x']),
            'y': str(robot_params['spawn']['y']),
            'yaw': str(robot_params['spawn']['theta'])
        },
        convert_types=True
    )

    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=robot_params['name'],
        output='screen',
        parameters=[
            amcl_params,
            {'use_sim_time': use_sim_time}
            ],
        remappings=[
            ('/initialpose', '/'+robot_params['name']+'/initialpose')
            ]
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=robot_params['name'],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )

    # Robot Node
    global_costmap_Config = RewrittenYaml(
        source_file = os.path.join(multibot2_robot_dir, 'params', 'global_costmap.yaml'),
        root_key = robot_params['name'],
        param_rewrites={
            'robot_base_frame': robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'robot_radius': str(robot_params['radius']),
            'inflation_radius': str(robot_params['radius']),
            'use_sim_time': str(use_sim_time)
        },
        convert_types=True
    )
    
    local_costmap_Config = RewrittenYaml(
        source_file = os.path.join(multibot2_robot_dir, 'params', 'local_costmap.yaml'),
        root_key = robot_params['name'],
        param_rewrites={
            'global_frame':     robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'robot_base_frame': robot_params['name'] + '/' + robot_params['odometry']['child_frame_id'],
            'topic':             '/'+ robot_params['name'] + '/' + robot_params['laser']['scan_topic'],
            'robot_radius': str(robot_params['radius']),
            'inflation_radius': str(robot_params['radius']),
            'use_sim_time': str(use_sim_time)
        },
        convert_types=True
    )

    optimConfig = RewrittenYaml(
        source_file = os.path.join(multibot2_robot_dir, 'params', 'trajectory_optimization.yaml'),
        root_key = robot_params['name'],
        param_rewrites={
            'odom_topic': '/' + robot_params['name'] + '/' + robot_params['odometry']['topic'],
            'map_frame': robot_params['name'] + '/' + robot_params['odometry']['frame_id'],
            'footprint_model.type': str(robot_params['footprint_model']['type']),
            'footprint_model.front_offset': str(robot_params['footprint_model']['front_offset']),
            'footprint_model.front_radius': str(robot_params['footprint_model']['front_radius']),
            'footprint_model.rear_offset': str(robot_params['footprint_model']['rear_offset']),
            'footprint_model.rear_radius': str(robot_params['footprint_model']['rear_radius']),
            'inflation_dist': str(robot_params['radius']),
            # 'dynamic_obstacle_inflation_dist': str(robot_params['radius']),
            'max_vel_x': str(robot_params['velocity_profile']['max_vel_x']),
            'max_vel_theta': str(robot_params['velocity_profile']['max_vel_theta']),
            'acc_lim_x': str(robot_params['velocity_profile']['acc_lim_x']),
            'acc_lim_theta': str(robot_params['velocity_profile']['acc_lim_theta']),
            'max_global_plan_lookahead_dist': str(robot_params['avoidance_dist'])
        },
        convert_types=True
    )

    multibot2_robot_cmd = Node(
        package='multibot2_robot',
        namespace=robot_params['name'],
        executable='robot',
        name='robot',
        parameters=[
            robotConfig,
            optimConfig,
            local_costmap_Config,
            global_costmap_Config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(fake_driver_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(multibot2_robot_cmd)

    return ld