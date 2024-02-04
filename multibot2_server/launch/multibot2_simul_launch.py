import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml

def generate_launch_description():
    
    multibot2_server_dir = get_package_share_directory("multibot2_server")
    multibot2_robot_dir = get_package_share_directory("multibot2_robot")
    
    multibot2_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            multibot2_server_dir, 'launch', 'multibot2_server_simul_launch.py'))
    )
    
    with open(os.path.join(multibot2_server_dir, 'params', 'robot_common_config.yaml')) as robot_common_config:
        robot_common_config = yaml.load(robot_common_config, Loader=yaml.Loader)
        robot_common_config = robot_common_config['/**']['ros__parameters']['robot']
    
    robots = ["robot1"]
    # robots = ["robot1", "robot2"]
    
    multibot2_robot_cmds = []        
    for robot in robots:
        multibot2_robot_launch = robot + '_simul_launch.py'
                
        multibot2_robot_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    multibot2_robot_dir, 'launch', multibot2_robot_launch)),
            )
        )

    ld = LaunchDescription()
    
    ld.add_action(multibot2_server_cmd)
    
    for multibot2_robot_cmd in multibot2_robot_cmds:
        ld.add_action(multibot2_robot_cmd)
    
    return ld