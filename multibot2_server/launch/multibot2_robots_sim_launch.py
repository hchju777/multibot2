import os
import threading

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    multibot2_robot_dir = get_package_share_directory("multibot2_robot")
    
    # List of robots to be simulated
    robots = ["robot1", "robot2", "robot3", "robot4", "robot5", "robot6"]
    
    ld = LaunchDescription()
    threads = []

    def add_robot_launch(robot):
        multibot2_robot_launch = robot + '_sim_launch.py'
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(multibot2_robot_dir, 'launch', multibot2_robot_launch)
                )
            )
        )

    # Create and start a thread for each robot's launch description
    for robot in robots:
        thread = threading.Thread(target=add_robot_launch, args=(robot,))
        thread.start()
        threads.append(thread)

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

    return ld