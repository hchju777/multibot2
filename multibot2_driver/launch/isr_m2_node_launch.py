import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    robot_name      = LaunchConfiguration("robot_name")

    odom_frame      = LaunchConfiguration("odom_frame")
    base_frame      = LaunchConfiguration("base_frame")
    laser_frame     = LaunchConfiguration("laser_frame")

    laser_offset_x = LaunchConfiguration("laser_offset_x")
    laser_offset_y = LaunchConfiguration("laser_offset_x")
    laser_offset_z = LaunchConfiguration("laser_offset_z")

    isr_m2_driver_node = Node(
        package='multibot2_driver',
        executable='isr_m2_node',
        name='isr_m2_node',
        namespace=robot_name,
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'baud': '115200'},
            {'odom_frame': odom_frame},
            {'base_frame': base_frame},
            {'use_sim_time': False}
        ],
        output='screen'
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        output='screen',
        arguments=['0','0','0','0','0','0',
                   odom_frame, base_frame]
    )

    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        output='screen',
        arguments=[laser_offset_x,
                   laser_offset_y,
                   laser_offset_z,
                   '0','0','0',
                   base_frame, laser_frame]

    )

    ld = LaunchDescription()

    ld.add_action(isr_m2_driver_node)
    ld.add_action(odom_to_base)
    ld.add_action(base_to_laser)

    return ld