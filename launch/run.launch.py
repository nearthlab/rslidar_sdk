import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches:
        - rslidar_sdk_node with a specified configuration file and namespace.

    Usage:
        ros2 launch rslidar_sdk run.launch.py [config_file:=<config_file>] [namespace:=<namespace>]

    Author: Junwoo Park <junwoo.park@nearthlab.com>
    """

    package_name = 'rslidar_sdk'
    package_share_dir = get_package_share_directory(package_name)
    default_config = 'airy.yaml'
    config_path = os.path.join(package_share_dir, 'config', default_config)
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='', # we do not use "ids" by default. it must be set explicitly by the user of this launch file
        description='Namespace for the ROS2 topics'
    )
    
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Configuration file for rslidar_sdk_node'
    )
    
    namespace = LaunchConfiguration('namespace')
    driver_node = Node(
        namespace=namespace,
        name='rslidar_driver_node',
        package=package_name,
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
    )
    
    return LaunchDescription([
        namespace_arg,
        config_file_arg,
        driver_node,
    ])
