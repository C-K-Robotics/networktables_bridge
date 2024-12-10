import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    pkg_name = 'networktable_bridge'
        
    pub_node=Node(
        executable='nt_client_pub_node',
        package=pkg_name,
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name),
                                 'config',
                                 'nt_client_pub.yaml')],
    )

    sub_node=Node(
        executable='nt_client_sub_node',
        package=pkg_name,
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name),
                                 'config',
                                 'nt_client_sub.yaml')],
    )

    ld = LaunchDescription()
    ld.add_action(pub_node)
    ld.add_action(sub_node)
    
    return ld
