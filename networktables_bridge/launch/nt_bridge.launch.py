import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_name = 'networktables_bridge'

    nt_server_ip_arg = DeclareLaunchArgument(
        'nt_server_ip', default_value='127.0.0.1',
        description='IP address of the NetworkTables server (roborio)'
    )

    bridge_node=LifecycleNode(
        package=pkg_name,
        executable='nt_bridge_node',
        name='nt_bridge_node',
        namespace='',
        output='screen',
        parameters=[
            {
                'nt_server_ip': LaunchConfiguration('nt_server_ip'),
            },
        ],
        remappings=[
            ('misc_report', 'misc_report'),
        ],
    )

    return LaunchDescription(
        [
            nt_server_ip_arg,
            bridge_node,
        ]
    )
