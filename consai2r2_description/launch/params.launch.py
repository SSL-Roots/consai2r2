import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config_path = os.path.join(
        get_package_share_directory( 'consai2r2_description' ),
        'config', 'config.yaml'
    )

    ld = LaunchDescription([
        Node(
            package='consai2r2_description', node_executable='consai2r2_description_node', output='screen',
            parameters=[config_path,])
    ])

    return ld
    