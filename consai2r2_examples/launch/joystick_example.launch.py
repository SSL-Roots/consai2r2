from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # parameter
    dev = LaunchConfiguration('dev')
    sim = LaunchConfiguration('sim') #TODO : 現在未使用 シミュレータの切り替え用
    # TODO : consai2r2_descriptionからのパラメータ読み込み
    
    declare_dev_cmd = DeclareLaunchArgument(
            'dev', default_value='/dev/input/js0',
            description='joystick device file'
    )
    
    start_joy_node_cmd = Node(
            package='joy', node_executable='joy_node',
            output='screen',
            parameters=[{'dev' : dev}]
    )
    
    start_teleop_node_cmd = Node(
            package='consai2r2_teleop', node_executable='teleop_node',
            output='screen'
    )
    
    start_sender_cmd = Node(
            package='consai2r2_sender', node_executable='sim_sender',
            output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_dev_cmd)
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_teleop_node_cmd)
    ld.add_action(start_sender_cmd)

    return ld
    