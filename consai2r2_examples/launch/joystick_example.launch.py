from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    # parameter
    param_joydev = LaunchConfiguration('joydev')
    param_sim = LaunchConfiguration('sim') #TODO : 現在未使用 シミュレータの切り替え用
    # TODO : consai2r2_descriptionからのパラメータ読み込み

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'joydev', default_value='/dev/input/js0',
            description='joystick device file'
    )])

    joy_node = Node(
            package='joy', node_executable='joy_node',
            output='screen',
            parameters=[{'_dev' : param_joydev}]
    )

    teleop_node = Node(
            package='consai2r2_teleop', node_executable='teleop_node',
            output='screen'
    )

    sender = Node(
            package='consai2r2_sender', node_executable='sim_sender',
            output='screen'
    )

    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(sender)

    return ld
    