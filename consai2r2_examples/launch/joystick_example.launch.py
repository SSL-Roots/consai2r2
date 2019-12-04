from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='consai2r2_teleop', node_executable='teleop_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='consai2r2_sender', node_executable='sim_sender',
            output='screen'
        ),
    ])