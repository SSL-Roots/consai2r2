# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # parameter
    dev = LaunchConfiguration('dev')
    # sim = LaunchConfiguration('sim')  # TODO : 現在未使用 シミュレータの切り替え用

    include_consai2r2_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('consai2r2_description'),
            'launch'),
            '/config.launch.py'
        ]),
    )

    declare_dev_cmd = DeclareLaunchArgument(
        'dev', default_value='/dev/input/js0',
        description='joystick device file'
    )

    declare_joyconfig = DeclareLaunchArgument(
        'joyconfig', default_value='f710',
        description='Keyconfig of joystick controllers: {f710, dualshock3}'
    )

    def func_get_joyconfig_filename(context):
        param_file = os.path.join(
            get_package_share_directory('consai2r2_teleop'),
            'config',
            'joy_' + context.launch_configurations['joyconfig'] + '.yaml')

        if os.path.exists(param_file):
            return [SetLaunchConfiguration('joyconfig_filename', param_file)]
        else:
            return [LogInfo(msg=param_file + ' is not exist.')]
    get_joyconfig_filename = OpaqueFunction(function=func_get_joyconfig_filename)

    start_joy_node_cmd = Node(
        package='joy', node_executable='joy_node',
        output='screen',
        parameters=[{'dev': dev}]
    )

    start_teleop_node_cmd = Node(
        package='consai2r2_teleop', node_executable='teleop_node',
        output='screen',
        parameters=[LaunchConfiguration('joyconfig_filename')]
    )

    start_sender_cmd = Node(
        package='consai2r2_sender', node_executable='sim_sender',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'consai2r2_sender'), 'config', 'grsim.yaml')]
    )

    ld = LaunchDescription()

    ld.add_action(include_consai2r2_description)
    ld.add_action(declare_dev_cmd)
    ld.add_action(declare_joyconfig)

    ld.add_action(get_joyconfig_filename)

    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_teleop_node_cmd)
    ld.add_action(start_sender_cmd)

    return ld
