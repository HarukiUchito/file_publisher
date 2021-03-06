# Copyright 2022 Haruki Uchiito
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

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rvizconfig = os.path.join(
        get_package_share_directory('file_publisher_node'),
        'rviz',
        'rviz.rviz'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='file_publisher_node',
            executable='file_publisher',
            output='log',
            emulate_tty=True,
            name='file_publisher_node',
            parameters=[{'base_link_frame_name': 'base_link'}]
        ),
        launch_ros.actions.Node(
            package='odom_3d_node',
            executable='odom_3d',
            output='screen',
            emulate_tty=True,
            name='odom_3d_node'
        )
    ])


'''
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            emulate_tty=True,
            name='rviz2',
            arguments=['-d', rvizconfig]
        )
        '''
