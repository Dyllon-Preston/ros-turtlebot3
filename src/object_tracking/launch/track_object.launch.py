#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='object_tracking',
            executable='select_object',
            name='select_object',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='object_tracking',
            executable='find_object',
            name='find_object',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='object_tracking',
            executable='rotate_robot',
            name='rotate_robot',
            output='screen'
        ),
    ])
