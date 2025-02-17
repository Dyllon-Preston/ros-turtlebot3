#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='object_following',
            executable='detect_object',
            name='detect_object',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='object_following',
            executable='object_range',
            name='object_range',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='object_following',
            executable='chase_object',
            name='chase_object',
            output='screen'
        ),
    ])
