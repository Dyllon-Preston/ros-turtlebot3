#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='navigate_to_goal',
            executable='actual_odom',
            name='actual_odom',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='navigate_to_goal',
            executable='object_range',
            name='object_range',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='navigate_to_goal',
            executable='goal_manager',
            name='goal_manager',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='navigate_to_goal',
            executable='state_manager',
            name='state_manager',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='navigate_to_goal',
            executable='go_to_goal',
            name='go_to_goal',
            output='screen'
        ),
    ])
