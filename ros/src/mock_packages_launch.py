#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # AMCL Localization Mock Node
        Node(
            package='amcl_localization',
            executable='amcl_mock_node',
            name='amcl_mock_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Person Tracking Mock Node
        Node(
            package='person_tracking',
            executable='person_tracking_mock_node',
            name='person_tracking_mock_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Path Planning Mock Node
        Node(
            package='path_planning',
            executable='path_planning_mock_node',
            name='path_planning_mock_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Kalman Filter Mock Node
        Node(
            package='kalman_filter',
            executable='kalman_filter_mock_node',
            name='kalman_filter_mock_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ]) 