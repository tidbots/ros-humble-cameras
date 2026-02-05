#!/usr/bin/env python3
"""Launch file for OpenCV viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('window_name', default_value='Camera View'),

        Node(
            package='camera_launch',
            executable='cv_viewer.py',
            name='cv_viewer',
            output='screen',
            parameters=[{
                'image': LaunchConfiguration('image'),
                'window_name': LaunchConfiguration('window_name'),
            }],
        ),
    ])
