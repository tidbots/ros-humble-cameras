#!/usr/bin/env python3
"""Launch file for Intel RealSense camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='realsense'),
        DeclareLaunchArgument('in_image', default_value='/realsense/color/image_raw'),
        DeclareLaunchArgument('in_info', default_value='/realsense/color/camera_info'),
        DeclareLaunchArgument('out_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_info', default_value='/camera/camera_info'),

        # Include RealSense launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'camera_name': LaunchConfiguration('name'),
                'camera_namespace': LaunchConfiguration('name'),
            }.items(),
        ),

        # Relay color image to unified topic
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_rs_image',
            output='screen',
            arguments=[
                LaunchConfiguration('in_image'),
                LaunchConfiguration('out_image'),
            ],
        ),

        # Relay camera info to unified topic
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_rs_info',
            output='screen',
            arguments=[
                LaunchConfiguration('in_info'),
                LaunchConfiguration('out_info'),
            ],
        ),
    ])
