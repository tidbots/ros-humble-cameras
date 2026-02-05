#!/usr/bin/env python3
"""Launch file for ASUS Xtion using openni2_camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='xtion'),
        DeclareLaunchArgument('device_id', default_value=''),
        DeclareLaunchArgument('rgb_camera_info_url', default_value='file:///root/.ros/camera_info/xtion_rgb.yaml'),
        DeclareLaunchArgument('depth_camera_info_url', default_value='file:///root/.ros/camera_info/xtion_depth.yaml'),
        DeclareLaunchArgument('in_image', default_value='/xtion/rgb/image_raw'),
        DeclareLaunchArgument('in_info', default_value='/xtion/rgb/camera_info'),
        DeclareLaunchArgument('out_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_info', default_value='/camera/camera_info'),

        Node(
            package='openni2_camera',
            executable='openni2_camera_node',
            name='openni2_camera',
            namespace=LaunchConfiguration('name'),
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'rgb_camera_info_url': LaunchConfiguration('rgb_camera_info_url'),
                'depth_camera_info_url': LaunchConfiguration('depth_camera_info_url'),
            }],
        ),

        # Relay RGB image to unified topic
        Node(
            package='topic_tools',
            executable='relay',
            name='xtion_rgb_relay',
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
            name='xtion_info_relay',
            output='screen',
            arguments=[
                LaunchConfiguration('in_info'),
                LaunchConfiguration('out_info'),
            ],
        ),
    ])
