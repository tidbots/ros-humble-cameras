#!/usr/bin/env python3
"""Launch file for USB webcam using usb_cam."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument('camera_name', default_value='webcam'),
        DeclareLaunchArgument('camera_info_url', default_value='file:///root/.ros/camera_info/webcam.yaml'),
        DeclareLaunchArgument('frame_id', default_value='webcam_link'),
        DeclareLaunchArgument('out_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('out_info', default_value='/camera/camera_info'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('device'),
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'framerate': LaunchConfiguration('fps'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
            remappings=[
                ('image_raw', LaunchConfiguration('out_image')),
                ('camera_info', LaunchConfiguration('out_info')),
            ],
        ),
    ])
