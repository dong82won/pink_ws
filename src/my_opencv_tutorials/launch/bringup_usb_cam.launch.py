import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video6',
                'image_width': 640,
                'image_height': 480,
                # 'yuyv' 대신 'mjpeg' 또는 'yuyv2rgb' 를 사용하여 rqt 호환성 문제 해결
                'pixel_format': 'yuyv2rgb'
            }]
        )
    ])