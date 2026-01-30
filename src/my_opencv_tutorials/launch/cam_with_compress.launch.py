import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 사용자가 만든 Raw 이미지 퍼블리셔 (img_pub)
        Node(
            package='my_opencv_tutorials',
            executable='img_pub',
            name='img_pub_node',
            output='screen'
        ),

        # 2. Raw 이미지를 받아 Compressed 이미지로 변환해주는 노드 (image_transport)
        Node(
            package='image_transport',
            executable='republish',
            name='img_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                # 입력: 사용자의 토픽 (/image_raw)
                ('in', '/image_raw'),
                # 출력: 압축된 토픽 이름 (/image_raw/compressed)
                ('out/compressed', '/image_raw/compressed')
            ],
            output='screen'
        )
    ])