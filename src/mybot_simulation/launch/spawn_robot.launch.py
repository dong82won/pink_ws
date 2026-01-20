#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 패키지 경로
    pkg_mybot_description = get_package_share_directory('mybot_description')

    # URDF 파일 경로
    # urdf_path = os.path.join(pkg_mybot_description, 'urdf', 'mybot_collision_inertia.urdf')
    # urdf_path = os.path.join(pkg_mybot_description, 'urdf', 'mybot_collision_inertia_gazebo.urdf')
    # urdf_path = os.path.join(pkg_mybot_description, 'urdf', 'mybot_collision_inertia_gazebo_plugin.urdf')

    urdf_path = os.path.join(pkg_mybot_description, 'urdf', 'mybot_collision_plugin_sensor.urdf')

    # URDF 내용을 읽어오기 위한 Command 객체
    robot_desc = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 파라미터 참조 (상위 런치 파일에서 정의됨)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # 로봇 모델 spawn 노드
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mybot',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node
    ])