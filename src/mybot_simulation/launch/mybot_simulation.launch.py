#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 패키지 경로
    pkg_mybot_simulation = get_package_share_directory('mybot_simulation')
    pkg_mybot_description = get_package_share_directory('mybot_description')

    # RViz 설정 파일 경로
    rviz_config_path = os.path.join(pkg_mybot_description, 'rviz', 'urdf_vis.rviz')

    # 파라미터 선언
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot'
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose', default_value='2.0',
        description='Z position of the robot'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz'
    )

    # 파라미터 참조
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_rviz = LaunchConfiguration('use_rviz')

    # world 런치 파일
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot_simulation, 'launch', 'start_world.launch.py')
        )
    )

    # spawn launch 파일
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        # params
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_rviz_arg,
        # launch files, node
        world_launch,
        spawn_launch,
        rviz_node
    ])