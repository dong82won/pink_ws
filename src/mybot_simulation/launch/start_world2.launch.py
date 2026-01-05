#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mybot_simulation = get_package_share_directory('mybot_simulation')

    gazebo_models_path = os.path.join(pkg_mybot_simulation, 'models', 'small_house')
    my_models_path = os.path.join(pkg_mybot_simulation, 'models', 'my_building')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path + ':' + my_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  gazebo_models_path + ':' + my_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_mybot_simulation, 'worlds', 'small_house.world'), ''],
            #default_value=[os.path.join(pkg_mybot_simulation, 'worlds', 'my_world_exmaple.world'), ''],
            description='SDF world file'),
            gazebo
    ])