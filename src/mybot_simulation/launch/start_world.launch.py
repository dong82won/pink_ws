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
    # pkg_small_house = get_package_share_directory('aws_robomaker_small_house_world')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    
    # description_package_name = "mybot_description"
    # install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_mybot_simulation, 'models', 'small_house')
    base_models_path = os.path.join(pkg_mybot_simulation, 'models')

#    house_models_path = os.path.join(pkg_small_house, 'models')

    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH']
    #     + ':' + install_dir + '/share'
    #     + ':' + gazebo_models_path
    #     + ':' + house_models_path
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"
    #     + ':' + gazebo_models_path
    #     + ':' + house_models_path

    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path + ':' + base_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  gazebo_models_path + ':' + base_models_path

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
            #default_value=[os.path.join(pkg_small_house, 'worlds', 'small_house.world'), ''],
            description='SDF world file'),
            gazebo
    ])