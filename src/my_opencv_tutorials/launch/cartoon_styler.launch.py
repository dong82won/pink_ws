import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 0. LaunchDescription 인스턴스 생성
    ld = LaunchDescription()

    # 1. 패키지 경로 및 YAML 파일 설정
    package_name = 'my_opencv_tutorials'
    pkg_share = get_package_share_directory(package_name)
    default_params_file = os.path.join(pkg_share, 'config', 'cam.yaml')

    # 2. 런치 인자(Argument) 선언 및 추가
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the config file to use'
    )
    ld.add_action(params_file_arg)

        # 3. 메인 카메라 노드 생성 및 추가
    camera_node = Node(
        package=package_name,
        executable='img_pub_param2',
        name='img_pub_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    ld.add_action(camera_node)


    # 3. 메인 카메라 노드 생성 및 추가
    img_cartoon_node = Node(
        package=package_name,
        executable='cartoon_multi_node',
        name='img_cartoon_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    ld.add_action(img_cartoon_node)


    # 5. 최종 LaunchDescription 반환
    return ld