import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 공유 디렉토리 경로 찾기 ($(find-pkg-share my_opencv_tutorials))
    package_name = 'my_opencv_tutorials'
    pkg_share = get_package_share_directory(package_name)

    # 2. YAML 파일의 기본 경로 설정 (/config/cam.yaml)
    default_params_file = os.path.join(pkg_share, 'config', 'cam.yaml')

    # 3. 런치 인자 선언 (<arg name="params_file" default="..." />)
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the config file to use'
    )

    # 4. 노드 설정 (<node ... > <param from="..." /> </node>)
    camera_node = Node(
        package=package_name,
        executable='img_pub_param_node',
        name='img_pub_node',
        output='screen',
        # 파라미터 파일 로드
        parameters=[LaunchConfiguration('params_file')]
    )

    # # 4. 이미지 압축 노드 (Raw -> Compressed 변환)
    # republish_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='img_republisher',
    #     arguments=['raw', 'compressed'],
    #     remappings=[
    #         # 입력: 카메라 노드가 발행하는 토픽 이름 (/image_raw)
    #         ('in', '/image_raw'),
    #         # 출력: 압축된 토픽 이름 (/image_raw/compressed)
    #         ('out/compressed', '/image_raw/compressed')
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        params_file_arg,
        camera_node,
        # republish_node
    ])