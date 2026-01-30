from setuptools import find_packages, setup

package_name = 'my_tf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='won',
    maintainer_email='2dongwon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'my_tf_node1 = my_tf_pkg.my_tf_1:main',
        'my_tf_node2 = my_tf_pkg.my_tf_2:main',
        'tf_child_frame_node = my_tf_pkg.child_frame:main',
        'tf_combined_frame_node = my_tf_pkg.combined_frame:main',
        'tf_distance_frame_node = my_tf_pkg.distance_world_child_publisher:main',
        'tf_marker_trail_node = my_tf_pkg.frame_trail_publisher_marker:main',
        'tf_marker_trail_path_node = my_tf_pkg.frame_trail_publisher_marker_nav:main',
        'tf_marker_trail_path_node2 = my_tf_pkg.frame_trail_publisher_marker_nav2:main',


        ],
    },
)
