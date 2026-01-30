from setuptools import find_packages, setup

package_name = 'controller_tutorials_pkg'

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
        'simple_rotate = controller_tutorials_pkg.simple_rotate:main',

        'control_rotate = controller_tutorials_pkg.control_rotate:main',

        'pose_dual_controller = controller_tutorials_pkg.pose_dual_controller:main',
        'qmonitor_for_pose_dual_controller = controller_tutorials_pkg.qmonitor_for_pose_dual_controller:main',

        'move_turtle = controller_tutorials_pkg.move_turtle:main',
        'monitor_for_move_turtle = controller_tutorials_pkg.monitor_for_move_turtle:main',

        'move_turtle_state_machine = controller_tutorials_pkg.move_turtle_state_machine:main',

        'qmonitor_state_machine = controller_tutorials_pkg.qmonitor_state_machine:main',


        'monitor_for_pose_dual_controller = controller_tutorials_pkg.p_monitor_for_pose_dual_controller:main',
        'move_turtle_behavior_tree = controller_tutorials_pkg.p_move_turtle_behavior_tree:main',
        'web_publisher_node = controller_tutorials_pkg.p_web_publisher_node:main',


        ],
    },
)
