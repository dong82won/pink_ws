from setuptools import find_packages, setup
import os

# # case 1
# import glob

# case 2
from glob import glob # glob 함수를 직접 import

package_name = 'my_opencv_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # # case 1
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*'))),

        # case 2
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
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
        'img_pub = my_opencv_tutorials.img_pub:main',
        'img_pub_param1 = my_opencv_tutorials.img_pub_param:main',
        'img_pub_param2 = my_opencv_tutorials.img_pub_param2:main',
        'cartoon_node = my_opencv_tutorials.cartoon:main',
        'cartoon_multi_node = my_opencv_tutorials.cartoon_styler_multi:main',
        ],
    },
)
