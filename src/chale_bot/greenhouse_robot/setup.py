from setuptools import setup
from glob import glob
import os

package_name = 'greenhouse_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Greenhouse fruit-harvesting robot (detection + FSM + arm control).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fruit_detector    = greenhouse_robot.fruit_detector:main',
            'robot_coordinator = greenhouse_robot.robot_coordinator:main',
            'arm_controller    = greenhouse_robot.arm_controller:main',
        ],
    },
)
