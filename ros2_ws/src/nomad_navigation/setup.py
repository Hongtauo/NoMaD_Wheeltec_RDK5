"""Install the Python package, ROS index marker, configuration and launch files."""

from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'nomad_navigation'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SocialEmbodiedAgent maintainers',
    maintainer_email='maintainers@example.com',
    description='ROS 2 NoMaD inference and preview-only waypoint velocity control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smoke_node = nomad_navigation.smoke_node:main',
            'video_publisher = nomad_navigation.video_publisher:main',
            'nomad_node = nomad_navigation.nomad_node:main',
            'waypoint_controller = nomad_navigation.controller_node:main',
            'nomad_visualization = nomad_navigation.visualization_node:main',
            'wait_for_nomad = nomad_navigation.wait_for_model:main',
            'robot_control = nomad_navigation.robot_control_node:main',
            'robot_gateway = nomad_navigation.robot_gateway:main',
            'campus_api_server = nomad_navigation.campus_api_server:main',
            'campus_api_client = nomad_navigation.campus_api_client:main',
        ],
    },
)
