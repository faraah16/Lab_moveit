from setuptools import setup
import os
from glob import glob

package_name = 'mission_orchestrator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '/missions'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Douaa ZAKI',
    maintainer_email='your_email@example.com',
    description='Mission Orchestrator pour warehouse autonome',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_orchestrator = mission_orchestrator.mission_orchestrator_node:main',
            'navigation_client = mission_orchestrator.navigation_client:main',
            'test_mission = mission_orchestrator.test_mission:main',
        ],
    },
)