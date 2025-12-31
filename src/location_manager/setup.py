from setuptools import setup
import os
from glob import glob

package_name = 'location_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installer le fichier de config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Douaa ZAKI',
    maintainer_email='your_email@example.com',
    description='Location Manager pour warehouse autonome',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'location_manager_node = location_manager.location_manager_node:main',
            'test_location_manager = location_manager.test_location_manager:main',
        ],
    },
)