from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='douaa',
    maintainer_email='douaa@todo.todo',
    description='Warehouse stock management system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stock_manager_node = warehouse_manager.stock_manager_node:main',
	        'employee_interface = warehouse_manager.employee_interface_node:main', 
            'mission_queue_manager = warehouse_manager.mission_queue_manager_node:main',
        ],
    },
)
