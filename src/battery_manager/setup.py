from setuptools import setup

package_name = 'battery_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='douaa',
    maintainer_email='douaa@todo.todo',
    description='Battery management system for warehouse robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_manager_node = battery_manager.battery_manager_node:main',
        ],
    },
)