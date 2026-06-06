from setuptools import find_packages, setup

package_name = 'balance_car_navigation'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Lidar Navigation & Tracking Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_avoid_node = balance_car_navigation.lidar_avoid_node:main',
            'lidar_track_node = balance_car_navigation.lidar_track_node:main',
            'lidar_follow_node = balance_car_navigation.lidar_follow_node:main',
            'vision_follow_node = balance_car_navigation.vision_follow_node:main',
        ],
    },
)
