from setuptools import find_packages, setup

package_name = "balance_car_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/navigation.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lifng0",
    maintainer_email="lifng0@example.com",
    description="ROS 2 navigation nodes for lidar avoidance and target following.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_avoid_node = balance_car_navigation.lidar_avoid_node:main",
            "lidar_follow_node = balance_car_navigation.lidar_follow_node:main",
        ],
    },
)
