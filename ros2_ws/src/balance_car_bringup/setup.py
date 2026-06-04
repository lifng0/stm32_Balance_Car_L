from glob import glob
from setuptools import find_packages, setup

package_name = "balance_car_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lifng0",
    maintainer_email="lifng0@example.com",
    description="Launch and configuration package for the balance car ROS 2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ros_ready_node = balance_car_bringup.ros_ready_node:main",
        ],
    },
)
