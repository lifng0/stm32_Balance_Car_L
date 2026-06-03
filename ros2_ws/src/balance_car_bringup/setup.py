from glob import glob
from setuptools import setup

package_name = "balance_car_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
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
)
