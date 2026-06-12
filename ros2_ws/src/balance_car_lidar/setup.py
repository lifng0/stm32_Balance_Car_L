from setuptools import find_packages, setup

package_name = "balance_car_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lifng0",
    maintainer_email="lifng0@example.com",
    description="ROS 2 lidar package for T-mini Plus summary publishing.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tminiplus_node = balance_car_lidar.tminiplus_node:main",
            "lidar_summary_node = balance_car_lidar.lidar_summary_node:main",
        ],
    },
)
