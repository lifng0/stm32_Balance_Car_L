from setuptools import find_packages, setup

package_name = "balance_car_bridge"

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
    description="ROS 2 bridge package for balance car host services.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = balance_car_bridge.bridge_node:main",
        ],
    },
)
