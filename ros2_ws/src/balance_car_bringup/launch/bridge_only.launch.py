from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="balance_car_bridge",
                executable="bridge_node",
                name="balance_car_bridge",
                parameters=[{"backend_host": "127.0.0.1", "backend_port": 8765}],
            ),
        ]
    )
