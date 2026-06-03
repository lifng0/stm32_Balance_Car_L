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
            Node(
                package="balance_car_lidar",
                executable="lidar_summary_node",
                name="balance_car_lidar_summary",
            ),
            Node(
                package="balance_car_behavior",
                executable="task_manager_node",
                name="balance_car_task_manager",
            ),
            Node(
                package="balance_car_perception",
                executable="k210_parser_node",
                name="balance_car_k210_parser",
            ),
        ]
    )
