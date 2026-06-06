from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    task = LaunchConfiguration("task")
    return LaunchDescription(
        [
            DeclareLaunchArgument("task", default_value="avoid"),
            DeclareLaunchArgument("backend_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("backend_port", default_value="8765"),
            Node(
                package="balance_car_navigation",
                executable="lidar_avoid_node",
                name="lidar_avoid_node",
                condition=IfCondition(PythonExpression(["'", task, "' == 'avoid'"])),
                parameters=[{"backend_host": LaunchConfiguration("backend_host"), "backend_port": LaunchConfiguration("backend_port")}],
            ),
            Node(
                package="balance_car_navigation",
                executable="lidar_follow_node",
                name="lidar_follow_node",
                condition=IfCondition(PythonExpression(["'", task, "' == 'follow'"])),
                parameters=[{"backend_host": LaunchConfiguration("backend_host"), "backend_port": LaunchConfiguration("backend_port")}],
            ),
            Node(
                package="balance_car_navigation",
                executable="lidar_track_node",
                name="lidar_track_node",
                condition=IfCondition(PythonExpression(["'", task, "' == 'track'"])),
                parameters=[{"backend_host": LaunchConfiguration("backend_host"), "backend_port": LaunchConfiguration("backend_port")}],
            ),
        ]
    )
