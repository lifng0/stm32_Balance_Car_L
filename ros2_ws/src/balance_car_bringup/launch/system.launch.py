from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    enable_navigation = LaunchConfiguration("enable_navigation")
    navigation_task = LaunchConfiguration("navigation_task")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_navigation", default_value="false"),
            DeclareLaunchArgument("navigation_task", default_value="avoid"),
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
            Node(
                package="balance_car_navigation",
                executable="vision_line_node",
                name="vision_line_node",
                condition=IfCondition(enable_navigation),
                parameters=[{"backend_host": "127.0.0.1", "backend_port": 8765}],
            ),
            Node(
                package="balance_car_navigation",
                executable="vision_follow_node",
                name="vision_follow_node",
                condition=IfCondition(enable_navigation),
                parameters=[{"backend_host": "127.0.0.1", "backend_port": 8765}],
            ),
            Node(
                package="balance_car_bringup",
                executable="ros_ready_node",
                name="balance_car_ros_ready",
                parameters=[
                    {"backend_host": "127.0.0.1", "backend_port": 8765},
                    {
                        "required_nodes": [
                            "/balance_car_bridge",
                            "/balance_car_lidar_summary",
                            "/balance_car_task_manager",
                            "/balance_car_k210_parser",
                            "/lidar_follow_node",
                            "/balance_car_ros_ready",
                        ]
                    },
                ],
            ),
            Node(
                package="balance_car_navigation",
                executable="lidar_avoid_node",
                name="lidar_avoid_node",
                condition=IfCondition(
                    PythonExpression(["'", enable_navigation, "' == 'true' and '", navigation_task, "' == 'avoid'"])
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "/workspaces/balance_car/ws/install/balance_car_navigation/lib/balance_car_navigation/lidar_follow_node",
                    "--ros-args",
                    "-r",
                    "__node:=lidar_follow_node",
                    "-p",
                    "backend_host:=127.0.0.1",
                    "-p",
                    "backend_port:=8765",
                ],
                output="screen",
            ),
        ]
    )
