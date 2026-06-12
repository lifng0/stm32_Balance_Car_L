import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 统一配置真值源:modes.yaml / devices.yaml 安装在 balance_car_bringup/share/config
_CONFIG_DIR = os.path.join(get_package_share_directory("balance_car_bringup"), "config")
MODES_CONFIG = os.path.join(_CONFIG_DIR, "modes.yaml")
DEVICES_CONFIG = os.path.join(_CONFIG_DIR, "devices.yaml")


def _lidar_params_from_devices():
    """从 devices.yaml 读取雷达设备参数;失败则返回空 dict 让节点走自动探测。"""
    try:
        with open(DEVICES_CONFIG, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        lidar = (data.get("lidar", {}) or {}).get("ros__parameters", {}) or {}
        params = {}
        if lidar.get("device"):
            params["device"] = str(lidar["device"])
        if lidar.get("baudrate"):
            params["baudrate"] = int(lidar["baudrate"])
        if lidar.get("scan_frequency"):
            params["scan_frequency"] = float(lidar["scan_frequency"])
        return params
    except Exception:  # noqa: BLE001 - 配置缺失时回退到节点自动探测
        return {}


def generate_launch_description():
    lidar_params = _lidar_params_from_devices()
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_navigation", default_value="false"),
            DeclareLaunchArgument("navigation_task", default_value="avoid"),
            DeclareLaunchArgument("use_native_lidar", default_value="true"),
            Node(
                package="balance_car_bridge",
                executable="control_mux_node",
                name="balance_car_control_mux",
                parameters=[{"modes_config": MODES_CONFIG}],
            ),
            Node(
                package="balance_car_bridge",
                executable="bridge_node",
                name="balance_car_bridge",
                parameters=[{"backend_host": "127.0.0.1", "backend_port": 8765}],
            ),
            Node(
                package="balance_car_lidar",
                executable="tminiplus_node",
                name="balance_car_lidar",
                parameters=[lidar_params] if lidar_params else [],
                condition=IfCondition(LaunchConfiguration("use_native_lidar")),
            ),
            Node(
                package="balance_car_lidar",
                executable="lidar_summary_node",
                name="balance_car_lidar",
                condition=UnlessCondition(LaunchConfiguration("use_native_lidar")),
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
                package="balance_car_bringup",
                executable="ros_ready_node",
                name="balance_car_ros_ready",
                parameters=[
                    {"backend_host": "127.0.0.1", "backend_port": 8765},
                    {
                        "required_nodes": [
                            "/balance_car_control_mux",
                            "/balance_car_bridge",
                            "/balance_car_lidar",
                            "/balance_car_task_manager",
                            "/balance_car_k210_parser",
                            "/lidar_avoid_node",
                            "/lidar_follow_node",
                            "/balance_car_ros_ready",
                        ]
                    },
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "/workspaces/balance_car/ws/install/balance_car_navigation/lib/balance_car_navigation/lidar_avoid_node",
                    "--ros-args",
                    "-r",
                    "__node:=lidar_avoid_node",
                    "-p",
                    "backend_host:=127.0.0.1",
                    "-p",
                    "backend_port:=8765",
                ],
                output="screen",
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
