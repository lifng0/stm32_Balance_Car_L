import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .backend_client import request_backend


class BalanceCarBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_bridge")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("poll_period", 0.5)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value

        self.state_pub = self.create_publisher(String, "/car/state_json", 10)
        self.system_pub = self.create_publisher(String, "/car/system_state_json", 10)
        self.event_pub = self.create_publisher(String, "/car/event_json", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.timer = self.create_timer(self.poll_period, self.poll_backend)

        self.last_event_counter = -1
        self.cmd_warned = False

    def on_cmd_vel(self, msg: Twist) -> None:
        if not self.cmd_warned:
            self.get_logger().warning(
                "cmd_vel received, but velocity forwarding is not wired yet. "
                "Current ROS skeleton is state-first."
            )
            self.cmd_warned = True
        _ = msg

    def poll_backend(self) -> None:
        try:
            snapshot = request_backend(
                self.backend_host,
                self.backend_port,
                {"cmd": "get_state"},
                timeout=2.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"backend poll failed: {exc}")
            return

        state_msg = String()
        state_msg.data = json.dumps(snapshot.get("car_state") or {}, ensure_ascii=False)
        self.state_pub.publish(state_msg)

        system_state = {
            "system_mode": snapshot.get("system_mode"),
            "lidar_enabled": snapshot.get("lidar_enabled"),
            "lidar_required": snapshot.get("lidar_required"),
            "pi_ready": snapshot.get("pi_ready"),
            "lidar_ready": snapshot.get("lidar_ready"),
            "system_ready": snapshot.get("system_ready"),
            "paused_by_pickup": snapshot.get("paused_by_pickup"),
            "shutdown_started": snapshot.get("shutdown_started"),
            "backend_port": snapshot.get("backend_port"),
        }
        system_msg = String()
        system_msg.data = json.dumps(system_state, ensure_ascii=False)
        self.system_pub.publish(system_msg)

        event_counter = snapshot.get("event_counter", -1)
        if event_counter != self.last_event_counter:
            event_payload = {
                "event_counter": event_counter,
                "event_code": snapshot.get("last_event_code", 0),
                "event_name": snapshot.get("last_event_name", "UNKNOWN"),
            }
            event_msg = String()
            event_msg.data = json.dumps(event_payload, ensure_ascii=False)
            self.event_pub.publish(event_msg)
            self.last_event_counter = event_counter


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalanceCarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
