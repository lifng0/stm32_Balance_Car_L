import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def request_backend(host: str, port: int, payload: dict, timeout: float = 0.5) -> dict:
    with socket.create_connection((host, port), timeout=timeout) as conn:
        conn.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
        raw = conn.recv(65535)
    if not raw:
        raise RuntimeError("backend closed connection")
    return json.loads(raw.decode("utf-8"))


class LidarSummaryNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_lidar_summary")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("poll_period", 0.5)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value

        self.summary_pub = self.create_publisher(String, "/lidar/summary_json", 10)
        self.timer = self.create_timer(self.poll_period, self.publish_summary)
        self.warned_not_ready = False

    def publish_summary(self) -> None:
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

        if not snapshot.get("lidar_enabled", False):
            self.warned_not_ready = False
            return

        summary = snapshot.get("lidar_summary") or {}
        if not summary:
            if not self.warned_not_ready:
                self.get_logger().warning("lidar summary not ready from backend yet")
                self.warned_not_ready = True
            return

        summary["device"] = snapshot.get("lidar_port", "")
        summary["scan_ok"] = bool(snapshot.get("lidar_ready", False))
        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=False)
        self.summary_pub.publish(msg)
        self.warned_not_ready = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSummaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
