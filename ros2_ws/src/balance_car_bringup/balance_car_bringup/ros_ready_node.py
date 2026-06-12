import rclpy
from rclpy.node import Node

from .backend_client import request_backend


class RosReadyNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_ros_ready")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter(
            "required_nodes",
            [
                "/balance_car_bridge",
                "/balance_car_lidar",
                "/balance_car_task_manager",
                "/balance_car_k210_parser",
                "/balance_car_ros_ready",
            ],
        )
        self.declare_parameter("heartbeat_period", 1.0)
        self.declare_parameter("missing_reason_prefix", "missing_nodes")

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.required_nodes = list(self.get_parameter("required_nodes").value)
        self.heartbeat_period = self.get_parameter("heartbeat_period").get_parameter_value().double_value
        self.missing_reason_prefix = self.get_parameter("missing_reason_prefix").get_parameter_value().string_value

        self.timer = self.create_timer(self.heartbeat_period, self.publish_ready_state)
        self.last_reason = ""

    def publish_ready_state(self) -> None:
        active_nodes = sorted(
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        )
        missing = [node for node in self.required_nodes if node not in active_nodes]
        ready = not missing
        reason = "ready" if ready else f"{self.missing_reason_prefix}:{','.join(missing)}"
        payload = {
            "cmd": "set_ros_ready",
            "ready": ready,
            "reason": reason,
            "nodes": active_nodes,
            "required_nodes": self.required_nodes,
        }
        try:
            request_backend(self.backend_host, self.backend_port, payload, timeout=1.0)
        except Exception as exc:
            self.get_logger().warning(f"failed to report ros readiness: {exc}")
            return

        if reason != self.last_reason:
            self.get_logger().info(f"ros readiness={ready} reason={reason}")
            self.last_reason = reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosReadyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
