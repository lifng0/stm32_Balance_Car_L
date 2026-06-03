import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def request_backend(host: str, port: int, payload: dict, timeout: float = 0.5) -> dict:
    message = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(message)
        sock.shutdown(socket.SHUT_WR)
        chunks = []
        while True:
            data = sock.recv(4096)
            if not data:
                break
            chunks.append(data)

    raw = b"".join(chunks).decode("utf-8").strip()
    if not raw:
        raise RuntimeError("empty backend response")
    return json.loads(raw)


class K210ParserNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_k210_parser")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("poll_period", 0.5)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value
        self.pub = self.create_publisher(String, "/vision/status_json", 10)
        self.timer = self.create_timer(self.poll_period, self.publish_vision_state)

    def publish_vision_state(self) -> None:
        try:
            snapshot = request_backend(
                self.backend_host,
                self.backend_port,
                {"cmd": "get_vision"},
                timeout=1.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"backend poll failed: {exc}")
            return

        payload = snapshot.get("vision_state") or {
            "source": "k210",
            "vision_type": 0,
            "mode_name": "unknown",
            "valid": False,
            "text": "",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K210ParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
