import json
import socket
import time

import rclpy
from balance_car_interfaces.msg import VisionTarget
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
        self.declare_parameter("backend_timeout", 1.0)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value
        self.backend_timeout = self.get_parameter("backend_timeout").get_parameter_value().double_value

        self.vision_pub = self.create_publisher(String, "/vision/status_json", 10)
        self.target_pub = self.create_publisher(VisionTarget, "/vision/target", 10)
        self.link_pub = self.create_publisher(String, "/vision/k210_link_json", 10)
        self.command_result_pub = self.create_publisher(String, "/vision/k210_command_result_json", 10)
        self.command_sub = self.create_subscription(String, "/vision/k210_command", self.on_command, 10)
        self.timer = self.create_timer(self.poll_period, self.poll_backend)

    def backend_request(self, payload: dict, timeout: float | None = None) -> dict:
        return request_backend(
            self.backend_host,
            self.backend_port,
            payload,
            timeout=self.backend_timeout if timeout is None else timeout,
        )

    def publish_json(self, publisher, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)

    def publish_target(self, payload: dict) -> None:
        msg = VisionTarget()
        msg.target_type = str(payload.get("target_type") or "")
        msg.detected = bool(payload.get("valid", False) and payload.get("vision_type") == 2)
        msg.x = float(payload.get("x", 0.0) or 0.0)
        msg.y = float(payload.get("y", 0.0) or 0.0)
        msg.confidence = float(payload.get("area", 0.0) or 0.0)
        msg.extra = json.dumps(
            {
                "mode": payload.get("mode"),
                "mode_name": payload.get("mode_name"),
                "w": payload.get("w"),
                "h": payload.get("h"),
                "area": payload.get("area"),
                "raw": payload.get("raw"),
                "text": payload.get("text"),
                "color_name": payload.get("color_name"),
            },
            ensure_ascii=False,
        )
        self.target_pub.publish(msg)

    def on_command(self, msg: String) -> None:
        text = msg.data
        response: dict
        try:
            response = self.backend_request(
                {"cmd": "set_k210_text", "text": text},
                timeout=max(self.backend_timeout, 1.0),
            )
        except Exception as exc:
            response = {
                "ok": False,
                "error": "backend_request_failed",
                "message": str(exc),
                "text": text,
                "timestamp": time.time(),
            }
            self.get_logger().warning(f"k210 command failed: {exc}")

        self.publish_json(self.command_result_pub, response)

    def poll_backend(self) -> None:
        try:
            snapshot = self.backend_request({"cmd": "get_k210_link"}, timeout=max(self.backend_timeout, 1.5))
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
        self.publish_json(self.vision_pub, payload)
        self.publish_target(payload)

        link_payload = {
            "k210_device": snapshot.get("k210_device"),
            "k210_port": snapshot.get("k210_port"),
            "k210_ready": snapshot.get("k210_ready"),
            "k210_last_error": snapshot.get("k210_last_error"),
            "k210_last_rx_at": snapshot.get("k210_last_rx_at"),
            "vision_source": (payload or {}).get("source"),
            "vision_valid": (payload or {}).get("valid"),
            "timestamp": snapshot.get("timestamp"),
        }
        self.publish_json(self.link_pub, link_payload)


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
