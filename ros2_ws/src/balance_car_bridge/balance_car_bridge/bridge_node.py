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
        self.declare_parameter("cmd_vel_linear_scale", 10.0)
        self.declare_parameter("cmd_vel_angular_scale", 10.0)
        self.declare_parameter("cmd_vel_deadband", 0.01)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value
        self.cmd_vel_linear_scale = self.get_parameter("cmd_vel_linear_scale").get_parameter_value().double_value
        self.cmd_vel_angular_scale = self.get_parameter("cmd_vel_angular_scale").get_parameter_value().double_value
        self.cmd_vel_deadband = self.get_parameter("cmd_vel_deadband").get_parameter_value().double_value

        self.state_pub = self.create_publisher(String, "/car/state_json", 10)
        self.system_pub = self.create_publisher(String, "/car/system_state_json", 10)
        self.event_pub = self.create_publisher(String, "/car/event_json", 10)
        self.control_pub = self.create_publisher(String, "/car/control_json", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.timer = self.create_timer(self.poll_period, self.poll_backend)

        self.last_event_counter = -1
        self.last_control_counter = -1
        self.last_car_state: dict = {}

    def backend_request(self, payload: dict, timeout: float = 1.0) -> dict:
        return request_backend(self.backend_host, self.backend_port, payload, timeout=timeout)

    def on_cmd_vel(self, msg: Twist) -> None:
        if self.last_car_state.get("stop_flag", True):
            self.get_logger().warning("cmd_vel ignored because stop_flag is asserted by stm32")
            return

        move_x = float(msg.linear.x) * self.cmd_vel_linear_scale
        move_z = float(msg.angular.z) * self.cmd_vel_angular_scale
        if abs(move_x) < self.cmd_vel_deadband:
            move_x = 0.0
        if abs(move_z) < self.cmd_vel_deadband:
            move_z = 0.0

        try:
            response = self.backend_request(
                {"cmd": "set_move", "move_x": move_x, "move_z": move_z},
                timeout=1.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"cmd_vel backend request failed: {exc}")
            return

        if not response.get("ok"):
            self.get_logger().warning(f"cmd_vel rejected by backend: {response}")

    def poll_backend(self) -> None:
        try:
            snapshot = self.backend_request({"cmd": "get_state"}, timeout=2.0)
        except Exception as exc:
            self.get_logger().warning(f"backend poll failed: {exc}")
            return

        self.last_car_state = snapshot.get("car_state") or {}
        state_msg = String()
        state_msg.data = json.dumps(self.last_car_state, ensure_ascii=False)
        self.state_pub.publish(state_msg)

        system_state = {
            "system_mode": snapshot.get("system_mode"),
            "lidar_enabled": snapshot.get("lidar_enabled"),
            "lidar_required": snapshot.get("lidar_required"),
            "ros_ready": snapshot.get("ros_ready"),
            "ros_ready_reason": snapshot.get("ros_ready_reason"),
            "pi_ready": snapshot.get("pi_ready"),
            "lidar_ready": snapshot.get("lidar_ready"),
            "system_ready": snapshot.get("system_ready"),
            "paused_by_pickup": snapshot.get("paused_by_pickup"),
            "shutdown_started": snapshot.get("shutdown_started"),
            "stop_flag": bool(self.last_car_state.get("stop_flag", True)),
            "backend_port": snapshot.get("backend_port"),
            "control_backend_ready": snapshot.get("control_backend_ready"),
        }
        system_msg = String()
        system_msg.data = json.dumps(system_state, ensure_ascii=False)
        self.system_pub.publish(system_msg)

        control_counter = snapshot.get("control_counter", -1)
        if control_counter != self.last_control_counter:
            control_msg = String()
            control_msg.data = json.dumps(snapshot.get("control_state") or {}, ensure_ascii=False)
            self.control_pub.publish(control_msg)
            self.last_control_counter = control_counter

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
