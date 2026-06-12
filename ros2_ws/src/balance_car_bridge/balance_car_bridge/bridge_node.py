import json
import time

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
        self.declare_parameter("state_poll_period", 0.5)
        self.declare_parameter("control_topic", "/cmd_vel_bl")
        self.declare_parameter("control_flush_period", 0.02)
        self.declare_parameter("control_timeout_sec", 0.30)
        self.declare_parameter("control_refresh_sec", 0.10)
        self.declare_parameter("cmd_vel_deadband", 0.01)

        self.backend_host = self.get_parameter("backend_host").get_parameter_value().string_value
        self.backend_port = self.get_parameter("backend_port").get_parameter_value().integer_value
        self.state_poll_period = self.get_parameter("state_poll_period").get_parameter_value().double_value
        self.control_topic = self.get_parameter("control_topic").get_parameter_value().string_value
        self.control_flush_period = self.get_parameter("control_flush_period").get_parameter_value().double_value
        self.control_timeout_sec = self.get_parameter("control_timeout_sec").get_parameter_value().double_value
        self.control_refresh_sec = self.get_parameter("control_refresh_sec").get_parameter_value().double_value
        self.cmd_vel_deadband = self.get_parameter("cmd_vel_deadband").get_parameter_value().double_value

        self.state_pub = self.create_publisher(String, "/car/state_json", 10)
        self.system_pub = self.create_publisher(String, "/car/system_state_json", 10)
        self.event_pub = self.create_publisher(String, "/car/event_json", 10)
        self.control_pub = self.create_publisher(String, "/car/control_json", 10)
        self.cmd_sub = self.create_subscription(Twist, self.control_topic, self.on_cmd_vel, 1)
        self.state_timer = self.create_timer(self.state_poll_period, self.poll_backend)
        self.control_timer = self.create_timer(self.control_flush_period, self.flush_control)

        self.last_event_counter = -1
        self.last_control_counter = -1
        self.last_car_state: dict = {}
        self.last_system_mode = "booting"
        self.pending_move = (0.0, 0.0)
        self.pending_move_at = 0.0
        self.last_sent_move = None
        self.last_sent_at = 0.0
        self.last_control_error = ""

    def backend_request(self, payload: dict, timeout: float = 1.0) -> dict:
        return request_backend(self.backend_host, self.backend_port, payload, timeout=timeout)

    def on_cmd_vel(self, msg: Twist) -> None:
        move_x = float(msg.linear.x)
        move_z = float(msg.angular.z)
        if abs(move_x) < self.cmd_vel_deadband:
            move_x = 0.0
        if abs(move_z) < self.cmd_vel_deadband:
            move_z = 0.0
        self.pending_move = (move_x, move_z)
        self.pending_move_at = time.time()

    def flush_control(self) -> None:
        now = time.time()
        if self.last_system_mode != "running":
            self.last_sent_move = None
            return
        if self.last_car_state.get("stop_flag", True):
            target = (0.0, 0.0)
        elif now - self.pending_move_at > self.control_timeout_sec:
            target = (0.0, 0.0)
        else:
            target = self.pending_move

        if target == self.last_sent_move and now - self.last_sent_at < self.control_refresh_sec:
            return
        try:
            response = self.backend_request(
                {
                    "cmd": "set_move",
                    "move_x": target[0],
                    "move_z": target[1],
                    "source": "ros_bridge",
                },
                timeout=0.2,
            )
        except Exception as exc:
            message = str(exc)
            if message != self.last_control_error:
                self.get_logger().warning(f"control backend request failed: {exc}")
                self.last_control_error = message
            return

        if not response.get("ok"):
            message = json.dumps(response, ensure_ascii=False)
            if message != self.last_control_error:
                self.get_logger().warning(f"control command rejected by backend: {response}")
                self.last_control_error = message
            return

        self.last_control_error = ""
        self.last_sent_move = target
        self.last_sent_at = now
        car_state = response.get("car_state") or {}
        if car_state:
            self.last_car_state = car_state

    def poll_backend(self) -> None:
        try:
            snapshot = self.backend_request({"cmd": "get_state"}, timeout=2.0)
        except Exception as exc:
            self.get_logger().warning(f"backend poll failed: {exc}")
            return

        self.last_car_state = snapshot.get("car_state") or {}
        self.last_system_mode = str(snapshot.get("system_mode", self.last_system_mode) or self.last_system_mode)
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
