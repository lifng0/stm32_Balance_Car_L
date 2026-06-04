from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted


class LidarAvoidNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_avoid_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("warning_distance", 0.80)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("turn_clear_distance", 0.70)
        self.declare_parameter("forward_speed", 8.0)
        self.declare_parameter("slow_speed", 4.0)
        self.declare_parameter("turn_speed", 8.0)

        self.adapter = BackendControllerAdapter(
            self.get_parameter("backend_host").get_parameter_value().string_value,
            self.get_parameter("backend_port").get_parameter_value().integer_value,
        )
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.turn_clear_distance = self.get_parameter("turn_clear_distance").get_parameter_value().double_value
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.slow_speed = self.get_parameter("slow_speed").get_parameter_value().double_value
        self.turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value

        self.system_state = {}
        self.last_command = None

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/lidar/summary_json", self.on_lidar, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_lidar(self, msg: String) -> None:
        lidar = decode_json_message(msg.data)
        if not is_system_permitted(self.system_state) or not lidar.get("scan_ok"):
            self.issue_stop("system_not_ready_or_scan_invalid")
            return

        front = float(lidar.get("front_min_distance_m") or 0.0)
        left = float(lidar.get("front_left_min_distance_m") or 0.0)
        right = float(lidar.get("front_right_min_distance_m") or 0.0)

        if front > self.warning_distance:
            self.issue_move(self.forward_speed, 0.0, "forward")
            return

        if front > self.stop_distance:
            self.issue_move(self.slow_speed, 0.0, "slow_forward")
            return

        turn = self.turn_speed
        if left >= max(right, self.turn_clear_distance):
            self.issue_move(0.0, -turn, "turn_left")
        elif right >= max(left, self.turn_clear_distance):
            self.issue_move(0.0, turn, "turn_right")
        elif left >= right:
            self.issue_move(0.0, -turn, "turn_left_tight")
        else:
            self.issue_move(0.0, turn, "turn_right_tight")

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
        move_x = clamp(move_x, -30.0, 30.0)
        move_z = clamp(move_z, -30.0, 30.0)
        command = ("move", round(move_x, 3), round(move_z, 3), reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(move_x, move_z)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"avoid command failed: {exc}")

    def issue_stop(self, reason: str) -> None:
        command = ("stop", reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(0.0, 0.0)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"avoid stop failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
