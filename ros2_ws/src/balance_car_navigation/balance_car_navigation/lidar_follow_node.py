from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted

MODE_LIDAR_FOLLOW = 8


class LidarFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_follow_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("target_distance", 0.70)
        self.declare_parameter("distance_tolerance", 0.12)
        self.declare_parameter("max_forward_speed", 8.0)
        self.declare_parameter("max_turn_speed", 8.0)
        self.declare_parameter("turn_gain", 0.18)
        self.declare_parameter("forward_gain", 12.0)
        self.declare_parameter("max_track_angle_deg", 60.0)

        self.adapter = BackendControllerAdapter(
            self.get_parameter("backend_host").get_parameter_value().string_value,
            self.get_parameter("backend_port").get_parameter_value().integer_value,
        )
        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter("distance_tolerance").get_parameter_value().double_value
        self.max_forward_speed = self.get_parameter("max_forward_speed").get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter("max_turn_speed").get_parameter_value().double_value
        self.turn_gain = self.get_parameter("turn_gain").get_parameter_value().double_value
        self.forward_gain = self.get_parameter("forward_gain").get_parameter_value().double_value
        self.max_track_angle_deg = self.get_parameter("max_track_angle_deg").get_parameter_value().double_value

        self.system_state = {}
        self.car_state = {}
        self.last_command = None

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(String, "/lidar/summary_json", self.on_lidar, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_car_state(self, msg: String) -> None:
        self.car_state = decode_json_message(msg.data)

    def mode_permitted(self) -> bool:
        return int(self.car_state.get("mode", -1) or -1) == MODE_LIDAR_FOLLOW

    def on_lidar(self, msg: String) -> None:
        lidar = decode_json_message(msg.data)
        if not self.mode_permitted():
            self.issue_stop("mode_not_lidar_follow")
            return
        if not is_system_permitted(self.system_state) or not lidar.get("scan_ok"):
            self.issue_stop("system_not_ready_or_scan_invalid")
            return

        distance = lidar.get("closest_target_distance_m")
        angle = lidar.get("closest_target_angle_deg")
        if distance is None or angle is None:
            self.issue_stop("no_target")
            return

        distance = float(distance)
        angle = float(angle)
        if distance <= 0.0 or abs(angle) > self.max_track_angle_deg:
            self.issue_stop("target_out_of_range")
            return

        distance_error = distance - self.target_distance
        if abs(distance_error) <= self.distance_tolerance:
            move_x = 0.0
        else:
            move_x = clamp(distance_error * self.forward_gain, -self.max_forward_speed, self.max_forward_speed)
        move_z = clamp(angle * self.turn_gain, -self.max_turn_speed, self.max_turn_speed)
        self.issue_move(move_x, move_z, "track_target")

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
        command = ("move", round(move_x, 3), round(move_z, 3), reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(move_x, move_z)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"follow command failed: {exc}")

    def issue_stop(self, reason: str) -> None:
        command = ("stop", reason)
        if command == self.last_command:
            return
        move_x = float(self.car_state.get("move_x", 0.0) or 0.0)
        move_z = float(self.car_state.get("move_z", 0.0) or 0.0)
        if abs(move_x) < 1e-3 and abs(move_z) < 1e-3 and self.system_state.get("system_mode") != "running":
            self.last_command = command
            return
        try:
            self.adapter.set_move(0.0, 0.0)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"follow stop failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
