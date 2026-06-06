from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted


class LidarTrackNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_track_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("max_detect_distance", 1.50)
        self.declare_parameter("stop_distance_insurance", 0.20)
        self.declare_parameter("distance_deadband", 0.05)
        self.declare_parameter("turn_deadband", 0.05)
        self.declare_parameter("max_forward_speed", 6.0)
        self.declare_parameter("max_turn_speed", 5.0)

        self.adapter = BackendControllerAdapter(
            self.get_parameter("backend_host").get_parameter_value().string_value,
            self.get_parameter("backend_port").get_parameter_value().integer_value,
        )
        self.max_detect_distance = self.get_parameter("max_detect_distance").get_parameter_value().double_value
        self.stop_distance_insurance = self.get_parameter("stop_distance_insurance").get_parameter_value().double_value
        self.distance_deadband = self.get_parameter("distance_deadband").get_parameter_value().double_value
        self.turn_deadband = self.get_parameter("turn_deadband").get_parameter_value().double_value
        self.max_forward_speed = self.get_parameter("max_forward_speed").get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter("max_turn_speed").get_parameter_value().double_value

        self.system_state = {}
        self.last_command = None
        self.target_distance = None
        self.lost_counter = 0

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/lidar/summary_json", self.on_lidar, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_lidar(self, msg: String) -> None:
        lidar = decode_json_message(msg.data)
        if not is_system_permitted(self.system_state) or not lidar.get("scan_ok"):
            self.issue_stop("system_not_ready_or_scan_invalid")
            return

        front = lidar.get("front_min_distance_m")
        front_left = lidar.get("front_left_min_distance_m")
        front_right = lidar.get("front_right_min_distance_m")
        if front is None or front_left is None or front_right is None:
            self.issue_stop("no_lidar_data")
            return

        front = float(front)
        front_left = float(front_left)
        front_right = float(front_right)

        if front > self.max_detect_distance:
            self.lost_counter += 1
            if self.lost_counter >= 20:
                self.issue_stop("lost_2s_timeout")
            return

        self.lost_counter = 0

        if self.target_distance is None:
            self.target_distance = front
            self.get_logger().info(f"[LOCK] target distance locked at {self.target_distance:.2f}m")
            return

        if front <= self.stop_distance_insurance:
            self.issue_stop("collision_guard")
            return

        distance_error = front - self.target_distance
        if abs(distance_error) <= self.distance_deadband:
            move_x = 0.0
        elif distance_error > self.distance_deadband:
            move_x = self.max_forward_speed
        else:
            move_x = -self.max_forward_speed

        diff = front_left - front_right
        if abs(diff) <= self.turn_deadband:
            move_z = 0.0
        elif diff < 0:
            move_z = -self.max_turn_speed
        else:
            move_z = self.max_turn_speed

        self.issue_move(move_x, move_z, "track_target")

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
        command = ("move", round(move_x, 3), round(move_z, 3), reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(move_x, move_z)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"track command failed: {exc}")

    def issue_stop(self, reason: str) -> None:
        command = ("stop", reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(0.0, 0.0)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"track stop failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
