from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import (
    choose_front_cluster,
    clamp,
    decode_json_message,
    extract_clusters,
    is_system_permitted,
    match_target_cluster,
    update_target_signature,
)

MODE_LIDAR_FOLLOW = 8


class LidarFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_follow_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("target_distance", 0.75)
        self.declare_parameter("distance_tolerance", 0.10)
        self.declare_parameter("max_forward_speed", 8.0)
        self.declare_parameter("max_turn_speed", 8.0)
        self.declare_parameter("turn_gain", 0.20)
        self.declare_parameter("forward_gain", 10.0)
        self.declare_parameter("max_track_angle_deg", 45.0)
        self.declare_parameter("initial_target_cone_deg", 18.0)
        self.declare_parameter("max_initial_target_distance_m", 2.2)
        self.declare_parameter("cluster_gap_m", 0.16)
        self.declare_parameter("cluster_min_points", 3)
        self.declare_parameter("max_cluster_distance_m", 3.2)
        self.declare_parameter("target_match_angle_deg", 18.0)
        self.declare_parameter("target_match_distance_m", 0.45)
        self.declare_parameter("target_match_width_m", 0.35)
        self.declare_parameter("target_lost_timeout_sec", 0.8)
        self.declare_parameter("target_reacquire_cooldown_sec", 0.5)

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
        self.initial_target_cone_deg = self.get_parameter("initial_target_cone_deg").get_parameter_value().double_value
        self.max_initial_target_distance_m = self.get_parameter("max_initial_target_distance_m").get_parameter_value().double_value
        self.cluster_gap_m = self.get_parameter("cluster_gap_m").get_parameter_value().double_value
        self.cluster_min_points = self.get_parameter("cluster_min_points").get_parameter_value().integer_value
        self.max_cluster_distance_m = self.get_parameter("max_cluster_distance_m").get_parameter_value().double_value
        self.target_match_angle_deg = self.get_parameter("target_match_angle_deg").get_parameter_value().double_value
        self.target_match_distance_m = self.get_parameter("target_match_distance_m").get_parameter_value().double_value
        self.target_match_width_m = self.get_parameter("target_match_width_m").get_parameter_value().double_value
        self.target_lost_timeout_sec = self.get_parameter("target_lost_timeout_sec").get_parameter_value().double_value
        self.target_reacquire_cooldown_sec = self.get_parameter("target_reacquire_cooldown_sec").get_parameter_value().double_value

        self.system_state = {}
        self.car_state = {}
        self.last_command = None
        self.target_signature: dict | None = None
        self.last_target_seen_at = 0.0
        self.last_target_lost_at = 0.0

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_car_state(self, msg: String) -> None:
        self.car_state = decode_json_message(msg.data)

    def mode_permitted(self) -> bool:
        return int(self.car_state.get("mode", -1) or -1) == MODE_LIDAR_FOLLOW

    def reset_target(self) -> None:
        self.target_signature = None
        self.last_target_seen_at = 0.0

    def on_scan(self, scan_msg: LaserScan) -> None:
        if not self.mode_permitted():
            self.reset_target()
            self.issue_stop("mode_not_lidar_follow")
            return
        if not is_system_permitted(self.system_state):
            self.issue_stop("system_not_ready")
            return

        clusters = extract_clusters(
            scan_msg,
            angle_min_deg=-85.0,
            angle_max_deg=85.0,
            max_cluster_gap_m=self.cluster_gap_m,
            min_cluster_points=self.cluster_min_points,
            max_distance_m=self.max_cluster_distance_m,
        )
        now = self.get_clock().now().nanoseconds / 1e9

        tracked = None
        if self.target_signature is None:
            if now - self.last_target_lost_at >= self.target_reacquire_cooldown_sec:
                tracked = choose_front_cluster(
                    clusters,
                    cone_deg=self.initial_target_cone_deg,
                    max_distance_m=self.max_initial_target_distance_m,
                )
                if tracked is not None:
                    self.target_signature = {
                        "centroid_angle_deg": tracked["centroid_angle_deg"],
                        "centroid_distance_m": tracked["centroid_distance_m"],
                        "width_m": tracked["width_m"],
                        "point_count": tracked["point_count"],
                    }
                    self.last_target_seen_at = now
        else:
            tracked = match_target_cluster(
                clusters,
                self.target_signature,
                max_angle_error_deg=self.target_match_angle_deg,
                max_distance_error_m=self.target_match_distance_m,
                max_width_error_m=self.target_match_width_m,
            )
            if tracked is not None:
                self.target_signature = update_target_signature(self.target_signature, tracked)
                self.last_target_seen_at = now

        if tracked is None:
            if self.target_signature is not None and self.last_target_seen_at > 0.0:
                if now - self.last_target_seen_at > self.target_lost_timeout_sec:
                    self.last_target_lost_at = now
                    self.reset_target()
                    self.issue_stop("target_lost")
                    return
            self.issue_stop("waiting_target")
            return

        angle = float(tracked["centroid_angle_deg"])
        distance = float(tracked["centroid_distance_m"])
        if abs(angle) > self.max_track_angle_deg:
            self.issue_stop("target_out_of_range")
            return

        distance_error = distance - self.target_distance
        if abs(distance_error) <= self.distance_tolerance:
            move_x = 0.0
        else:
            move_x = clamp(distance_error * self.forward_gain, -self.max_forward_speed, self.max_forward_speed)

        angle_ratio = min(1.0, abs(angle) / max(self.max_track_angle_deg, 1.0))
        forward_scale = max(0.15, 1.0 - angle_ratio * 0.85)
        move_x *= forward_scale
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
        if abs(move_x) < 1e-3 and abs(move_z) < 1e-3:
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
