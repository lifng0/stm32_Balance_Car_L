from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted, sector_distance

MODE_LIDAR_AVOID = 7


class LidarAvoidNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_avoid_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("control_topic", "/cmd_vel/lidar_avoid")
        self.declare_parameter("cruise_speed", 8.0)
        self.declare_parameter("slow_speed", 5.0)
        self.declare_parameter("wall_follow_speed", 4.5)
        self.declare_parameter("turn_speed", 8.0)
        self.declare_parameter("warning_distance", 0.90)
        self.declare_parameter("stop_distance", 0.42)
        self.declare_parameter("wall_detect_distance", 0.70)
        self.declare_parameter("wall_target_distance", 0.38)
        self.declare_parameter("wall_exit_front_distance", 0.95)
        self.declare_parameter("steer_gain", 11.0)
        self.declare_parameter("wall_gain", 16.0)

        self.adapter = BackendControllerAdapter(
            self,
            self.get_parameter("control_topic").get_parameter_value().string_value,
        )
        self.cruise_speed = self.get_parameter("cruise_speed").get_parameter_value().double_value
        self.slow_speed = self.get_parameter("slow_speed").get_parameter_value().double_value
        self.wall_follow_speed = self.get_parameter("wall_follow_speed").get_parameter_value().double_value
        self.turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.wall_detect_distance = self.get_parameter("wall_detect_distance").get_parameter_value().double_value
        self.wall_target_distance = self.get_parameter("wall_target_distance").get_parameter_value().double_value
        self.wall_exit_front_distance = self.get_parameter("wall_exit_front_distance").get_parameter_value().double_value
        self.steer_gain = self.get_parameter("steer_gain").get_parameter_value().double_value
        self.wall_gain = self.get_parameter("wall_gain").get_parameter_value().double_value

        self.system_state = {}
        self.car_state = {}
        self.last_command = None
        self.behavior_state = "cruise"
        self.follow_wall_side = ""

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_car_state(self, msg: String) -> None:
        self.car_state = decode_json_message(msg.data)

    def mode_permitted(self) -> bool:
        return int(self.car_state.get("mode", -1) or -1) == MODE_LIDAR_AVOID

    def on_scan(self, scan_msg: LaserScan) -> None:
        if not self.mode_permitted():
            self.behavior_state = "cruise"
            self.follow_wall_side = ""
            self.issue_stop("mode_not_lidar_avoid")
            return
        if not is_system_permitted(self.system_state):
            self.issue_stop("system_not_ready")
            return

        front = sector_distance(scan_msg, -18.0, 18.0, percentile=0.20)
        front_wide = sector_distance(scan_msg, -35.0, 35.0, percentile=0.20)
        left_front = sector_distance(scan_msg, 20.0, 65.0, percentile=0.20)
        right_front = sector_distance(scan_msg, -65.0, -20.0, percentile=0.20)
        left_side = sector_distance(scan_msg, 70.0, 110.0, percentile=0.25)
        right_side = sector_distance(scan_msg, -110.0, -70.0, percentile=0.25)

        front = front if front is not None else 9.9
        front_wide = front_wide if front_wide is not None else front
        left_front = left_front if left_front is not None else 9.9
        right_front = right_front if right_front is not None else 9.9
        left_side = left_side if left_side is not None else 9.9
        right_side = right_side if right_side is not None else 9.9

        if self.behavior_state.startswith("wall_follow"):
            self.follow_wall(scan_msg, front, front_wide, left_front, right_front, left_side, right_side)
            return

        if front <= self.stop_distance:
            turn_right = right_front > left_front
            self.behavior_state = "wall_follow_left" if turn_right else "wall_follow_right"
            self.follow_wall_side = "left" if turn_right else "right"
            move_z = -self.turn_speed if turn_right else self.turn_speed
            self.issue_move(0.0, move_z, "emergency_turn")
            return

        if front <= self.warning_distance:
            turn_right = right_front > left_front
            follow_side = "left" if turn_right else "right"
            self.behavior_state = f"wall_follow_{follow_side}"
            self.follow_wall_side = follow_side
            move_z = -self.turn_speed * 0.85 if turn_right else self.turn_speed * 0.85
            self.issue_move(self.slow_speed * 0.5, move_z, "avoid_arc_turn")
            return

        steer_bias = clamp((right_front - left_front) * self.steer_gain, -3.0, 3.0)
        self.behavior_state = "cruise"
        self.follow_wall_side = ""
        self.issue_move(self.cruise_speed, steer_bias, "cruise_clear")

    def follow_wall(
        self,
        scan_msg: LaserScan,
        front: float,
        front_wide: float,
        left_front: float,
        right_front: float,
        left_side: float,
        right_side: float,
    ) -> None:
        follow_left = self.follow_wall_side == "left"
        wall_side = left_side if follow_left else right_side
        free_front_side = right_front if follow_left else left_front
        signed = 1.0 if follow_left else -1.0

        if front_wide >= self.wall_exit_front_distance and free_front_side >= self.wall_exit_front_distance:
            self.behavior_state = "cruise"
            self.follow_wall_side = ""
            self.issue_move(self.cruise_speed, 0.0, "wall_exit")
            return

        if front <= self.stop_distance:
            self.issue_move(0.0, -signed * self.turn_speed, "wall_pivot")
            return

        side_error = self.wall_target_distance - wall_side
        front_error = self.wall_target_distance - min(front_wide, free_front_side)
        turn_command = clamp(signed * (side_error * self.wall_gain) - signed * (front_error * self.steer_gain), -self.turn_speed, self.turn_speed)
        forward_speed = self.wall_follow_speed if front > self.warning_distance else self.wall_follow_speed * 0.7
        self.issue_move(forward_speed, turn_command, f"wall_follow_{self.follow_wall_side}")

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
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
        move_x = float(self.car_state.get("move_x", 0.0) or 0.0)
        move_z = float(self.car_state.get("move_z", 0.0) or 0.0)
        if abs(move_x) < 1e-3 and abs(move_z) < 1e-3:
            self.last_command = command
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


if __name__ == "__main__":
    main()
