import json
import time

import rclpy
from std_msgs.msg import String
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import decode_json_message, is_system_permitted

MODE_LIDAR_FOLLOW = 8


class SinglePID:
    def __init__(self, p: float = 0.1, i: float = 0.0, d: float = 0.1) -> None:
        self.kp = p
        self.ki = i
        self.kd = d
        self.pid_reset()

    def pid_compute(self, target: float, current: float) -> float:
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        result = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return result

    def pid_reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0


class LidarFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_follow_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("control_topic", "/cmd_vel/lidar_follow")
        self.declare_parameter("priority_angle_deg", 25.0)
        self.declare_parameter("laser_angle_deg", 40.0)
        self.declare_parameter("response_distance_m", 0.45)
        self.declare_parameter("response_offset_m", 0.5)
        self.declare_parameter("linear_scale", 45.0)
        self.declare_parameter("angular_scale", 200.0)
        self.declare_parameter("publish_distance_limit_factor", 2.0)
        self.declare_parameter("control_refresh_sec", 0.05)
        self.declare_parameter("distance_lock_epsilon_m", 0.1)

        self.adapter = BackendControllerAdapter(
            self,
            self.get_parameter("control_topic").get_parameter_value().string_value,
        )
        self.priority_angle = self.get_parameter("priority_angle_deg").get_parameter_value().double_value
        self.laser_angle = self.get_parameter("laser_angle_deg").get_parameter_value().double_value
        self.response_distance = self.get_parameter("response_distance_m").get_parameter_value().double_value
        self.response_offset = self.get_parameter("response_offset_m").get_parameter_value().double_value
        self.linear_scale = self.get_parameter("linear_scale").get_parameter_value().double_value
        self.angular_scale = self.get_parameter("angular_scale").get_parameter_value().double_value
        self.publish_distance_limit_factor = self.get_parameter("publish_distance_limit_factor").get_parameter_value().double_value
        self.control_refresh_sec = self.get_parameter("control_refresh_sec").get_parameter_value().double_value
        self.distance_lock_epsilon = self.get_parameter("distance_lock_epsilon_m").get_parameter_value().double_value

        self.system_state = {}
        self.car_state = {}
        self.last_command = None
        self.last_move_sent_at = 0.0
        self.lin_pid = SinglePID(1.25, 0.0, 1.25)
        self.ang_pid = SinglePID(2.15, 0.0, 1.5)
        self.debug_pub = self.create_publisher(String, "/lidar_follow/debug_json", 10)

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(String, "/lidar/summary_json", self.on_summary, 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_car_state(self, msg: String) -> None:
        self.car_state = decode_json_message(msg.data)

    def mode_permitted(self) -> bool:
        return int(self.car_state.get("mode", -1) or -1) == MODE_LIDAR_FOLLOW

    def on_summary(self, msg: String) -> None:
        if not self.mode_permitted():
            self.lin_pid.pid_reset()
            self.ang_pid.pid_reset()
            self.issue_stop("mode_not_lidar_follow")
            return
        if not is_system_permitted(self.system_state):
            self.lin_pid.pid_reset()
            self.ang_pid.pid_reset()
            self.issue_stop("system_not_ready")
            return

        payload = decode_json_message(msg.data)
        if not payload.get("scan_ok", False):
            self.lin_pid.pid_reset()
            self.ang_pid.pid_reset()
            self.issue_stop("scan_not_ready")
            return

        target_distance = self.read_float(payload, "closest_target_distance_m")
        target_angle = self.read_float(payload, "closest_target_angle_deg")
        front_distance = self.read_float(payload, "front_min_distance_m")

        if front_distance is None and target_distance is None:
            self.lin_pid.pid_reset()
            self.ang_pid.pid_reset()
            self.issue_stop("no_target")
            return

        if target_distance is None or target_angle is None:
            target_distance = front_distance
            target_angle = 0.0
        elif abs(target_angle) > self.laser_angle:
            target_distance = front_distance
            target_angle = 0.0

        if target_distance is None:
            self.lin_pid.pid_reset()
            self.ang_pid.pid_reset()
            self.issue_stop("no_target")
            return

        adjusted_dist = self.response_distance if abs(target_distance - self.response_distance) < self.distance_lock_epsilon else target_distance
        move_x = -self.lin_pid.pid_compute(self.response_distance, adjusted_dist) * self.linear_scale
        angular = self.ang_pid.pid_compute(target_angle / 48.0, 0.0) * self.angular_scale
        move_z = 0.0 if abs(angular) < 0.1 else angular

        if front_distance is not None and front_distance < 0.12 and move_x > 0.0:
            move_x = 0.0
            reason = "front_guard"
        else:
            reason = "track_target"

        self.emit_debug(
            reason=reason,
            min_distance_m=round(target_distance, 3),
            min_angle_deg=round(target_angle, 3),
            front_distance_m=None if front_distance is None else round(front_distance, 3),
            move_x=round(float(move_x), 3),
            move_z=round(float(move_z), 3),
            summary_age_s=self.age_seconds(payload.get("summary_publish_time")),
        )

        if target_distance < self.response_distance * self.publish_distance_limit_factor:
            self.issue_move(move_x, move_z, reason)
        else:
            self.issue_stop("target_too_far")

    def emit_debug(self, **extra) -> None:
        payload = {
            "mode": int(self.car_state.get("mode", -1) or -1),
            "mode_name": self.car_state.get("mode_name", ""),
            "system_mode": self.system_state.get("system_mode", ""),
            "system_ready": bool(self.system_state.get("system_ready", False)),
            "stop_flag": bool(self.car_state.get("stop_flag", True)),
            "timestamp": time.time(),
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    @staticmethod
    def read_float(payload: dict, key: str) -> float | None:
        value = payload.get(key)
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if parsed <= 0.0 and "angle" not in key:
            return None
        return parsed

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
        now = time.time()
        command = ("move", round(move_x, 3), round(move_z, 3), reason)
        if command == self.last_command and now - self.last_move_sent_at < self.control_refresh_sec:
            return
        self.adapter.set_move(move_x, move_z)
        self.last_command = command
        self.last_move_sent_at = now

    def issue_stop(self, reason: str) -> None:
        now = time.time()
        command = ("stop", reason)
        if command == self.last_command and now - self.last_move_sent_at < self.control_refresh_sec:
            return
        self.adapter.set_move(0.0, 0.0)
        self.last_command = command
        self.last_move_sent_at = now

    def age_seconds(self, timestamp_value: float) -> float | None:
        try:
            value = float(timestamp_value)
        except (TypeError, ValueError):
            return None
        if value <= 0.0:
            return None
        return round(time.time() - value, 3)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
