import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MODE_TO_SOURCES = {
    3: ("vision_line",),
    4: ("vision_follow",),
    7: ("lidar_avoid",),
    8: ("lidar_follow", "lidar_track"),
}


class BalanceCarControlMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_control_mux")
        self.declare_parameter("output_topic", "/cmd_vel_bl")
        self.declare_parameter("manual_topic", "/cmd_vel")
        self.declare_parameter("lidar_avoid_topic", "/cmd_vel/lidar_avoid")
        self.declare_parameter("lidar_follow_topic", "/cmd_vel/lidar_follow")
        self.declare_parameter("lidar_track_topic", "/cmd_vel/lidar_track")
        self.declare_parameter("vision_follow_topic", "/cmd_vel/vision_follow")
        self.declare_parameter("vision_line_topic", "/cmd_vel/vision_line")
        self.declare_parameter("publish_period", 0.02)
        self.declare_parameter("command_refresh_sec", 0.10)
        self.declare_parameter("command_freshness_sec", 0.35)
        self.declare_parameter("manual_linear_scale", 10.0)
        self.declare_parameter("manual_angular_scale", 10.0)
        self.declare_parameter("manual_deadband", 0.01)

        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.manual_linear_scale = self.get_parameter("manual_linear_scale").get_parameter_value().double_value
        self.manual_angular_scale = self.get_parameter("manual_angular_scale").get_parameter_value().double_value
        self.manual_deadband = self.get_parameter("manual_deadband").get_parameter_value().double_value
        self.publish_period = self.get_parameter("publish_period").get_parameter_value().double_value
        self.command_refresh_sec = self.get_parameter("command_refresh_sec").get_parameter_value().double_value
        self.command_freshness_sec = self.get_parameter("command_freshness_sec").get_parameter_value().double_value

        self.command_state = {
            "manual": None,
            "lidar_avoid": None,
            "lidar_follow": None,
            "lidar_track": None,
            "vision_follow": None,
            "vision_line": None,
        }
        self.current_mode = -1
        self.current_mode_name = "unknown"
        self.stop_flag = True
        self.last_output = None
        self.last_output_source = ""
        self.last_output_at = 0.0

        self.output_pub = self.create_publisher(Twist, self.output_topic, 1)
        self.debug_pub = self.create_publisher(String, "/car/control_mux_json", 10)

        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(
            Twist,
            self.get_parameter("manual_topic").get_parameter_value().string_value,
            self.on_manual_cmd_vel,
            1,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("lidar_avoid_topic").get_parameter_value().string_value,
            lambda msg: self.on_direct_command("lidar_avoid", msg),
            1,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("lidar_follow_topic").get_parameter_value().string_value,
            lambda msg: self.on_direct_command("lidar_follow", msg),
            1,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("lidar_track_topic").get_parameter_value().string_value,
            lambda msg: self.on_direct_command("lidar_track", msg),
            1,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("vision_follow_topic").get_parameter_value().string_value,
            lambda msg: self.on_direct_command("vision_follow", msg),
            1,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("vision_line_topic").get_parameter_value().string_value,
            lambda msg: self.on_direct_command("vision_line", msg),
            1,
        )
        self.timer = self.create_timer(self.publish_period, self.publish_selected_command)

    def on_car_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        self.current_mode = int(payload.get("mode", self.current_mode) or -1)
        self.current_mode_name = str(payload.get("mode_name", self.current_mode_name) or self.current_mode_name)
        self.stop_flag = bool(payload.get("stop_flag", self.stop_flag))

    def on_manual_cmd_vel(self, msg: Twist) -> None:
        move_x = float(msg.linear.x) * self.manual_linear_scale
        move_z = float(msg.angular.z) * self.manual_angular_scale
        if abs(move_x) < self.manual_deadband:
            move_x = 0.0
        if abs(move_z) < self.manual_deadband:
            move_z = 0.0
        self.store_command("manual", move_x, move_z)

    def on_direct_command(self, source: str, msg: Twist) -> None:
        self.store_command(source, float(msg.linear.x), float(msg.angular.z))

    def store_command(self, source: str, move_x: float, move_z: float) -> None:
        self.command_state[source] = {
            "move_x": round(float(move_x), 3),
            "move_z": round(float(move_z), 3),
            "timestamp": time.time(),
        }

    def select_active_command(self) -> tuple[str, float, float, str]:
        now = time.time()
        if self.stop_flag:
            return "", 0.0, 0.0, "stop_flag"

        configured_sources = MODE_TO_SOURCES.get(self.current_mode)
        if configured_sources:
            freshest = None
            freshest_source = ""
            for source in configured_sources:
                state = self.command_state.get(source)
                if not state:
                    continue
                age = now - float(state.get("timestamp", 0.0) or 0.0)
                if age > self.command_freshness_sec:
                    continue
                if freshest is None or age < freshest[0]:
                    freshest = (age, state)
                    freshest_source = source
            if freshest is not None:
                _, state = freshest
                return (
                    freshest_source,
                    float(state.get("move_x", 0.0) or 0.0),
                    float(state.get("move_z", 0.0) or 0.0),
                    "mode_source",
                )
            return "", 0.0, 0.0, "active_source_stale"

        state = self.command_state.get("manual")
        if state:
            age = now - float(state.get("timestamp", 0.0) or 0.0)
            if age <= self.command_freshness_sec:
                return (
                    "manual",
                    float(state.get("move_x", 0.0) or 0.0),
                    float(state.get("move_z", 0.0) or 0.0),
                    "manual_fallback",
                )
        return "", 0.0, 0.0, "no_fresh_command"

    def publish_selected_command(self) -> None:
        source, move_x, move_z, reason = self.select_active_command()
        now = time.time()
        current = (round(move_x, 3), round(move_z, 3))
        if (
            current == self.last_output
            and source == self.last_output_source
            and now - self.last_output_at < self.command_refresh_sec
        ):
            return

        cmd = Twist()
        cmd.linear.x = current[0]
        cmd.angular.z = current[1]
        self.output_pub.publish(cmd)
        self.last_output = current
        self.last_output_source = source
        self.last_output_at = now

        payload = {
            "selected_source": source,
            "reason": reason,
            "mode": self.current_mode,
            "mode_name": self.current_mode_name,
            "stop_flag": self.stop_flag,
            "move_x": current[0],
            "move_z": current[1],
            "timestamp": now,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalanceCarControlMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
