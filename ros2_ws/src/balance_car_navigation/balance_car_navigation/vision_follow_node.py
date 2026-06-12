from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted


TEXT_TAG = "FOLLOW"


class VisionFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_follow_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("control_topic", "/cmd_vel/vision_follow")
        self.declare_parameter("cx_deadband", 25)
        self.declare_parameter("width_deadband", 8)
        self.declare_parameter("max_forward_speed", 8.0)
        self.declare_parameter("max_turn_speed", 8.0)
        self.declare_parameter("cx_center", 160)

        self.adapter = BackendControllerAdapter(
            self,
            self.get_parameter("control_topic").get_parameter_value().string_value,
        )
        self.cx_deadband = self.get_parameter("cx_deadband").get_parameter_value().integer_value
        self.width_deadband = self.get_parameter("width_deadband").get_parameter_value().integer_value
        self.max_forward_speed = self.get_parameter("max_forward_speed").get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter("max_turn_speed").get_parameter_value().double_value
        self.cx_center = self.get_parameter("cx_center").get_parameter_value().integer_value

        self.system_state = {}
        self.last_command = None
        self.target_width = None
        self.lost_counter = 0
        self.sent_mode_color = False

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/vision/status_json", self.on_vision, 10)
        self.k210_cmd_pub = self.create_publisher(String, "/vision/k210_command", 10)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)
        if not self.sent_mode_color:
            cmd = String()
            cmd.data = "MODE:COLOR"
            self.k210_cmd_pub.publish(cmd)
            self.sent_mode_color = True
            self.get_logger().info("[CMD] sent MODE:COLOR to K210")

    def on_vision(self, msg: String) -> None:
        payload = decode_json_message(msg.data)
        if not payload.get("valid", False):
            return

        raw_text = payload.get("text", "")

        if not raw_text.startswith(TEXT_TAG):
            return

        if "WAITING" in raw_text:
            self.target_width = None
            self.issue_stop("k210_learning")
            return

        if "NONE" in raw_text:
            if self.target_width is None:
                return
            self.lost_counter += 1
            if self.lost_counter >= 20:
                self.issue_stop("lost_2s_timeout")
            return

        self.lost_counter = 0
        data_str = raw_text.split(":", 1)[1]
        parts = data_str.replace(";", ",").split(",")
        if len(parts) < 3:
            self.issue_stop("bad_frame")
            return

        cx = int(parts[0])
        width = int(parts[2])

        if self.target_width is None:
            self.target_width = width
            self.get_logger().info(
                f"[LOCK] target width locked at {self.target_width} px"
            )
            return

        width_error = self.target_width - width
        if abs(width_error) <= self.width_deadband:
            move_x = 0.0
        elif width_error > self.width_deadband:
            move_x = self.max_forward_speed
        else:
            move_x = -self.max_forward_speed

        offset = cx - self.cx_center
        if abs(offset) <= self.cx_deadband:
            move_z = 0.0
        else:
            move_z = clamp(
                offset * 0.1,
                -self.max_turn_speed,
                self.max_turn_speed,
            )

        self.issue_move(move_x, move_z, "track_vision")

    def issue_move(self, move_x: float, move_z: float, reason: str) -> None:
        command = ("move", round(move_x, 3), round(move_z, 3), reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(move_x, move_z)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"move failed: {exc}")

    def issue_stop(self, reason: str) -> None:
        command = ("stop", reason)
        if command == self.last_command:
            return
        try:
            self.adapter.set_move(0.0, 0.0)
            self.last_command = command
        except Exception as exc:
            self.get_logger().warning(f"stop failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
