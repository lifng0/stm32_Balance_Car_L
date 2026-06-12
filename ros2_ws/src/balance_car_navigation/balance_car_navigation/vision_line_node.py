import time

import rclpy
from balance_car_interfaces.msg import VisionTarget
from rclpy.node import Node
from std_msgs.msg import String

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted


class VisionLineNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_line_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("control_topic", "/cmd_vel/vision_line")
        self.declare_parameter("target_center_x", 160.0)
        self.declare_parameter("forward_speed", 15.0)
        self.declare_parameter("turn_kp", 0.25)
        self.declare_parameter("turn_deadband_px", 8.0)
        self.declare_parameter("max_turn_speed", 80.0)
        self.declare_parameter("target_timeout_sec", 0.6)
        self.declare_parameter("control_period_sec", 0.1)

        self.controller = BackendControllerAdapter(
            self,
            self.get_parameter("control_topic").get_parameter_value().string_value,
        )
        self.target_center_x = self.get_parameter("target_center_x").get_parameter_value().double_value
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.turn_kp = self.get_parameter("turn_kp").get_parameter_value().double_value
        self.turn_deadband_px = self.get_parameter("turn_deadband_px").get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter("max_turn_speed").get_parameter_value().double_value
        self.target_timeout_sec = self.get_parameter("target_timeout_sec").get_parameter_value().double_value
        control_period = self.get_parameter("control_period_sec").get_parameter_value().double_value

        self.system_state: dict = {}
        self.car_state: dict = {}
        self.last_target: VisionTarget | None = None
        self.last_target_at = 0.0
        self.last_move = (None, None)

        self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.create_subscription(VisionTarget, "/vision/target", self.on_target, 10)
        self.timer = self.create_timer(control_period, self.control_loop)

    def on_system_state(self, msg: String) -> None:
        self.system_state = decode_json_message(msg.data)

    def on_car_state(self, msg: String) -> None:
        self.car_state = decode_json_message(msg.data)

    def on_target(self, msg: VisionTarget) -> None:
        self.last_target = msg
        self.last_target_at = time.time()

    def control_loop(self) -> None:
        if self.car_state.get("mode_name") != "K210_Line":
            return
        if not is_system_permitted(self.system_state):
            return
        if self.last_target is None or time.time() - self.last_target_at > self.target_timeout_sec:
            self.send_move(0.0, 0.0)
            return
        if self.last_target.target_type != "line" or not self.last_target.detected:
            self.send_move(0.0, 0.0)
            return

        error_x = float(self.last_target.x) - self.target_center_x
        if abs(error_x) < self.turn_deadband_px:
            error_x = 0.0

        move_x = self.forward_speed
        move_z = clamp(error_x * self.turn_kp, -self.max_turn_speed, self.max_turn_speed)
        self.send_move(move_x, move_z)

    def send_move(self, move_x: float, move_z: float) -> None:
        rounded = (round(move_x, 2), round(move_z, 2))
        if rounded == self.last_move:
            return
        try:
            self.controller.set_move(move_x, move_z)
            self.last_move = rounded
        except Exception as exc:
            self.get_logger().warning(f"line control failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
