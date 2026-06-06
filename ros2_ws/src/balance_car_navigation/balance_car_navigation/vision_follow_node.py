import time

import rclpy
from balance_car_interfaces.msg import VisionTarget
from rclpy.node import Node
from std_msgs.msg import String

from .controller_adapter import BackendControllerAdapter
from .lidar_utils import clamp, decode_json_message, is_system_permitted


class VisionFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_follow_node")
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8765)
        self.declare_parameter("target_center_x", 160.0)
        self.declare_parameter("target_area", 6500.0)
        self.declare_parameter("area_kp", 0.001)
        self.declare_parameter("area_kd", 0.0005)
        self.declare_parameter("turn_kp", 2.15)
        self.declare_parameter("turn_kd", 0.2)
        self.declare_parameter("max_forward_speed", 20.0)
        self.declare_parameter("max_turn_speed", 120.0)
        self.declare_parameter("area_deadband_high", 1500.0)
        self.declare_parameter("area_deadband_low", -4000.0)
        self.declare_parameter("target_timeout_sec", 0.6)
        self.declare_parameter("control_period_sec", 0.1)

        self.controller = BackendControllerAdapter(
            self.get_parameter("backend_host").get_parameter_value().string_value,
            self.get_parameter("backend_port").get_parameter_value().integer_value,
        )
        self.target_center_x = self.get_parameter("target_center_x").get_parameter_value().double_value
        self.target_area = self.get_parameter("target_area").get_parameter_value().double_value
        self.area_kp = self.get_parameter("area_kp").get_parameter_value().double_value
        self.area_kd = self.get_parameter("area_kd").get_parameter_value().double_value
        self.turn_kp = self.get_parameter("turn_kp").get_parameter_value().double_value
        self.turn_kd = self.get_parameter("turn_kd").get_parameter_value().double_value
        self.max_forward_speed = self.get_parameter("max_forward_speed").get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter("max_turn_speed").get_parameter_value().double_value
        self.area_deadband_high = self.get_parameter("area_deadband_high").get_parameter_value().double_value
        self.area_deadband_low = self.get_parameter("area_deadband_low").get_parameter_value().double_value
        self.target_timeout_sec = self.get_parameter("target_timeout_sec").get_parameter_value().double_value
        control_period = self.get_parameter("control_period_sec").get_parameter_value().double_value

        self.system_state: dict = {}
        self.car_state: dict = {}
        self.last_target: VisionTarget | None = None
        self.last_target_at = 0.0
        self.last_area_error = 0.0
        self.last_x_error = 0.0
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
        if self.car_state.get("mode_name") != "K210_Follow":
            return
        if not is_system_permitted(self.system_state):
            return
        if self.last_target is None or time.time() - self.last_target_at > self.target_timeout_sec:
            self.last_area_error = 0.0
            self.last_x_error = 0.0
            self.send_move(0.0, 0.0)
            return
        if self.last_target.target_type != "follow" or not self.last_target.detected:
            self.last_area_error = 0.0
            self.last_x_error = 0.0
            self.send_move(0.0, 0.0)
            return

        area = max(0.0, float(self.last_target.confidence))
        area_error = self.target_area - area
        area_delta = area_error - self.last_area_error
        self.last_area_error = area_error

        move_x = 0.0
        if area_error > self.area_deadband_high or area_error < self.area_deadband_low:
            move_x = area_error * self.area_kp + area_delta * self.area_kd
            move_x = clamp(move_x, -self.max_forward_speed, self.max_forward_speed)

        x_error = self.target_center_x - float(self.last_target.x)
        x_delta = x_error - self.last_x_error
        self.last_x_error = x_error
        move_z = -(x_error * self.turn_kp + x_delta * self.turn_kd)
        move_z = clamp(move_z, -self.max_turn_speed, self.max_turn_speed)

        self.send_move(move_x, move_z)

    def send_move(self, move_x: float, move_z: float) -> None:
        rounded = (round(move_x, 2), round(move_z, 2))
        if rounded == self.last_move:
            return
        try:
            self.controller.set_move(move_x, move_z)
            self.last_move = rounded
        except Exception as exc:
            self.get_logger().warning(f"follow control failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
