import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_task_manager")
        self.state_sub = self.create_subscription(String, "/car/state_json", self.on_car_state, 10)
        self.system_sub = self.create_subscription(String, "/car/system_state_json", self.on_system_state, 10)
        self.event_sub = self.create_subscription(String, "/car/event_json", self.on_event, 10)
        self.status_pub = self.create_publisher(String, "/behavior/status_json", 10)
        self.timer = self.create_timer(0.5, self.publish_status)

        self.current_mode = "unknown"
        self.system_mode = "booting"
        self.paused = False
        self.last_event = "BOOT"

    def on_car_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        self.current_mode = data.get("mode_name", self.current_mode)

    def on_system_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        self.system_mode = data.get("system_mode", self.system_mode)
        self.paused = bool(data.get("paused_by_pickup", self.paused))

    def on_event(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        self.last_event = data.get("event_name", self.last_event)

    def publish_status(self) -> None:
        payload = {
            "current_mode": self.current_mode,
            "system_mode": self.system_mode,
            "paused": self.paused,
            "last_event": self.last_event,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
