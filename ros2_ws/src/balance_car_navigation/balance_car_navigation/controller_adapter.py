import time

from geometry_msgs.msg import Twist


class BackendControllerAdapter:
    def __init__(self, node, topic_name: str) -> None:
        self.publisher = node.create_publisher(Twist, topic_name, 1)

    def get_state(self) -> dict:
        raise RuntimeError("BackendControllerAdapter no longer supports direct backend state requests")

    def set_mode(self, mode_id: int) -> dict:
        raise RuntimeError("BackendControllerAdapter no longer supports direct backend mode changes")

    def set_move(self, move_x: float, move_z: float) -> dict:
        msg = Twist()
        msg.linear.x = float(move_x)
        msg.angular.z = float(move_z)
        self.publisher.publish(msg)
        return {
            "ok": True,
            "car_state": {
                "move_x": float(move_x),
                "move_z": float(move_z),
                "stop_flag": False,
            },
            "timestamp": time.time(),
        }
