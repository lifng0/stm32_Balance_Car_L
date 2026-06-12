import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


DEFAULT_SDK_ROOT = Path.home() / "workspace" / "balance_car" / "vendor" / "YDLidar-SDK"
CONTAINER_SDK_ROOT = Path("/workspaces/balance_car/vendor/YDLidar-SDK")


def resolve_sdk_root(preferred: str | Path | None = None) -> Path:
    candidates: list[Path] = []
    if preferred:
        candidates.append(Path(preferred).expanduser())

    for env_name in ("YDLIDAR_SDK_ROOT", "BALANCE_CAR_SDK_ROOT"):
        env_value = os.environ.get(env_name)
        if env_value:
            candidates.append(Path(env_value).expanduser())

    candidates.extend([CONTAINER_SDK_ROOT, DEFAULT_SDK_ROOT])

    for candidate in candidates:
        if (candidate / "ydlidar.py").exists() or (candidate / "_ydlidar.so").exists():
            return candidate
        build_python = candidate / "build" / "python"
        if (build_python / "ydlidar.py").exists() or (build_python / "_ydlidar.so").exists():
            return build_python

    return candidates[0] if candidates else DEFAULT_SDK_ROOT


def load_sdk(sdk_root: Path):
    sdk_root = resolve_sdk_root(sdk_root)
    if str(sdk_root) not in sys.path:
        sys.path.insert(0, str(sdk_root))
    import ydlidar  # type: ignore

    return ydlidar, sdk_root


def resolve_stable_device(device: str) -> str:
    device_path = Path(device).expanduser()
    if device_path.exists():
        device_real = device_path.resolve()
        serial_by_id = Path("/dev/serial/by-id")
        if serial_by_id.exists():
            for item in sorted(serial_by_id.iterdir()):
                try:
                    if item.resolve() == device_real:
                        return str(item)
                except FileNotFoundError:
                    continue
    return str(device_path)


def iter_serial_by_id() -> list[Path]:
    serial_by_id = Path("/dev/serial/by-id")
    if not serial_by_id.exists():
        return []
    return sorted(serial_by_id.iterdir())


def lidar_candidate_score(path: Path) -> int:
    name = path.name.lower()
    score = 0
    if "ydlidar" in name or "tmini" in name:
        score += 120
    if "silicon_labs" in name or "cp210" in name:
        score += 100
    if "1a86" in name or "ch340" in name or "ftdi" in name:
        score -= 80
    return score


def detect_lidar_by_id() -> str | None:
    candidates = iter_serial_by_id()
    if not candidates:
        return None
    best = max(candidates, key=lidar_candidate_score)
    if lidar_candidate_score(best) > 0:
        return str(best)
    return None


def detect_port(ydlidar, preferred: str | None) -> str:
    if preferred:
        return resolve_stable_device(preferred)

    by_id_port = detect_lidar_by_id()
    if by_id_port:
        return by_id_port

    ports = ydlidar.lidarPortList()
    if ports:
        for _, value in ports.items():
            return resolve_stable_device(value)

    candidates = iter_serial_by_id()
    if candidates:
        return str(candidates[0])

    tty_candidates = sorted(Path("/dev").glob("ttyUSB*"))
    if tty_candidates:
        return str(tty_candidates[0])

    raise RuntimeError("No lidar serial device found.")


def build_laser(ydlidar, port: str, baudrate: int, scan_frequency: float):
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, scan_frequency)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 12.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
    laser.setlidaropt(ydlidar.LidarPropFixedResolution, True)
    laser.setlidaropt(ydlidar.LidarPropReversion, False)
    laser.setlidaropt(ydlidar.LidarPropInverted, False)
    laser.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
    laser.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, True)
    laser.setlidaropt(ydlidar.LidarPropSupportHeartBeat, False)
    laser.enableGlassNoise(False)
    laser.enableSunNoise(False)
    return laser


def normalize_deg(angle_rad: float) -> float:
    angle_deg = math.degrees(angle_rad)
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def sector_min(points, angle_min: float, angle_max: float) -> float | None:
    distances = [distance_m for angle_deg, distance_m, _ in points if angle_min <= angle_deg <= angle_max and distance_m > 0.05]
    return min(distances) if distances else None


def sector_percentile(points, angle_min: float, angle_max: float, percentile: float) -> float | None:
    distances = [distance_m for angle_deg, distance_m, _ in points if angle_min <= angle_deg <= angle_max and distance_m > 0.05]
    if not distances:
        return None
    distances.sort()
    index = int(round((len(distances) - 1) * max(0.0, min(1.0, percentile))))
    return distances[index]


def closest_point(points, angle_min: float = -60.0, angle_max: float = 60.0):
    valid = [
        (angle_deg, distance_m)
        for angle_deg, distance_m, _ in points
        if angle_min <= angle_deg <= angle_max and distance_m > 0.05
    ]
    if not valid:
        return None
    return min(valid, key=lambda item: item[1])


def scan_to_points(scan) -> list[tuple[float, float, float]]:
    result = []
    for index in range(len(scan.points)):
        point = scan.points[index]
        result.append((normalize_deg(point.angle), float(point.range), float(point.intensity)))
    return result


def summarize_scan(scan) -> dict:
    points = scan_to_points(scan)
    closest = closest_point(points)
    return {
        "points": len(points),
        "scan_frequency_hz": round(float(scan.scanFreq), 3),
        "scan_time_s": round(float(scan.config.scan_time), 4),
        "front_min_distance_m": sector_min(points, -15.0, 15.0),
        "front_p20_distance_m": sector_percentile(points, -15.0, 15.0, 0.20),
        "front_median_distance_m": sector_percentile(points, -15.0, 15.0, 0.50),
        "front_left_min_distance_m": sector_min(points, 15.0, 60.0),
        "front_right_min_distance_m": sector_min(points, -60.0, -15.0),
        "closest_target_angle_deg": None if closest is None else round(closest[0], 2),
        "closest_target_distance_m": None if closest is None else round(closest[1], 3),
    }


def scan_to_message(scan, frame_id: str, stamp) -> LaserScan:
    polar_points: list[tuple[float, float, float]] = []
    for index in range(len(scan.points)):
        point = scan.points[index]
        polar_points.append((float(point.angle), float(point.range), float(point.intensity)))

    polar_points.sort(key=lambda item: item[0])
    point_count = len(polar_points)
    angle_min = polar_points[0][0] if polar_points else 0.0
    angle_max = polar_points[-1][0] if polar_points else 0.0
    angle_increment = 0.0 if point_count <= 1 else (angle_max - angle_min) / (point_count - 1)
    scan_time = float(getattr(getattr(scan, "config", None), "scan_time", 0.0) or 0.0)
    time_increment = 0.0 if point_count <= 0 else scan_time / point_count
    range_min = float(getattr(getattr(scan, "config", None), "min_range", 0.05) or 0.05)
    range_max = float(getattr(getattr(scan, "config", None), "max_range", 12.0) or 12.0)

    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = angle_increment
    msg.time_increment = time_increment
    msg.scan_time = scan_time
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = [float("inf") if distance_m <= 0.0 else float(distance_m) for _, distance_m, _ in polar_points]
    msg.intensities = [float(intensity) for _, _, intensity in polar_points]
    return msg


class TminiPlusNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_car_lidar")
        self.declare_parameter("sdk_root", str(resolve_sdk_root()))
        self.declare_parameter("device", "")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("scan_frequency", 10.0)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("summary_topic", "/lidar/summary_json")
        self.declare_parameter("poll_period", 0.01)
        self.declare_parameter("retry_period", 2.0)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_summary", True)
        self.declare_parameter("failure_reset_count", 5)

        self.sdk_root = self.get_parameter("sdk_root").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value.strip()
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.scan_frequency = self.get_parameter("scan_frequency").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.summary_topic = self.get_parameter("summary_topic").get_parameter_value().string_value
        self.poll_period = self.get_parameter("poll_period").get_parameter_value().double_value
        self.retry_period = self.get_parameter("retry_period").get_parameter_value().double_value
        self.publish_scan_enabled = self.get_parameter("publish_scan").get_parameter_value().bool_value
        self.publish_summary_enabled = self.get_parameter("publish_summary").get_parameter_value().bool_value
        self.failure_reset_count = self.get_parameter("failure_reset_count").get_parameter_value().integer_value

        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.ydlidar = None
        self.sdk_path = ""
        self.laser = None
        self.scan = None
        self.connected_port = ""
        self.consecutive_failures = 0
        self.last_connect_attempt = 0.0
        self.last_not_ready_reason = ""

        self.retry_timer = self.create_timer(self.retry_period, self.ensure_connected)
        self.poll_timer = self.create_timer(self.poll_period, self.poll_once)
        self.ensure_connected()

    def ensure_connected(self) -> None:
        if self.laser is not None and self.scan is not None:
            return

        now = time.time()
        if now - self.last_connect_attempt < max(self.retry_period * 0.5, 0.2):
            return
        self.last_connect_attempt = now

        try:
            if self.ydlidar is None:
                self.ydlidar, sdk_path = load_sdk(Path(self.sdk_root).expanduser())
                self.sdk_path = str(sdk_path)
                self.get_logger().info(f"loaded YDLidar SDK from {self.sdk_path}")

            port = detect_port(self.ydlidar, self.device or None)
            laser = build_laser(self.ydlidar, port, int(self.baudrate), float(self.scan_frequency))
            if not laser.initialize():
                raise RuntimeError(f"Failed to initialize lidar: {laser.DescribeError()}")
            if not laser.turnOn():
                raise RuntimeError(f"Failed to start lidar: {laser.DescribeError()}")

            self.laser = laser
            self.scan = self.ydlidar.LaserScan()
            self.connected_port = port
            self.consecutive_failures = 0
            self.last_not_ready_reason = ""
            self.get_logger().info(
                f"T-mini Plus ready on {self.connected_port}, baudrate={self.baudrate}, scan_frequency={self.scan_frequency}"
            )
        except Exception as exc:
            self.close_lidar()
            self.publish_not_ready(f"connect_failed:{exc}")
            self.get_logger().warning(f"native lidar connect failed: {exc}")

    def poll_once(self) -> None:
        if self.laser is None or self.scan is None or self.ydlidar is None:
            return

        try:
            ok = self.laser.doProcessSimple(self.scan)
        except Exception as exc:
            self.handle_scan_failure(f"scan_exception:{exc}")
            return

        if not ok:
            self.handle_scan_failure("scan_failed")
            return

        self.consecutive_failures = 0
        self.last_not_ready_reason = ""
        stamp = self.get_clock().now().to_msg()

        if self.publish_scan_enabled:
            self.scan_pub.publish(scan_to_message(self.scan, self.frame_id, stamp))

        if self.publish_summary_enabled:
            payload = summarize_scan(self.scan)
            payload["scan_ok"] = True
            payload["device"] = self.connected_port
            payload["summary_publish_time"] = time.time()
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.summary_pub.publish(msg)

    def handle_scan_failure(self, reason: str) -> None:
        self.consecutive_failures += 1
        self.publish_not_ready(reason)
        if self.consecutive_failures == 1 or self.consecutive_failures % 10 == 0:
            self.get_logger().warning(f"native lidar read failed: {reason} (count={self.consecutive_failures})")
        if self.consecutive_failures >= max(1, int(self.failure_reset_count)):
            self.get_logger().warning("native lidar resetting after repeated read failures")
            self.close_lidar()

    def publish_not_ready(self, reason: str) -> None:
        if not self.publish_summary_enabled:
            return
        if reason == self.last_not_ready_reason:
            return
        payload = {
            "scan_ok": False,
            "device": self.connected_port or self.device,
            "points": 0,
            "error": reason,
            "summary_publish_time": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.summary_pub.publish(msg)
        self.last_not_ready_reason = reason

    def close_lidar(self) -> None:
        laser = self.laser
        self.laser = None
        self.scan = None
        self.connected_port = ""
        if laser is None:
            return
        try:
            laser.turnOff()
        except Exception:
            pass
        try:
            laser.disconnecting()
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self.close_lidar()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TminiPlusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
