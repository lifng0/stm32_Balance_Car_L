#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


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
    if sdk_root.exists():
        sys.path.insert(0, str(sdk_root))
    import ydlidar  # type: ignore
    return ydlidar


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
    # The official USB-UART adapter exposes DTR-based motor control.
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
    distances = []
    for angle_deg, distance_m, _ in points:
        if angle_min <= angle_deg <= angle_max:
            if distance_m > 0.05:
                distances.append(distance_m)
    return min(distances) if distances else None


def sector_percentile(points, angle_min: float, angle_max: float, percentile: float) -> float | None:
    distances = []
    for angle_deg, distance_m, _ in points:
        if angle_min <= angle_deg <= angle_max:
            if distance_m > 0.05:
                distances.append(distance_m)
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
    for i in range(len(scan.points)):
        point = scan.points[i]
        result.append((normalize_deg(point.angle), point.range, point.intensity))
    return result


def scan_to_laserscan_dict(scan) -> dict:
    polar_points: list[tuple[float, float, float]] = []
    for i in range(len(scan.points)):
        point = scan.points[i]
        polar_points.append((float(point.angle), float(point.range), float(point.intensity)))

    if not polar_points:
        return {
            "points": 0,
            "angle_min": 0.0,
            "angle_max": 0.0,
            "angle_increment": 0.0,
            "scan_time": 0.0,
            "time_increment": 0.0,
            "range_min": 0.05,
            "range_max": 12.0,
            "ranges": [],
            "intensities": [],
        }

    polar_points.sort(key=lambda item: item[0])
    angle_min = polar_points[0][0]
    angle_max = polar_points[-1][0]
    point_count = len(polar_points)
    angle_increment = 0.0 if point_count <= 1 else (angle_max - angle_min) / (point_count - 1)
    scan_time = float(getattr(getattr(scan, "config", None), "scan_time", 0.0) or 0.0)
    time_increment = 0.0 if point_count <= 0 else scan_time / max(point_count, 1)
    range_min = float(getattr(getattr(scan, "config", None), "min_range", 0.05) or 0.05)
    range_max = float(getattr(getattr(scan, "config", None), "max_range", 12.0) or 12.0)

    ranges = []
    intensities = []
    for _, distance_m, intensity in polar_points:
        if distance_m <= 0.0:
            ranges.append(None)
        else:
            ranges.append(round(distance_m, 4))
        intensities.append(round(float(intensity), 3))

    return {
        "points": point_count,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "scan_time": scan_time,
        "time_increment": time_increment,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
        "intensities": intensities,
    }


def summarize_scan(scan) -> dict:
    points = scan_to_points(scan)
    closest = closest_point(points)
    summary = {
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
    return summary


def cmd_list(args):
    ydlidar = load_sdk(Path(args.sdk_root).expanduser())
    ports = ydlidar.lidarPortList()
    if ports:
        for key, value in ports.items():
            print(f"{key}: {value}")
        return 0

    serial_by_id = sorted(Path("/dev/serial/by-id").glob("*"))
    for item in serial_by_id:
        print(item)
    return 0


def run_scan(args, stream: bool):
    ydlidar = load_sdk(Path(args.sdk_root).expanduser())
    port = detect_port(ydlidar, args.device)
    print(f"Using port: {port}")

    laser = build_laser(ydlidar, port, args.baudrate, args.scan_frequency)
    if not laser.initialize():
        raise RuntimeError(f"Failed to initialize lidar: {laser.DescribeError()}")
    if not laser.turnOn():
        raise RuntimeError(f"Failed to start lidar: {laser.DescribeError()}")

    scan = ydlidar.LaserScan()
    try:
        if stream:
            count = 0
            while ydlidar.os_isOk():
                ok = laser.doProcessSimple(scan)
                if not ok:
                    print(json.dumps({"scan_ok": False, "error": "Failed to get lidar data"}))
                    time.sleep(0.05)
                    continue
                summary = summarize_scan(scan)
                summary["scan_ok"] = True
                summary["port"] = port
                summary["index"] = count
                print(json.dumps(summary, ensure_ascii=False))
                sys.stdout.flush()
                count += 1
                if args.count and count >= args.count:
                    break
        else:
            ok = laser.doProcessSimple(scan)
            if not ok:
                raise RuntimeError("Failed to get lidar data")
            summary = summarize_scan(scan)
            summary["scan_ok"] = True
            summary["port"] = port
            print(json.dumps(summary, ensure_ascii=False, indent=2))
    finally:
        laser.turnOff()
        laser.disconnecting()

    return 0


def main():
    parser = argparse.ArgumentParser(description="T-mini Plus lidar bridge for Raspberry Pi")
    parser.add_argument("--sdk-root", default=str(resolve_sdk_root()), help="YDLidar-SDK root path")
    parser.add_argument("--device", default=None, help="serial device path, auto-detect if omitted")
    parser.add_argument("--baudrate", type=int, default=230400)
    parser.add_argument("--scan-frequency", type=float, default=10.0)

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list", help="list candidate lidar serial devices")
    subparsers.add_parser("scan-once", help="capture one scan summary")

    stream_parser = subparsers.add_parser("stream", help="continuously print scan summaries as JSON")
    stream_parser.add_argument("--count", type=int, default=0, help="stop after N scan summaries, 0 means infinite")

    args = parser.parse_args()

    if args.command == "list":
        return cmd_list(args)
    if args.command == "scan-once":
        return run_scan(args, stream=False)
    if args.command == "stream":
        return run_scan(args, stream=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
