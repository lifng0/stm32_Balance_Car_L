import json
import math
from typing import Any


def decode_json_message(raw: str) -> dict:
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def is_system_permitted(system_state: dict) -> bool:
    if not system_state:
        return False
    if system_state.get("system_mode") != "running":
        return False
    if system_state.get("paused_by_pickup"):
        return False
    if system_state.get("shutdown_started"):
        return False
    if system_state.get("stop_flag", True):
        return False
    return bool(system_state.get("system_ready"))


def _percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = int(clamp(ratio, 0.0, 1.0) * (len(ordered) - 1))
    return ordered[index]


def scan_to_points(scan_msg: Any, range_margin: float = 0.02) -> list[dict]:
    points = []
    angle = float(scan_msg.angle_min)
    angle_increment = float(scan_msg.angle_increment)
    range_min = float(scan_msg.range_min)
    range_max = float(scan_msg.range_max)
    intensities = list(scan_msg.intensities)

    for index, raw_distance in enumerate(scan_msg.ranges):
        distance = float(raw_distance)
        current_angle = angle + index * angle_increment
        if not math.isfinite(distance):
            continue
        if distance < max(0.01, range_min - range_margin):
            continue
        if range_max > 0.0 and distance > range_max + range_margin:
            continue
        intensity = float(intensities[index]) if index < len(intensities) else 0.0
        x = distance * math.cos(current_angle)
        y = distance * math.sin(current_angle)
        points.append(
            {
                "index": index,
                "angle": current_angle,
                "angle_deg": math.degrees(current_angle),
                "distance": distance,
                "intensity": intensity,
                "x": x,
                "y": y,
            }
        )
    return points


def sector_distance(
    scan_msg: Any,
    angle_min_deg: float,
    angle_max_deg: float,
    percentile: float = 0.25,
) -> float | None:
    distances = []
    for point in scan_to_points(scan_msg):
        if angle_min_deg <= point["angle_deg"] <= angle_max_deg:
            distances.append(point["distance"])
    return _percentile(distances, percentile)


def extract_clusters(
    scan_msg: Any,
    angle_min_deg: float = -90.0,
    angle_max_deg: float = 90.0,
    max_cluster_gap_m: float = 0.18,
    min_cluster_points: int = 3,
    max_distance_m: float = 3.5,
) -> list[dict]:
    candidates = [
        point
        for point in scan_to_points(scan_msg)
        if angle_min_deg <= point["angle_deg"] <= angle_max_deg and point["distance"] <= max_distance_m
    ]
    if not candidates:
        return []

    clusters: list[list[dict]] = []
    current = [candidates[0]]
    for point in candidates[1:]:
        previous = current[-1]
        gap = math.hypot(point["x"] - previous["x"], point["y"] - previous["y"])
        if point["index"] != previous["index"] + 1 or gap > max_cluster_gap_m:
            clusters.append(current)
            current = [point]
        else:
            current.append(point)
    clusters.append(current)

    features = []
    for cluster in clusters:
        if len(cluster) < min_cluster_points:
            continue
        xs = [point["x"] for point in cluster]
        ys = [point["y"] for point in cluster]
        distances = [point["distance"] for point in cluster]
        start = cluster[0]
        end = cluster[-1]
        centroid_x = sum(xs) / len(xs)
        centroid_y = sum(ys) / len(ys)
        centroid_distance = math.hypot(centroid_x, centroid_y)
        centroid_angle = math.degrees(math.atan2(centroid_y, centroid_x))
        width = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        features.append(
            {
                "points": cluster,
                "point_count": len(cluster),
                "min_distance": min(distances),
                "max_distance": max(distances),
                "mean_distance": sum(distances) / len(distances),
                "start_angle_deg": start["angle_deg"],
                "end_angle_deg": end["angle_deg"],
                "angle_span_deg": end["angle_deg"] - start["angle_deg"],
                "centroid_angle_deg": centroid_angle,
                "centroid_distance_m": centroid_distance,
                "width_m": width,
                "centroid_x": centroid_x,
                "centroid_y": centroid_y,
            }
        )
    return features


def choose_front_cluster(
    clusters: list[dict],
    cone_deg: float = 18.0,
    max_distance_m: float = 2.5,
) -> dict | None:
    eligible = [
        cluster
        for cluster in clusters
        if abs(cluster["centroid_angle_deg"]) <= cone_deg and cluster["centroid_distance_m"] <= max_distance_m
    ]
    if not eligible:
        return None
    return min(
        eligible,
        key=lambda cluster: (
            abs(cluster["centroid_angle_deg"]),
            cluster["centroid_distance_m"],
            -cluster["point_count"],
        ),
    )


def match_target_cluster(
    clusters: list[dict],
    target_signature: dict,
    max_angle_error_deg: float,
    max_distance_error_m: float,
    max_width_error_m: float,
) -> dict | None:
    best_cluster = None
    best_score = None
    for cluster in clusters:
        angle_error = abs(cluster["centroid_angle_deg"] - target_signature["centroid_angle_deg"])
        distance_error = abs(cluster["centroid_distance_m"] - target_signature["centroid_distance_m"])
        width_error = abs(cluster["width_m"] - target_signature["width_m"])
        if angle_error > max_angle_error_deg:
            continue
        if distance_error > max_distance_error_m:
            continue
        if width_error > max_width_error_m:
            continue
        score = angle_error * 1.6 + distance_error * 3.5 + width_error * 2.0 - cluster["point_count"] * 0.02
        if best_score is None or score < best_score:
            best_score = score
            best_cluster = cluster
    return best_cluster


def update_target_signature(previous: dict, cluster: dict, alpha: float = 0.35) -> dict:
    return {
        "centroid_angle_deg": (1.0 - alpha) * previous["centroid_angle_deg"] + alpha * cluster["centroid_angle_deg"],
        "centroid_distance_m": (1.0 - alpha) * previous["centroid_distance_m"] + alpha * cluster["centroid_distance_m"],
        "width_m": (1.0 - alpha) * previous["width_m"] + alpha * cluster["width_m"],
        "point_count": int(round((1.0 - alpha) * previous["point_count"] + alpha * cluster["point_count"])),
    }
