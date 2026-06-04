import json


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
