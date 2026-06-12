#!/usr/bin/env python3
import argparse
from collections import deque
import json
import socketserver
import struct
import subprocess
import threading
import time
from pathlib import Path

import serial

from pi_serial_bridge import (
    CMD_ACK,
    CMD_EVENT,
    CMD_HEARTBEAT,
    CMD_HEARTBEAT_ACK,
    CMD_NACK,
    CMD_QUERY_STATUS,
    CMD_SET_HOST_STATE,
    CMD_SET_MODE,
    CMD_SET_MOVE,
    CMD_STATUS,
    ERROR_NAME,
    EVENT_NAME,
    FrameParser,
    build_frame,
    open_port,
)
from tminiplus_bridge import build_laser, detect_port, load_sdk, resolve_sdk_root, scan_to_laserscan_dict, summarize_scan


HOST_STATE_PI_READY = 0x01
HOST_STATE_LIDAR_READY = 0x02
HOST_STATE_SYSTEM_READY = 0x04
HOST_STATE_SHUTDOWN_ACK = 0x08

BACKEND_DEFAULT_HOST = "127.0.0.1"
BACKEND_DEFAULT_PORT = 8765
ROS_READY_LEASE_SEC = 3.0

EVENT_START_REQUEST = 0x10
EVENT_TIMEOUT_STOP = 0x04
EVENT_MODE_SELECT = 0x11
EVENT_STOP_ASSERT = 0x12
EVENT_STOP_CLEAR = 0x13
EVENT_SHUTDOWN_REQ = 0x14
VISION_TYPE_NONE = 0
VISION_TYPE_TEXT = 1
VISION_TYPE_AI = 2

MODE_NAME = {
    0: "Normal",
    1: "Weight_M",
    3: "K210_Line",
    4: "K210_Follow",
    7: "Lidar_Avoid",
    8: "Lidar_Follow",
}

SUPPORTED_MODE_IDS = {0, 1, 3, 4, 8}
K210_VISION_MODES = {3, 4}
LINE_MODE_ID = 3
FOLLOW_MODE_ID = 4
LIDAR_FOLLOW_MODE_ID = 8
K210_ALWAYS_CONNECTED = True

DEFAULT_LIDAR_POLICY = {
    "default_enabled": False,
    "prewarm_on_mode_select": True,
    "keep_enabled_during_pause": True,
    "disable_grace_period_sec": 3.0,
    "modes": {
        "0": {"name": "Normal", "lidar_enabled": False},
        "1": {"name": "Weight_M", "lidar_enabled": False},
        "3": {"name": "K210_Line", "lidar_enabled": False},
        "4": {"name": "K210_Follow", "lidar_enabled": False},
        "7": {"name": "Lidar_Avoid", "lidar_enabled": True},
        "8": {"name": "Lidar_Follow", "lidar_enabled": True},
        "9": {"name": "Lidar_SLAM", "lidar_enabled": True},
    },
}
DEFAULT_LIDAR_POLICY_PATH = Path.home() / "workspace" / "balance_car" / "scripts" / "lidar_mode_policy.json"


def log(message: str) -> None:
    print(f"[pi-coordinator] {message}", flush=True)


def sanitize_k210_text(text: str, max_len: int = 48) -> str:
    sanitized = []
    for ch in str(text):
        code = ord(ch)
        if ch in "\r\n\t":
            sanitized.append(" ")
        elif 32 <= code <= 126:
            sanitized.append(ch)
        else:
            sanitized.append("?")
    return "".join(sanitized).strip()[:max_len]


def load_lidar_policy(policy_path: Path | None) -> dict:
    policy = json.loads(json.dumps(DEFAULT_LIDAR_POLICY))
    if policy_path is None or not policy_path.exists():
        return policy

    loaded = json.loads(policy_path.read_text(encoding="utf-8"))
    for key in ("default_enabled", "prewarm_on_mode_select", "keep_enabled_during_pause", "disable_grace_period_sec"):
        if key in loaded:
            policy[key] = loaded[key]
    if isinstance(loaded.get("modes"), dict):
        for mode_id, mode_policy in loaded["modes"].items():
            mode_key = str(mode_id)
            current = policy["modes"].get(mode_key, {"name": f"mode_{mode_key}", "lidar_enabled": False})
            if isinstance(mode_policy, dict):
                current.update(mode_policy)
            else:
                current["lidar_enabled"] = bool(mode_policy)
            policy["modes"][mode_key] = current
    return policy


class HostLidarWorker:
    def __init__(
        self,
        sdk_root: Path,
        preferred_device: str | None,
        baudrate: int,
        scan_frequency: float,
        motor_dtr_control: bool,
        motor_dtr_active_high: bool,
        motor_settle_time: float,
    ) -> None:
        self.sdk_root = sdk_root
        self.preferred_device = preferred_device
        self.baudrate = baudrate
        self.scan_frequency = scan_frequency
        self.motor_dtr_control = motor_dtr_control
        self.motor_dtr_active_high = motor_dtr_active_high
        self.motor_settle_time = motor_settle_time
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._enabled = False
        self._ydlidar = None
        self._laser = None
        self._scan = None
        self._port = ""
        self._summary: dict | None = None
        self._scan_data: dict | None = None
        self._last_error = ""
        self._motor_hold_serial: serial.Serial | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        if self.motor_dtr_control:
            self._hold_motor_disabled()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._close_laser()
        self._hold_motor_disabled()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._release_motor_hold()

    def wait_until_ready(self, timeout: float) -> bool:
        return self._ready_event.wait(timeout)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled
            if not enabled:
                self._summary = None
                self._scan_data = None
                self._last_error = ""
                self._ready_event.clear()
        if not enabled:
            self._close_laser(clear_summary=True)

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def snapshot(self) -> tuple[str, dict | None, dict | None, bool, bool]:
        with self._lock:
            return (
                self._port,
                None if self._summary is None else dict(self._summary),
                None if self._scan_data is None else dict(self._scan_data),
                self._enabled,
                self._ready_event.is_set(),
            )

    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _resolved_port(self) -> str:
        preferred = self.preferred_device or self._port
        if preferred:
            return str(preferred)
        ydlidar = self._ydlidar or load_sdk(self.sdk_root)
        return detect_port(ydlidar, self.preferred_device)

    def _dtr_level_for_motor(self, enabled: bool) -> bool:
        return enabled if self.motor_dtr_active_high else not enabled

    def _apply_motor_control(self, handle: serial.Serial, enabled: bool) -> None:
        level = self._dtr_level_for_motor(enabled)
        handle.dtr = level
        # Some USB-UART boards internally mirror or gate motor control via RTS.
        handle.rts = level

    def _release_motor_hold(self) -> None:
        handle = self._motor_hold_serial
        self._motor_hold_serial = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def _hold_motor_disabled(self) -> None:
        if not self.motor_dtr_control:
            return
        try:
            port = self._resolved_port()
            handle = self._motor_hold_serial
            if handle is None or not handle.is_open or Path(handle.port) != Path(port):
                self._release_motor_hold()
                handle = serial.Serial(port, baudrate=self.baudrate, timeout=0.1)
                self._motor_hold_serial = handle
            self._apply_motor_control(handle, enabled=False)
        except Exception as exc:
            self._set_error(f"lidar motor hold failed: {exc}")
            log(f"lidar motor hold failed: {exc}")

    def _connect(self) -> None:
        self._release_motor_hold()
        self._ydlidar = load_sdk(self.sdk_root)
        port = detect_port(self._ydlidar, self.preferred_device)
        laser = build_laser(self._ydlidar, port, self.baudrate, self.scan_frequency)
        if not laser.initialize():
            raise RuntimeError(f"lidar initialize failed: {laser.DescribeError()}")
        if not laser.turnOn():
            raise RuntimeError(f"lidar start failed: {laser.DescribeError()}")
        if self.motor_dtr_control and self.motor_settle_time > 0:
            time.sleep(self.motor_settle_time)
        scan = self._ydlidar.LaserScan()
        with self._lock:
            self._port = port
        self._laser = laser
        self._scan = scan
        self._ready_event.clear()

    def _close_laser(self, clear_summary: bool = False) -> None:
        laser = self._laser
        self._laser = None
        self._scan = None
        if clear_summary:
            with self._lock:
                self._summary = None
                self._scan_data = None
        if laser is not None:
            try:
                laser.turnOff()
                laser.disconnecting()
            except Exception:
                pass
        self._hold_motor_disabled()
        if self.motor_dtr_control and self.motor_settle_time > 0:
            time.sleep(self.motor_settle_time)

    def _run(self) -> None:
        warned_scan_failure = False
        while not self._stop_event.is_set():
            if not self.is_enabled():
                self._ready_event.clear()
                if self._laser is not None:
                    log("lidar worker disabled, stopping scan")
                    self._close_laser(clear_summary=True)
                time.sleep(0.1)
                continue

            if self._laser is None or self._scan is None or self._ydlidar is None:
                try:
                    self._connect()
                    warned_scan_failure = False
                    self._set_error("")
                    log(f"lidar connected on {self._port}")
                except Exception as exc:
                    self._set_error(str(exc))
                    self._ready_event.clear()
                    log(f"lidar init retry in 1s: {exc}")
                    self._close_laser(clear_summary=True)
                    time.sleep(1.0)
                    continue

            try:
                if not self._laser.doProcessSimple(self._scan):
                    if not warned_scan_failure:
                        log("lidar scan failed, reconnecting")
                        warned_scan_failure = True
                    self._set_error("lidar scan failed")
                    self._ready_event.clear()
                    self._close_laser(clear_summary=True)
                    time.sleep(0.5)
                    continue

                summary = summarize_scan(self._scan)
                scan_data = scan_to_laserscan_dict(self._scan)
                now = time.time()
                summary["coordinator_scan_time"] = now
                scan_data["coordinator_scan_time"] = now
                with self._lock:
                    self._summary = summary
                    self._scan_data = scan_data
                self._set_error("")
                self._ready_event.set()
                warned_scan_failure = False
            except Exception as exc:
                self._set_error(str(exc))
                self._ready_event.clear()
                log(f"lidar runtime error, reconnecting: {exc}")
                self._close_laser(clear_summary=True)
                time.sleep(0.5)


class CoordinatorBackendServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 128

    def __init__(self, server_address, request_handler_class, coordinator):
        self.coordinator = coordinator
        super().__init__(server_address, request_handler_class)


class CoordinatorBackendHandler(socketserver.BaseRequestHandler):
    max_request_bytes = 65536

    def handle(self) -> None:
        self.request.settimeout(2.0)
        raw = bytearray()
        try:
            while len(raw) < self.max_request_bytes:
                chunk = self.request.recv(4096)
                if not chunk:
                    break
                raw.extend(chunk)
                if b"\n" in chunk:
                    break
        except TimeoutError:
            response = {"ok": False, "error": "timeout"}
        else:
            line = bytes(raw).split(b"\n", 1)[0].strip()
            if not line:
                return
            try:
                request = json.loads(line.decode("utf-8"))
            except Exception:
                response = {"ok": False, "error": "bad_json"}
            else:
                try:
                    response = self.server.coordinator.handle_backend_request(request)
                except Exception as exc:
                    response = {
                        "ok": False,
                        "error": "backend_exception",
                        "message": str(exc),
                        "type": type(exc).__name__,
                    }

        payload = (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            self.request.sendall(payload)
        finally:
            try:
                self.request.shutdown(2)
            except Exception:
                pass
            try:
                self.request.close()
            except Exception:
                pass


def parse_status_payload(payload: bytes) -> dict:
    if len(payload) < 11:
        raise ValueError("STATUS payload too short")
    move_x = int.from_bytes(payload[3:5], "little", signed=True) / 10.0
    move_z = int.from_bytes(payload[5:7], "little", signed=True) / 10.0
    battery = int.from_bytes(payload[7:9], "little", signed=False) / 100.0
    angle = int.from_bytes(payload[9:11], "little", signed=True) / 10.0
    return {
        "mode": payload[0],
        "mode_name": MODE_NAME.get(payload[0], str(payload[0])),
        "stop_flag": payload[1],
        "low_power": payload[2],
        "move_x": move_x,
        "move_z": move_z,
        "battery": battery,
        "angle": angle,
    }


def parse_vision_payload(payload: bytes) -> dict:
    if len(payload) < 3:
        raise ValueError("VISION payload too short")

    vision_type = payload[0]
    mode = payload[1]
    valid = bool(payload[2])
    result = {
        "source": "k210",
        "vision_type": vision_type,
        "mode": mode,
        "mode_name": MODE_NAME.get(mode, str(mode)),
        "valid": valid,
    }

    if vision_type == VISION_TYPE_TEXT:
        text_len = payload[3] if len(payload) >= 4 else 0
        text_bytes = payload[4 : 4 + text_len]
        result["text"] = text_bytes.decode("utf-8", errors="replace")
        return result

    if vision_type == VISION_TYPE_AI:
        if len(payload) < 13:
            raise ValueError("VISION AI payload too short")
        x, y, w, h, area = struct.unpack_from("<HHHHH", payload, 3)
        result.update(
            {
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": area,
            }
        )
        return result

    result["text"] = ""
    return result


def parse_k210_uart_message(mode_id: int | None, body: str) -> dict:
    mode_value = int(mode_id) if mode_id is not None else -1
    result = {
        "source": "k210_uart",
        "transport": "usb_serial",
        "mode": mode_value,
        "mode_name": MODE_NAME.get(mode_value, str(mode_value)),
        "valid": True,
        "raw": body,
        "timestamp": time.time(),
    }

    parts = body.split(":", 1)
    prefix = parts[0].strip().upper() if parts else ""
    detail = parts[1].strip() if len(parts) > 1 else ""

    if prefix in ("BOOT", "PONG", "ACK", "ERR", "SHOW_OK", "STATUS", "COLOR"):
        result["message_type"] = prefix

    if prefix == "BOOT":
        result["vision_type"] = VISION_TYPE_TEXT
        result["text"] = detail or body
        result["valid"] = True
        return result

    if prefix == "PONG":
        result["vision_type"] = VISION_TYPE_TEXT
        result["text"] = "PONG"
        result["valid"] = True
        return result

    if prefix in ("ACK", "ERR", "SHOW_OK"):
        result["vision_type"] = VISION_TYPE_TEXT
        result["text"] = detail or body
        result["valid"] = prefix != "ERR"
        return result

    if prefix == "STATUS":
        result["vision_type"] = VISION_TYPE_TEXT
        result["text"] = detail or body
        result["valid"] = True
        for item in detail.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            key = key.strip().lower()
            value = value.strip()
            result[key] = value
        if "target" in result:
            result["color_target"] = result["target"]
        if "detected" in result:
            result["detected"] = result["detected"] not in ("0", "false", "False", "")
        return result

    if prefix == "COLOR":
        fields = [item.strip() for item in detail.split(",") if item.strip()]
        result["vision_type"] = VISION_TYPE_AI
        result["target_type"] = "follow"
        result["valid"] = False
        if fields:
            result["color_name"] = fields[0]
            if fields[0].upper() != "NONE":
                result["valid"] = True
        if len(fields) >= 6:
            try:
                result["x"] = int(fields[1])
                result["y"] = int(fields[2])
                result["w"] = int(fields[3])
                result["h"] = int(fields[4])
                result["area"] = int(fields[5])
            except ValueError:
                pass
        result["text"] = detail or body
        return result

    if prefix in ("LINE", "FOLLOW"):
        fields = [item.strip() for item in detail.split(",") if item.strip()]
        coords = []
        for item in fields:
            try:
                coords.append(int(item))
            except ValueError:
                continue
        x = coords[0] if len(coords) >= 1 else 0
        y = coords[1] if len(coords) >= 2 else 0
        w = coords[2] if len(coords) >= 3 else 0
        h = coords[3] if len(coords) >= 4 else 0
        result.update(
            {
                "vision_type": VISION_TYPE_AI,
                "target_type": "line" if prefix == "LINE" else "follow",
                "valid": bool(coords),
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": w * h,
            }
        )
        return result

    digits = "".join(ch for ch in body if ch.isdigit())
    if mode_value in K210_VISION_MODES and len(digits) >= 6:
        x = int(digits[0:3])
        y = int(digits[3:6])
        w = int(digits[6:9]) if len(digits) >= 9 else 0
        h = int(digits[9:12]) if len(digits) >= 12 else 0
        target_type = "follow" if mode_value == FOLLOW_MODE_ID else "line"
        area = w * h
        result.update(
            {
                "vision_type": VISION_TYPE_AI,
                "target_type": target_type,
                "valid": True,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": area,
            }
        )
        return result

    result["vision_type"] = VISION_TYPE_TEXT
    result["text"] = body
    return result


class HostK210Worker:
    def __init__(
        self,
        preferred_device: str | None,
        baudrate: int,
        mode_provider,
    ) -> None:
        self.preferred_device = preferred_device
        self.baudrate = baudrate
        self.mode_provider = mode_provider
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._serial: serial.Serial | None = None
        self._port = ""
        self._ready = False
        self._enabled = False
        self._last_error = ""
        self._last_rx_at = 0.0
        self._vision_state: dict | None = None
        self._last_requested_remote_mode: str | None = None
        self._buffer = bytearray()
        self._collecting = False
        self._line_buffer = bytearray()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._stop_remote_activity()
        self._close_serial()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def set_enabled(self, enabled: bool) -> None:
        should_disable = False
        with self._lock:
            previous_enabled = self._enabled
            self._enabled = bool(enabled)
            if previous_enabled and not self._enabled:
                self._ready = False
                self._last_error = ""
                self._last_requested_remote_mode = None
                should_disable = True
        if should_disable:
            self._stop_remote_activity()
            self._close_serial()

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def snapshot(self) -> tuple[str, dict | None, bool, str, float]:
        with self._lock:
            return (
                self._port,
                None if self._vision_state is None else dict(self._vision_state),
                self._ready,
                self._last_error,
                self._last_rx_at,
            )

    def wait_until_ready(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                handle = self._serial
                if self._ready and handle is not None and handle.is_open:
                    return True
            time.sleep(0.05)
        return False

    def send_text(self, text: str) -> None:
        self.set_enabled(True)
        if not self.wait_until_ready(timeout=1.0):
            raise RuntimeError("k210 serial not connected")
        clean_text = sanitize_k210_text(text)
        if not clean_text:
            raise RuntimeError("k210 text is empty after sanitization")
        payload = (clean_text + "\n").encode("ascii", errors="replace")
        with self._lock:
            handle = self._serial
        if handle is None or not handle.is_open:
            raise RuntimeError("k210 serial not connected")
        handle.write(payload)
        handle.flush()

    def _send_best_effort(self, text: str) -> None:
        clean_text = sanitize_k210_text(text)
        if not clean_text:
            return
        with self._lock:
            handle = self._serial
        if handle is None or not handle.is_open:
            return
        try:
            handle.write((clean_text + "\n").encode("ascii", errors="replace"))
            handle.flush()
        except Exception:
            pass

    def _stop_remote_activity(self) -> None:
        # Explicitly park the K210 in idle mode before closing the serial
        # link so it won't keep running a visual demo after we leave
        # vision-related modes during mode selection.
        self._send_best_effort("COLOR:STOP")
        self._send_best_effort("MODE:IDLE")
        self._last_requested_remote_mode = "IDLE"

    def sync_remote_mode(self, desired_mode: str) -> None:
        normalized_mode = desired_mode.strip().upper()
        if not normalized_mode:
            return
        self.set_enabled(True)
        if not self.wait_until_ready(timeout=1.0):
            raise RuntimeError("k210 serial not connected")
        if normalized_mode == self._last_requested_remote_mode:
            return
        self._send_best_effort(f"MODE:{normalized_mode}")
        self._last_requested_remote_mode = normalized_mode

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _set_ready(self, ready: bool) -> None:
        with self._lock:
            self._ready = ready

    def _resolved_port(self) -> str:
        if self.preferred_device:
            return str(self.preferred_device)
        return detect_k210_port(None)

    def _connect(self) -> None:
        port = self._resolved_port()
        handle = serial.Serial(port, baudrate=self.baudrate, timeout=0.1)
        with self._lock:
            self._serial = handle
            self._port = port
            self._ready = True
            self._last_error = ""
        self._collecting = False
        self._buffer = bytearray()
        self._line_buffer = bytearray()

    def _close_serial(self) -> None:
        with self._lock:
            handle = self._serial
            self._serial = None
            self._ready = False
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def _handle_frame(self, body: str) -> None:
        parsed = parse_k210_uart_message(self.mode_provider(), body)
        with self._lock:
            self._vision_state = parsed
            self._last_rx_at = time.time()

    def _handle_line(self, body: str) -> None:
        text = body.strip()
        if not text:
            return
        self._handle_frame(text)

    def _feed_bytes(self, data: bytes) -> None:
        for value in data:
            if value in (ord("\r"), ord("\n")):
                if self._line_buffer:
                    body = self._line_buffer.decode("utf-8", errors="replace")
                    self._line_buffer = bytearray()
                    self._handle_line(body)
                continue
            if value == ord("$") and not self._collecting:
                self._collecting = True
                self._buffer = bytearray()
                continue
            if not self._collecting:
                self._line_buffer.append(value)
                if len(self._line_buffer) > 128:
                    body = self._line_buffer.decode("utf-8", errors="replace")
                    self._line_buffer = bytearray()
                    self._handle_line(body)
                continue
            if value == ord("#"):
                body = self._buffer.decode("utf-8", errors="replace")
                self._collecting = False
                self._buffer = bytearray()
                if body:
                    self._handle_frame(body)
                continue
            self._buffer.append(value)
            if len(self._buffer) > 64:
                self._collecting = False
                self._buffer = bytearray()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if not self.is_enabled():
                if self._serial is not None:
                    log("k210 worker disabled, closing serial")
                    self._close_serial()
                time.sleep(0.1)
                continue

            if self._serial is None:
                try:
                    self._connect()
                    log(f"k210 connected on {self._port}")
                except Exception as exc:
                    self._set_error(str(exc))
                    log(f"k210 init retry in 1s: {exc}")
                    time.sleep(1.0)
                    continue

            try:
                with self._lock:
                    handle = self._serial
                if handle is None:
                    time.sleep(0.1)
                    continue
                data = handle.read(64)
                if not data:
                    continue
                with self._lock:
                    self._last_rx_at = time.time()
                    self._ready = True
                    self._last_error = ""
                self._feed_bytes(data)
            except Exception as exc:
                self._set_error(str(exc))
                log(f"k210 runtime error, reconnecting: {exc}")
                self._close_serial()
                time.sleep(0.5)


class STM32Coordinator:
    def __init__(
        self,
        device: str,
        baudrate: int,
        heartbeat_interval: float,
        k210_device: str | None,
        k210_baudrate: int,
        backend_host: str,
        backend_port: int,
        lidar_policy: dict,
        use_host_lidar: bool,
        lidar_motor_dtr_control: bool,
        lidar_motor_dtr_active_high: bool,
        lidar_motor_settle_time: float,
    ) -> None:
        self.device = device
        self.baudrate = baudrate
        self.heartbeat_interval = heartbeat_interval
        self.k210_device = k210_device
        self.k210_baudrate = k210_baudrate
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.lidar_policy = lidar_policy
        self.use_host_lidar = use_host_lidar
        self.lidar_motor_dtr_control = lidar_motor_dtr_control
        self.lidar_motor_dtr_active_high = lidar_motor_dtr_active_high
        self.lidar_motor_settle_time = lidar_motor_settle_time
        self.serial = open_port(device, baudrate)
        self.serial.timeout = min(float(self.serial.timeout or 0.1), 0.02)
        self.parser = FrameParser()
        self.pending_frames = deque()
        self.seq = 1
        self.host_state = 0
        self.current_status: dict | None = None
        self.system_mode = "booting"
        self.paused_by_pickup = False
        self.last_heartbeat_at = 0.0
        self.last_status_refresh_at = 0.0
        self.status_refresh_interval = 1.0
        self.shutdown_started = False
        self.last_event_code = 0
        self.last_event_name = "BOOT"
        self.event_counter = 0
        self.lidar_port = ""
        self.lidar_summary: dict | None = None
        self.lidar_scan: dict | None = None
        self.vision_state: dict | None = None
        self.k210_port = ""
        self.k210_ready = False
        self.k210_last_error = ""
        self.k210_last_rx_at = 0.0
        self.lidar_enabled = bool(self.lidar_policy.get("default_enabled", False))
        self.lidar_required = False
        self.ros_ready = False
        self.ros_ready_reason = "waiting_for_ros"
        self.last_ros_ready_at = 0.0
        self.last_ros_nodes: list[str] = []
        self.last_ros_required_nodes: list[str] = []
        self.host_state_supported = True
        self.k210_worker: HostK210Worker | None = None
        self.lidar_worker: HostLidarWorker | None = None
        self.last_lidar_demand_at = 0.0
        self.last_control_state: dict | None = None
        self.control_counter = 0
        self.last_motion_log: tuple[float, float, int, int] | None = None
        self.motion_test_active_until = 0.0
        self.last_lidar_follow_control_at = 0.0
        self.last_lidar_follow_command: tuple[float, float, str] | None = None
        self.last_lidar_follow_wait_log_at = 0.0
        self.lidar_follow_front_history = deque(maxlen=3)
        self.lidar_follow_motion_state = "hold"
        self.lidar_follow_last_move_x = 0.0
        self.enable_host_lidar_follow = False
        self.last_host_state_sync_at = 0.0
        self.last_host_state_sent: int | None = None
        self.host_state_confirmed = False
        self.state_lock = threading.RLock()
        self.serial_lock = threading.RLock()
        self.backend_server: CoordinatorBackendServer | None = None
        self.backend_thread: threading.Thread | None = None

    def enter_stm32_disconnected_safe_state(self, reason: str = "") -> None:
        reason_text = reason.strip() or "stm32_disconnected"
        log(f"enter safe standby: {reason_text}")
        with self.state_lock:
            self.system_mode = "stm32_disconnected"
            self.paused_by_pickup = False
            self.current_status = None
            self.lidar_required = False
            self.lidar_enabled = False
            self.lidar_summary = None
            self.lidar_scan = None
            self.k210_ready = False
            self.k210_last_error = reason_text
            self.vision_state = None
            self.host_state = HOST_STATE_PI_READY
        self.last_lidar_demand_at = 0.0
        if self.k210_worker is not None:
            self.k210_worker.set_enabled(False)
            _, _, k210_ready, k210_last_error, k210_last_rx_at = self.k210_worker.snapshot()
            with self.state_lock:
                self.k210_ready = k210_ready
                self.k210_last_error = k210_last_error or reason_text
                self.k210_last_rx_at = k210_last_rx_at
        if self.lidar_worker is not None:
            self.lidar_worker.set_enabled(False)
            lidar_port, lidar_summary, lidar_scan, lidar_enabled, _ = self.lidar_worker.snapshot()
            with self.state_lock:
                if lidar_port:
                    self.lidar_port = lidar_port
                self.lidar_enabled = lidar_enabled
                self.lidar_summary = lidar_summary
                self.lidar_scan = lidar_scan

    def close(self) -> None:
        self.stop_backend_server()
        self.enter_stm32_disconnected_safe_state("service_stopping")
        if self.k210_worker is not None:
            self.k210_worker.stop()
        if self.lidar_worker is not None:
            self.lidar_worker.stop()
        try:
            self.serial.close()
        except Exception:
            pass

    def start_backend_server(self) -> None:
        if self.backend_server is not None:
            return
        self.backend_server = CoordinatorBackendServer(
            (self.backend_host, self.backend_port),
            CoordinatorBackendHandler,
            self,
        )
        self.backend_thread = threading.Thread(target=self.backend_server.serve_forever, daemon=True)
        self.backend_thread.start()
        log(f"backend ready tcp://{self.backend_host}:{self.backend_port}")

    def stop_backend_server(self) -> None:
        if self.backend_server is None:
            return
        self.backend_server.shutdown()
        self.backend_server.server_close()
        self.backend_server = None
        self.backend_thread = None

    def next_seq(self) -> int:
        value = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        return value

    def _write_frame_unlocked(self, cmd: int, payload: bytes = b"") -> int:
        seq = self.next_seq()
        self.serial.write(build_frame(cmd, seq, payload))
        self.serial.flush()
        return seq

    def write_frame(self, cmd: int, payload: bytes = b"") -> int:
        with self.serial_lock:
            return self._write_frame_unlocked(cmd, payload)

    def _read_once_unlocked(self, timeout: float):
        if self.pending_frames:
            return self.pending_frames.popleft()

        deadline = time.time() + timeout
        while time.time() < deadline:
            data = self.serial.read(64)
            if not data:
                continue
            for value in data:
                frame = self.parser.feed(value)
                if frame:
                    self.pending_frames.append(frame)
            if self.pending_frames:
                return self.pending_frames.popleft()
        return None

    def read_once(self, timeout: float):
        with self.serial_lock:
            return self._read_once_unlocked(timeout)

    def wait_for_cmd(self, expected_cmd: int, expected_seq: int, timeout: float):
        with self.serial_lock:
            deadline = time.time() + timeout
            while time.time() < deadline:
                frame = self._read_once_unlocked(deadline - time.time())
                if not frame:
                    continue
                self.handle_frame(frame)
                if (
                    frame["ok"]
                    and frame["cmd"] == expected_cmd
                    and frame["seq"] == expected_seq
                ):
                    return frame
        return None

    def send_host_state(self, flags: int, timeout: float = 1.0) -> bool:
        with self.state_lock:
            self.host_state = flags & 0xFF
        if not self.host_state_supported:
            self.host_state_confirmed = True
            self.last_host_state_sent = self.host_state
            return True
        with self.serial_lock:
            seq = self._write_frame_unlocked(CMD_SET_HOST_STATE, bytes([self.host_state]))
            deadline = time.time() + timeout
            while time.time() < deadline:
                frame = self._read_once_unlocked(deadline - time.time())
                if not frame:
                    continue
                self.handle_frame(frame)
                if not frame["ok"]:
                    continue
                payload = frame["payload"]
                if frame["cmd"] == CMD_ACK and len(payload) >= 2:
                    if payload[0] == CMD_SET_HOST_STATE and payload[1] == seq:
                        self.host_state_confirmed = True
                        self.last_host_state_sent = self.host_state
                        self.last_host_state_sync_at = time.time()
                        return True
                if frame["cmd"] == CMD_NACK and len(payload) >= 3:
                    if payload[0] == CMD_SET_HOST_STATE and payload[1] == seq:
                        error_code = payload[2]
                        if error_code == 3:
                            self.host_state_supported = False
                            self.host_state_confirmed = True
                            self.last_host_state_sent = self.host_state
                            self.last_host_state_sync_at = time.time()
                            log("stm32 firmware does not support SET_HOST_STATE, fallback to backend-only readiness flags")
                            return True
                        error_name = ERROR_NAME.get(error_code, str(error_code))
                        log(f"stm32 rejected host state flags=0x{self.host_state:02X} err={error_name}")
                        self.host_state_confirmed = False
                        self.last_host_state_sync_at = time.time()
                        return False
        self.host_state_confirmed = False
        self.last_host_state_sync_at = time.time()
        return False

    def sync_host_state_if_needed(self, force: bool = False, min_interval: float = 1.0) -> bool:
        if not self.host_state_supported:
            self.host_state_confirmed = True
            self.last_host_state_sent = self.host_state
            return True

        now = time.time()
        host_state_changed = self.last_host_state_sent != self.host_state
        sync_due = (now - self.last_host_state_sync_at) >= min_interval
        if not force:
            if self.host_state_confirmed and not host_state_changed:
                return True
            if not host_state_changed and not sync_due and not self.host_state_confirmed:
                return False

        return self.send_host_state(self.host_state, timeout=0.3 if not force else 1.0)

    def query_status(self, timeout: float = 1.0) -> dict | None:
        with self.serial_lock:
            seq = self._write_frame_unlocked(CMD_QUERY_STATUS)
            frame = self.wait_for_cmd(CMD_STATUS, seq, timeout)
            if frame and frame["ok"]:
                with self.state_lock:
                    self.current_status = parse_status_payload(frame["payload"])
        return self.current_status

    def query_vision(self, timeout: float = 1.0) -> dict | None:
        _ = timeout
        self.refresh_k210_state()
        return self.vision_state

    def mode_needs_k210(self, mode_id: int | None) -> bool:
        if K210_ALWAYS_CONNECTED:
            return True
        return mode_id in K210_VISION_MODES

    def desired_k210_remote_mode(self, mode_id: int | None) -> str:
        if mode_id == FOLLOW_MODE_ID:
            return "COLOR"
        return "IDLE"

    def refresh_k210_state(self, force_enabled: bool = False) -> None:
        if self.k210_worker is None:
            return
        mode_id = self.current_mode_id()
        desired_enabled = force_enabled or self.mode_needs_k210(mode_id)
        previous_enabled = self.k210_worker.is_enabled()
        if desired_enabled != previous_enabled:
            log(f"k210 {'enable' if desired_enabled else 'disable'} by mode={mode_id} force={force_enabled}")
        self.k210_worker.set_enabled(desired_enabled)
        if desired_enabled:
            desired_remote_mode = self.desired_k210_remote_mode(mode_id)
            try:
                self.k210_worker.sync_remote_mode(desired_remote_mode)
            except Exception as exc:
                log(f"k210 mode sync failed: {exc}")
        k210_port, vision_state, k210_ready, k210_last_error, k210_last_rx_at = self.k210_worker.snapshot()
        with self.state_lock:
            if k210_port:
                self.k210_port = k210_port
            self.k210_ready = k210_ready
            self.k210_last_error = k210_last_error
            self.k210_last_rx_at = k210_last_rx_at
            if not desired_enabled:
                self.vision_state = None
            elif vision_state is not None:
                self.vision_state = vision_state

    def _request_ack(self, command: int, payload: bytes = b"", timeout: float = 1.0) -> dict:
        with self.serial_lock:
            seq = self._write_frame_unlocked(command, payload)
            deadline = time.time() + timeout
            while time.time() < deadline:
                frame = self._read_once_unlocked(deadline - time.time())
                if not frame:
                    continue
                self.handle_frame(frame)
                if not frame["ok"]:
                    continue
                response_payload = frame["payload"]
                if frame["cmd"] == CMD_ACK and len(response_payload) >= 2:
                    if response_payload[0] == command and response_payload[1] == seq:
                        return {"ok": True, "seq": seq, "cmd": command}
                if frame["cmd"] == CMD_NACK and len(response_payload) >= 3:
                    if response_payload[0] == command and response_payload[1] == seq:
                        error_code = response_payload[2]
                        return {
                            "ok": False,
                            "seq": seq,
                            "cmd": command,
                            "error": "nack",
                            "error_code": error_code,
                            "error_name": ERROR_NAME.get(error_code, str(error_code)),
                        }
        return {"ok": False, "cmd": command, "error": "timeout"}

    def set_mode_command(self, mode_id: int) -> dict:
        response = self._request_ack(CMD_SET_MODE, bytes([mode_id & 0xFF]))
        response["value"] = mode_id
        if response.get("ok"):
            response["car_state"] = self.query_status(timeout=0.3)
            response["vision_state"] = self.query_vision(timeout=0.3)
            self.refresh_lidar_state()
        response["timestamp"] = time.time()
        return response

    def move_matches(self, status: dict | None, move_x: float, move_z: float, tolerance: float = 0.2) -> bool:
        if not status:
            return False
        try:
            current_x = float(status.get("move_x", 0.0) or 0.0)
            current_z = float(status.get("move_z", 0.0) or 0.0)
        except (TypeError, ValueError):
            return False
        return abs(current_x - move_x) <= tolerance and abs(current_z - move_z) <= tolerance

    def set_move_command(self, move_x: float, move_z: float, wait_ack: bool = False) -> dict:
        clamped_x = max(-30.0, min(30.0, float(move_x)))
        clamped_z = max(-450.0, min(450.0, float(move_z)))
        payload = struct.pack("<hh", int(round(clamped_x * 10.0)), int(round(clamped_z * 10.0)))
        if wait_ack:
            response = self._request_ack(CMD_SET_MOVE, payload, timeout=0.8)
            response["move_x"] = clamped_x
            response["move_z"] = clamped_z
            if response.get("ok"):
                response["car_state"] = self.query_status(timeout=0.3)
            elif response.get("error") == "timeout":
                status = self.query_status(timeout=0.5)
                response["car_state"] = status
                if self.move_matches(status, clamped_x, clamped_z):
                    response["ok"] = True
                    response["verified_after_timeout"] = True
                else:
                    retry = self._request_ack(CMD_SET_MOVE, payload, timeout=0.8)
                    response["retry"] = retry
                    if retry.get("ok"):
                        status = self.query_status(timeout=0.5)
                        response["car_state"] = status
                        response["ok"] = True
                        response["verified_after_retry"] = self.move_matches(status, clamped_x, clamped_z)
            response["timestamp"] = time.time()
            response["ack_waited"] = True
            return response

        with self.serial_lock:
            seq = self._write_frame_unlocked(CMD_SET_MOVE, payload)
        car_state = self.current_status or {}
        motion_snapshot = (
            round(clamped_x, 1),
            round(clamped_z, 1),
            int(car_state.get("mode", -1) or -1),
            int(car_state.get("stop_flag", 1) or 0),
        )
        if motion_snapshot != self.last_motion_log:
            log(
                "set_move tx "
                f"x={motion_snapshot[0]:.1f} z={motion_snapshot[1]:.1f} "
                f"mode={motion_snapshot[2]} stop={motion_snapshot[3]} seq={seq}"
            )
            self.last_motion_log = motion_snapshot
        return {
            "ok": True,
            "cmd": CMD_SET_MOVE,
            "seq": seq,
            "move_x": clamped_x,
            "move_z": clamped_z,
            "car_state": self.current_status,
            "timestamp": time.time(),
            "ack_waited": False,
        }

    @staticmethod
    def _distance_from_summary(summary: dict | None, key: str) -> float | None:
        if not summary:
            return None
        try:
            value = float(summary.get(key, 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        return value if value > 0.0 else None

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def reset_lidar_follow_controller(self) -> None:
        self.last_lidar_follow_command = None
        self.lidar_follow_front_history.clear()
        self.lidar_follow_motion_state = "hold"
        self.lidar_follow_last_move_x = 0.0

    def stable_front_distance(self, summary: dict | None) -> tuple[float | None, float | None, float | None]:
        raw_min = self._distance_from_summary(summary, "front_min_distance_m")
        front = (
            self._distance_from_summary(summary, "front_p20_distance_m")
            or self._distance_from_summary(summary, "front_median_distance_m")
            or raw_min
        )
        if front is None:
            self.lidar_follow_front_history.clear()
            return None, raw_min, None

        self.lidar_follow_front_history.append(front)
        filtered_values = sorted(self.lidar_follow_front_history)
        filtered = filtered_values[len(filtered_values) // 2]
        return front, raw_min, filtered

    def run_lidar_follow_control(self) -> None:
        status = self.current_status or {}
        mode_id = int(status.get("mode", -1) or -1)
        if mode_id != LIDAR_FOLLOW_MODE_ID:
            self.last_lidar_follow_command = None
            return
        if self.system_mode != "running":
            self.log_lidar_follow_wait(
                f"not_running system_mode={self.system_mode} stop={status.get('stop_flag')} "
                f"lidar_required={self.lidar_required} lidar_enabled={self.lidar_enabled} "
                f"host_lidar_ready={bool(self.host_state & HOST_STATE_LIDAR_READY)}"
            )
            self.last_lidar_follow_command = None
            return
        if bool(status.get("stop_flag", True)):
            self.log_lidar_follow_wait("stop_flag_asserted")
            self.last_lidar_follow_command = None
            return
        if self.lidar_worker is None:
            self.log_lidar_follow_wait("no_lidar_worker")
            return

        _, summary, _, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
        if not lidar_enabled or not lidar_ready:
            self.log_lidar_follow_wait(f"lidar_not_ready enabled={lidar_enabled} ready={lidar_ready}")
            return

        front = self._distance_from_summary(summary, "front_min_distance_m")
        left = self._distance_from_summary(summary, "front_left_min_distance_m")
        right = self._distance_from_summary(summary, "front_right_min_distance_m")
        move_z = 0.0
        move_x = 0.0
        reason = "hold_position"
        side_blocked = (left is not None and left < 0.18) or (right is not None and right < 0.18)
        if front is not None and 0.0 < front < 0.18:
            move_x = -15.0
            reason = "front_too_close"
        elif front is not None and 0.28 < front < 0.45 and not side_blocked:
            move_x = 15.0
            reason = "front_too_far"
        elif side_blocked:
            reason = "side_too_close"

        now = time.time()
        command = (move_x, move_z, reason)
        if command == self.last_lidar_follow_command and now - self.last_lidar_follow_control_at < 0.12:
            return
        response = self.set_move_command(move_x, move_z, wait_ack=False)
        self.remember_control_state(response)
        self.last_lidar_follow_control_at = now
        self.last_lidar_follow_command = command
        log(
            "auto_lidar_follow "
            f"x={move_x:.1f} z={move_z:.1f} reason={reason} "
            f"front={front if front is not None else -1:.3f} "
            f"left={left if left is not None else -1:.3f} "
            f"right={right if right is not None else -1:.3f}"
        )

    def log_lidar_follow_wait(self, reason: str) -> None:
        now = time.time()
        if now - self.last_lidar_follow_wait_log_at >= 1.0:
            log(f"auto_lidar_follow_wait {reason}")
            self.last_lidar_follow_wait_log_at = now

    def remember_control_state(self, response: dict) -> dict:
        control_state = {
            "control_counter": self.control_counter + 1,
            "ok": bool(response.get("ok")),
            "cmd": response.get("cmd"),
            "value": response.get("value"),
            "move_x": response.get("move_x"),
            "move_z": response.get("move_z"),
            "error": response.get("error"),
            "error_name": response.get("error_name"),
            "timestamp": response.get("timestamp", time.time()),
            "car_state": response.get("car_state"),
        }
        with self.state_lock:
            self.control_counter = control_state["control_counter"]
            self.last_control_state = control_state
        return response

    def motion_control_permitted(self) -> tuple[bool, str]:
        car_state = self.current_status or {}
        if self.shutdown_started:
            return False, "shutdown_started"
        if self.system_mode != "running":
            return False, f"system_mode_{self.system_mode}"
        if not self.ros_ready:
            return False, f"ros_not_ready_{self.ros_ready_reason}"
        if bool(car_state.get("stop_flag", True)):
            return False, "stop_flag_asserted"
        return True, ""

    def update_ros_ready(self, ready: bool, reason: str = "", nodes: list[str] | None = None, required_nodes: list[str] | None = None) -> dict:
        with self.state_lock:
            self.ros_ready = bool(ready)
            self.ros_ready_reason = reason or ("ready" if ready else "not_ready")
            self.last_ros_ready_at = time.time()
            self.last_ros_nodes = list(nodes or [])
            self.last_ros_required_nodes = list(required_nodes or [])
        self.refresh_lidar_state()
        return {
            "ok": True,
            "ros_ready": self.ros_ready,
            "ros_ready_reason": self.ros_ready_reason,
            "last_ros_ready_at": self.last_ros_ready_at,
            "timestamp": time.time(),
        }

    def refresh_ros_ready(self) -> None:
        with self.state_lock:
            if self.last_ros_ready_at <= 0.0:
                self.ros_ready = False
                self.ros_ready_reason = "waiting_for_ros"
                return
            if time.time() - self.last_ros_ready_at > ROS_READY_LEASE_SEC:
                self.ros_ready = False
                self.ros_ready_reason = "ros_heartbeat_timeout"

    def compute_system_ready(self, lidar_ready: bool) -> bool:
        self.refresh_ros_ready()
        return self.ros_ready and (not self.lidar_required or lidar_ready)

    def current_mode_id(self) -> int | None:
        if self.current_status is None:
            return None
        mode = self.current_status.get("mode")
        return int(mode) if mode is not None else None

    def mode_needs_lidar(self, mode_id: int | None) -> bool:
        if mode_id is None:
            return bool(self.lidar_policy.get("default_enabled", False))
        mode_policy = self.lidar_policy.get("modes", {}).get(str(mode_id))
        if mode_policy is None:
            return bool(self.lidar_policy.get("default_enabled", False))
        return bool(mode_policy.get("lidar_enabled", False))

    def refresh_lidar_state(self) -> None:
        mode_id = self.current_mode_id()
        mode_requires_lidar = self.mode_needs_lidar(mode_id)
        with self.state_lock:
            self.lidar_required = mode_requires_lidar

        now = time.time()
        desired_enabled = False
        if not self.shutdown_started:
            if self.system_mode == "running":
                desired_enabled = mode_requires_lidar and (not self.paused_by_pickup or bool(self.lidar_policy.get("keep_enabled_during_pause", False)))
            elif self.system_mode == "mode_select":
                desired_enabled = mode_requires_lidar and bool(self.lidar_policy.get("prewarm_on_mode_select", False))

        if desired_enabled:
            self.last_lidar_demand_at = now
        else:
            grace = float(self.lidar_policy.get("disable_grace_period_sec", 0.0))
            if self.last_lidar_demand_at and now - self.last_lidar_demand_at < grace:
                desired_enabled = True

        if self.lidar_worker is not None:
            previous_enabled = self.lidar_worker.is_enabled()
            if desired_enabled != previous_enabled:
                log(f"lidar {'enable' if desired_enabled else 'disable'} by mode={mode_id} system_mode={self.system_mode}")
            self.lidar_worker.set_enabled(desired_enabled)
            _, lidar_summary, lidar_scan, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
            with self.state_lock:
                self.lidar_enabled = lidar_enabled
                self.lidar_summary = lidar_summary
                self.lidar_scan = lidar_scan
                lidar_bit = HOST_STATE_LIDAR_READY if lidar_ready else 0
                system_ready = self.compute_system_ready(lidar_ready)
                self.host_state = HOST_STATE_PI_READY | lidar_bit | (HOST_STATE_SYSTEM_READY if system_ready else 0)
            return

        with self.state_lock:
            self.lidar_enabled = False
            self.lidar_summary = None
            self.lidar_scan = None
            self.lidar_port = ""
            system_ready = self.compute_system_ready(True)
            self.host_state = HOST_STATE_PI_READY | (HOST_STATE_SYSTEM_READY if system_ready else 0)

    def heartbeat(self) -> None:
        seq = self.write_frame(CMD_HEARTBEAT)
        self.last_heartbeat_at = time.time()
        self.wait_for_cmd(CMD_HEARTBEAT_ACK, seq, 0.3)
        self.sync_host_state_if_needed(min_interval=1.0)

    def handle_frame(self, frame) -> None:
        if not frame["ok"]:
            log(f"drop bad frame cmd=0x{frame['cmd']:02X} seq={frame['seq']}")
            return

        cmd = frame["cmd"]
        payload = frame["payload"]

        if cmd == CMD_STATUS:
            with self.state_lock:
                self.current_status = parse_status_payload(payload)
            return

        if cmd == CMD_EVENT and payload:
            self.handle_event(payload[0])
            return

    def handle_event(self, event_code: int) -> None:
        event_name = EVENT_NAME.get(event_code, f"0x{event_code:02X}")
        with self.state_lock:
            previous_event_code = self.last_event_code
            self.last_event_code = event_code
            self.last_event_name = event_name
            self.event_counter += 1
        log(f"event={event_name}")

        if event_code == EVENT_TIMEOUT_STOP:
            self.paused_by_pickup = False
            self.refresh_lidar_state()
            log("heartbeat timeout stop; keep lidar warm")
            return

        if event_code == EVENT_START_REQUEST:
            self.system_mode = "running"
            status = self.query_status()
            self.refresh_lidar_state()
            if status:
                log(
                    "start mode="
                    f"{status['mode_name']} stop={status['stop_flag']} "
                    f"battery={status['battery']:.2f} angle={status['angle']:.1f}"
                )
            return

        if event_code == EVENT_MODE_SELECT:
            self.system_mode = "mode_select"
            self.paused_by_pickup = False
            self.refresh_lidar_state()
            log("return to mode select")
            return

        if event_code == EVENT_STOP_ASSERT:
            if previous_event_code == EVENT_TIMEOUT_STOP:
                self.paused_by_pickup = False
                self.refresh_lidar_state()
                log("stop asserted after timeout; keep lidar warm")
                return
            self.paused_by_pickup = True
            self.refresh_lidar_state()
            log("pause by stop assert")
            return

        if event_code == EVENT_STOP_CLEAR:
            self.paused_by_pickup = False
            self.refresh_lidar_state()
            log("pickup cleared")
            return

        if event_code == EVENT_SHUTDOWN_REQ:
            self.begin_shutdown()
            return

    def begin_shutdown(self) -> None:
        if self.shutdown_started:
            return
        self.shutdown_started = True
        with self.state_lock:
            self.system_mode = "shutdown"
        log("shutdown requested by stm32")
        self.send_host_state(self.host_state | HOST_STATE_SHUTDOWN_ACK, timeout=1.0)
        time.sleep(1.0)
        subprocess.run(["sudo", "poweroff"], check=False)

    def _cached_state_snapshot(self, include_lidar_scan: bool = False) -> dict:
        with self.state_lock:
            self.refresh_ros_ready()
            car_state = None if self.current_status is None else dict(self.current_status)
            vision_state = None if self.vision_state is None else dict(self.vision_state)
            lidar_summary = None if self.lidar_summary is None else dict(self.lidar_summary)
            lidar_scan = None if self.lidar_scan is None else dict(self.lidar_scan)
            control_state = None if self.last_control_state is None else dict(self.last_control_state)
            return {
                "ok": True,
                "backend_host": self.backend_host,
                "backend_port": self.backend_port,
                "system_mode": self.system_mode,
                "host_state_flags": self.host_state,
                "pi_ready": bool(self.host_state & HOST_STATE_PI_READY),
                "lidar_ready": bool(self.host_state & HOST_STATE_LIDAR_READY),
                "system_ready": bool(self.host_state & HOST_STATE_SYSTEM_READY),
                "shutdown_ack": bool(self.host_state & HOST_STATE_SHUTDOWN_ACK),
                "paused_by_pickup": self.paused_by_pickup,
                "shutdown_started": self.shutdown_started,
                "stop_flag": bool((car_state or {}).get("stop_flag", True)),
                "last_event_code": self.last_event_code,
                "last_event_name": self.last_event_name,
                "event_counter": self.event_counter,
                "stm32_device": self.device,
                "k210_device": self.k210_device,
                "k210_port": self.k210_port,
                "k210_ready": self.k210_ready,
                "k210_last_error": self.k210_last_error,
                "k210_last_rx_at": self.k210_last_rx_at,
                "lidar_port": self.lidar_port,
                "lidar_enabled": self.lidar_enabled,
                "lidar_required": self.lidar_required,
                "ros_ready": self.ros_ready,
                "ros_ready_reason": self.ros_ready_reason,
                "last_ros_ready_at": self.last_ros_ready_at,
                "ros_nodes": list(self.last_ros_nodes),
                "ros_required_nodes": list(self.last_ros_required_nodes),
                "lidar_summary": lidar_summary,
                "lidar_scan": lidar_scan if include_lidar_scan else None,
                "vision_state": vision_state,
                "car_state": car_state,
                "control_state": control_state,
                "control_counter": self.control_counter,
                "host_state_supported": self.host_state_supported,
                "control_backend_ready": True,
                "timestamp": time.time(),
            }

    def snapshot_state(self, include_lidar_scan: bool = False) -> dict:
        return self._cached_state_snapshot(include_lidar_scan=include_lidar_scan)

    def handle_backend_request(self, request: dict) -> dict:
        command = request.get("cmd", "get_state")

        if command == "ping":
            return {"ok": True, "reply": "pong", "timestamp": time.time()}

        if command == "get_state":
            return self.snapshot_state()

        if command == "begin_motion_test":
            duration = max(1.0, min(60.0, float(request.get("duration_sec", 20.0) or 20.0)))
            self.motion_test_active_until = time.time() + duration
            log(f"motion test lock enabled duration={duration:.1f}s")
            return {
                "ok": True,
                "cmd": command,
                "duration_sec": duration,
                "active_until": self.motion_test_active_until,
                "timestamp": time.time(),
            }

        if command == "end_motion_test":
            self.motion_test_active_until = 0.0
            log("motion test lock disabled")
            return {"ok": True, "cmd": command, "timestamp": time.time()}

        if command == "get_lidar_scan":
            if self.lidar_worker is not None:
                lidar_port, lidar_summary, lidar_scan, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
                with self.state_lock:
                    self.lidar_port = lidar_port or self.lidar_port
                    self.lidar_enabled = lidar_enabled
                    self.lidar_summary = lidar_summary
                    self.lidar_scan = lidar_scan
                    if lidar_ready:
                        self.host_state |= HOST_STATE_LIDAR_READY
                    else:
                        self.host_state &= ~HOST_STATE_LIDAR_READY
                    if self.compute_system_ready(lidar_ready):
                        self.host_state |= HOST_STATE_SYSTEM_READY
                    else:
                        self.host_state &= ~HOST_STATE_SYSTEM_READY
            snapshot = self.snapshot_state(include_lidar_scan=True)
            return {
                "ok": True,
                "lidar_enabled": snapshot.get("lidar_enabled", False),
                "lidar_ready": snapshot.get("lidar_ready", False),
                "lidar_port": snapshot.get("lidar_port", ""),
                "lidar_summary": snapshot.get("lidar_summary"),
                "lidar_scan": snapshot.get("lidar_scan"),
                "timestamp": snapshot.get("timestamp", time.time()),
            }

        if command == "get_lidar_summary":
            if self.lidar_worker is not None:
                lidar_port, lidar_summary, _, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
                with self.state_lock:
                    self.lidar_port = lidar_port or self.lidar_port
                    self.lidar_enabled = lidar_enabled
                    self.lidar_summary = lidar_summary
                    if lidar_ready:
                        self.host_state |= HOST_STATE_LIDAR_READY
                    else:
                        self.host_state &= ~HOST_STATE_LIDAR_READY
                    if self.compute_system_ready(lidar_ready):
                        self.host_state |= HOST_STATE_SYSTEM_READY
                    else:
                        self.host_state &= ~HOST_STATE_SYSTEM_READY
            snapshot = self.snapshot_state(include_lidar_scan=False)
            return {
                "ok": True,
                "lidar_enabled": snapshot.get("lidar_enabled", False),
                "lidar_ready": snapshot.get("lidar_ready", False),
                "lidar_port": snapshot.get("lidar_port", ""),
                "lidar_summary": snapshot.get("lidar_summary"),
                "timestamp": snapshot.get("timestamp", time.time()),
            }

        if command == "get_vision":
            self.refresh_k210_state()
            return {"ok": True, "vision_state": self.vision_state, "timestamp": time.time()}

        if command == "get_k210_link":
            self.refresh_k210_state()
            return {
                "ok": True,
                "k210_device": self.k210_device,
                "k210_port": self.k210_port,
                "k210_ready": self.k210_ready,
                "k210_last_error": self.k210_last_error,
                "k210_last_rx_at": self.k210_last_rx_at,
                "vision_state": self.vision_state,
                "timestamp": time.time(),
            }

        if command == "set_k210_text":
            text = sanitize_k210_text(str(request.get("text", "")))
            if not text:
                return {"ok": False, "error": "bad_value", "cmd": command}
            if self.k210_worker is None:
                return {"ok": False, "error": "k210_unavailable", "cmd": command}
            try:
                self.refresh_k210_state(force_enabled=True)
                self.k210_worker.send_text(text)
            except Exception as exc:
                return {"ok": False, "error": "k210_write_failed", "cmd": command, "message": str(exc)}
            return {"ok": True, "cmd": command, "text": text, "timestamp": time.time()}

        if command == "set_ros_ready":
            ready = bool(request.get("ready", False))
            reason = str(request.get("reason", ""))
            nodes = request.get("nodes") if isinstance(request.get("nodes"), list) else []
            required_nodes = request.get("required_nodes") if isinstance(request.get("required_nodes"), list) else []
            return self.update_ros_ready(ready, reason=reason, nodes=nodes, required_nodes=required_nodes)

        if command == "set_mode":
            value = request.get("value")
            if not isinstance(value, int):
                return {"ok": False, "error": "bad_value", "cmd": command}
            if value not in SUPPORTED_MODE_IDS:
                return {
                    "ok": False,
                    "error": "unsupported_mode",
                    "cmd": command,
                    "value": value,
                    "message": f"supported modes are {sorted(SUPPORTED_MODE_IDS)}",
                    "timestamp": time.time(),
                }
            return self.remember_control_state(self.set_mode_command(value))

        if command == "set_move":
            try:
                move_x = float(request.get("move_x", 0.0))
                move_z = float(request.get("move_z", 0.0))
            except (TypeError, ValueError):
                return {"ok": False, "error": "bad_value", "cmd": command}
            wait_ack = bool(request.get("wait_ack", False))
            source = str(request.get("source", "") or "")
            if time.time() < self.motion_test_active_until and source != "motion_smoke_test":
                return {
                    "ok": True,
                    "cmd": command,
                    "move_x": move_x,
                    "move_z": move_z,
                    "car_state": self.current_status,
                    "timestamp": time.time(),
                    "ignored_by_motion_test": True,
                }
            mode_id = int((self.current_status or {}).get("mode", -1) or -1)
            if self.enable_host_lidar_follow and self.system_mode == "running" and mode_id == LIDAR_FOLLOW_MODE_ID and source != "motion_smoke_test":
                return {
                    "ok": True,
                    "cmd": command,
                    "move_x": move_x,
                    "move_z": move_z,
                    "car_state": self.current_status,
                    "timestamp": time.time(),
                    "ignored_by_auto_lidar_follow": True,
                }
            permitted, reason = self.motion_control_permitted()
            if not permitted:
                return {
                    "ok": False,
                    "error": "forbidden",
                    "cmd": command,
                    "message": f"motion command rejected: {reason}",
                    "timestamp": time.time(),
                }
            return self.remember_control_state(self.set_move_command(move_x, move_z, wait_ack=wait_ack))

        if command in ("set_enable", "stop"):
            return {
                "ok": False,
                "error": "forbidden",
                "cmd": command,
                "message": "stop_flag is owned by stm32 only",
                "timestamp": time.time(),
            }

        return {"ok": False, "error": "unsupported_cmd", "cmd": command}

    def run(self, sdk_root: Path, lidar_device: str | None, lidar_baudrate: int, scan_frequency: float) -> int:
        log(f"stm32 device={self.device} baudrate={self.baudrate}")
        self.start_backend_server()
        self.k210_worker = HostK210Worker(
            preferred_device=self.k210_device,
            baudrate=self.k210_baudrate,
            mode_provider=self.current_mode_id,
        )
        self.k210_worker.start()
        startup_retries = 0
        max_startup_retries = 3
        while startup_retries < max_startup_retries and not self.send_host_state(HOST_STATE_PI_READY):
            log("stm32 not ready for PI_READY, retry in 1s")
            startup_retries += 1
            time.sleep(1.0)
        if startup_retries >= max_startup_retries:
            with self.state_lock:
                self.host_state = HOST_STATE_PI_READY
            self.host_state_confirmed = False
            self.last_host_state_sent = None
            self.last_host_state_sync_at = 0.0
            log("continue without blocking on PI_READY; will keep syncing host state in background")
        else:
            log("reported PI_READY")

        if self.use_host_lidar:
            self.lidar_worker = HostLidarWorker(
                sdk_root=sdk_root,
                preferred_device=lidar_device,
                baudrate=lidar_baudrate,
                scan_frequency=scan_frequency,
                motor_dtr_control=self.lidar_motor_dtr_control,
                motor_dtr_active_high=self.lidar_motor_dtr_active_high,
                motor_settle_time=self.lidar_motor_settle_time,
            )
            self.lidar_worker.start()
        else:
            log("host lidar worker disabled; lidar ownership moved to ROS native node")
        self.system_mode = "mode_select"
        self.query_status(timeout=1.0)
        self.refresh_k210_state()
        self.refresh_lidar_state()
        log("reported SYSTEM_READY")

        self.last_heartbeat_at = time.time()
        self.last_status_refresh_at = time.time()

        try:
            while not self.shutdown_started:
                if time.time() - self.last_heartbeat_at >= self.heartbeat_interval:
                    self.heartbeat()

                if time.time() - self.last_status_refresh_at >= self.status_refresh_interval:
                    self.query_status(timeout=0.2)
                    self.refresh_k210_state()
                    self.last_status_refresh_at = time.time()
                    self.refresh_lidar_state()
                    self.sync_host_state_if_needed(min_interval=1.0)
                    if self.lidar_worker is not None:
                        lidar_port, lidar_summary, lidar_scan, lidar_enabled, _ = self.lidar_worker.snapshot()
                        with self.state_lock:
                            if lidar_port:
                                self.lidar_port = lidar_port
                            self.lidar_enabled = lidar_enabled
                            self.lidar_summary = lidar_summary
                            self.lidar_scan = lidar_scan

                frame = self.read_once(0.05)
                if frame:
                    self.handle_frame(frame)
                if self.enable_host_lidar_follow:
                    self.run_lidar_follow_control()
        except Exception as exc:
            self.enter_stm32_disconnected_safe_state(str(exc))
            raise

        return 0


def stm32_candidate_score(path: Path) -> int:
    name = path.name.lower()
    score = 0
    if "1a86" in name or "ch340" in name or "ftdi" in name:
        score += 100
    if "silicon_labs" in name or "cp210" in name or "ydlidar" in name:
        score -= 80
    return score


def detect_stm32_port(preferred: str | None) -> str:
    if preferred:
        return preferred

    serial_by_id = Path("/dev/serial/by-id")
    if serial_by_id.exists():
        candidates = sorted(serial_by_id.iterdir())
        if candidates:
            best = max(candidates, key=stm32_candidate_score)
            if stm32_candidate_score(best) > 0:
                return str(best)
            return str(candidates[0])

    for candidate in sorted(Path("/dev").glob("ttyUSB*")):
        return str(candidate)

    raise RuntimeError("No STM32 serial device found.")


def k210_candidate_score(path: Path) -> int:
    name = path.name.lower()
    score = 0
    if "silicon_labs" in name or "cp210" in name:
        score += 80
    if "ydlidar" in name:
        score -= 100
    return score


def detect_k210_port(preferred: str | None) -> str:
    if preferred:
        return preferred

    serial_by_id = Path("/dev/serial/by-id")
    if serial_by_id.exists():
        candidates = sorted(serial_by_id.iterdir())
        filtered = [item for item in candidates if "ydlidar" not in item.name.lower()]
        if filtered:
            best = max(filtered, key=k210_candidate_score)
            if k210_candidate_score(best) > 0:
                return str(best)

    for candidate in sorted(Path("/dev").glob("ttyUSB*")):
        return str(candidate)

    raise RuntimeError("No K210 serial device found.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Raspberry Pi coordinator for STM32 balance car")
    parser.add_argument("--stm32-device", default=None, help="STM32 serial device path, auto-detect if omitted")
    parser.add_argument("--stm32-baudrate", type=int, default=921600)
    parser.add_argument("--heartbeat-interval", type=float, default=0.25)
    parser.add_argument("--k210-device", default=None, help="K210 serial device path, auto-detect if omitted")
    parser.add_argument("--k210-baudrate", type=int, default=115200)
    parser.add_argument("--lidar-device", default=None, help="Lidar serial device path, auto-detect if omitted")
    parser.add_argument("--lidar-baudrate", type=int, default=230400)
    parser.add_argument("--lidar-scan-frequency", type=float, default=10.0)
    parser.add_argument("--backend-host", default=BACKEND_DEFAULT_HOST)
    parser.add_argument("--backend-port", type=int, default=BACKEND_DEFAULT_PORT)
    parser.add_argument("--sdk-root", default=str(resolve_sdk_root()))
    parser.add_argument("--lidar-policy-config", default=str(DEFAULT_LIDAR_POLICY_PATH))
    parser.add_argument("--host-lidar", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lidar-motor-dtr-control", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lidar-motor-dtr-active-high", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lidar-motor-settle-time", type=float, default=0.2)
    args = parser.parse_args()

    stm32_device = detect_stm32_port(args.stm32_device)
    sdk_root = resolve_sdk_root(args.sdk_root)
    lidar_policy = load_lidar_policy(Path(args.lidar_policy_config).expanduser())

    coordinator = STM32Coordinator(
        device=stm32_device,
        baudrate=args.stm32_baudrate,
        heartbeat_interval=args.heartbeat_interval,
        k210_device=args.k210_device,
        k210_baudrate=args.k210_baudrate,
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        lidar_policy=lidar_policy,
        use_host_lidar=args.host_lidar,
        lidar_motor_dtr_control=args.lidar_motor_dtr_control,
        lidar_motor_dtr_active_high=args.lidar_motor_dtr_active_high,
        lidar_motor_settle_time=args.lidar_motor_settle_time,
    )
    try:
        return coordinator.run(
            sdk_root=sdk_root,
            lidar_device=args.lidar_device,
            lidar_baudrate=args.lidar_baudrate,
            scan_frequency=args.lidar_scan_frequency,
        )
    finally:
        coordinator.close()


if __name__ == "__main__":
    raise SystemExit(main())
