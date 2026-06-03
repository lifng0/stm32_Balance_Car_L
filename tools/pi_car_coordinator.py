#!/usr/bin/env python3
import argparse
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
    CMD_QUERY_VISION,
    CMD_QUERY_STATUS,
    CMD_SET_HOST_STATE,
    CMD_STATUS,
    CMD_VISION_STATUS,
    ERROR_NAME,
    EVENT_NAME,
    FrameParser,
    build_frame,
    open_port,
)
from tminiplus_bridge import build_laser, detect_port, load_sdk, resolve_sdk_root, summarize_scan


HOST_STATE_PI_READY = 0x01
HOST_STATE_LIDAR_READY = 0x02
HOST_STATE_SYSTEM_READY = 0x04
HOST_STATE_SHUTDOWN_ACK = 0x08

BACKEND_DEFAULT_HOST = "127.0.0.1"
BACKEND_DEFAULT_PORT = 8765

EVENT_START_REQUEST = 0x10
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
    2: "K210_QR",
    3: "K210_Line",
    4: "K210_Follow",
    5: "K210_SelfLearn",
    6: "K210_mnist",
}

DEFAULT_LIDAR_POLICY = {
    "default_enabled": False,
    "prewarm_on_mode_select": True,
    "keep_enabled_during_pause": False,
    "disable_grace_period_sec": 3.0,
    "modes": {
        "0": {"name": "Normal", "lidar_enabled": False},
        "1": {"name": "Weight_M", "lidar_enabled": False},
        "2": {"name": "K210_QR", "lidar_enabled": False},
        "3": {"name": "K210_Line", "lidar_enabled": False},
        "4": {"name": "K210_Follow", "lidar_enabled": False},
        "5": {"name": "K210_SelfLearn", "lidar_enabled": False},
        "6": {"name": "K210_mnist", "lidar_enabled": False},
        "7": {"name": "Lidar_Avoid", "lidar_enabled": True},
        "8": {"name": "Lidar_Follow", "lidar_enabled": True},
        "9": {"name": "Lidar_SLAM", "lidar_enabled": True},
    },
}
DEFAULT_LIDAR_POLICY_PATH = Path.home() / "workspace" / "balance_car" / "scripts" / "lidar_mode_policy.json"


def log(message: str) -> None:
    print(f"[pi-coordinator] {message}", flush=True)


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
                self._last_error = ""
                self._ready_event.clear()
        if not enabled:
            self._close_laser(clear_summary=True)

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def snapshot(self) -> tuple[str, dict | None, bool, bool]:
        with self._lock:
            return (
                self._port,
                None if self._summary is None else dict(self._summary),
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
                with self._lock:
                    self._summary = summary
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

    def __init__(self, server_address, request_handler_class, coordinator):
        self.coordinator = coordinator
        super().__init__(server_address, request_handler_class)


class CoordinatorBackendHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        raw = self.rfile.readline()
        if not raw:
            return

        try:
            request = json.loads(raw.decode("utf-8"))
        except Exception:
            response = {"ok": False, "error": "bad_json"}
        else:
            response = self.server.coordinator.handle_backend_request(request)

        self.wfile.write((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))


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


class STM32Coordinator:
    def __init__(
        self,
        device: str,
        baudrate: int,
        heartbeat_interval: float,
        backend_host: str,
        backend_port: int,
        lidar_policy: dict,
        lidar_motor_dtr_control: bool,
        lidar_motor_dtr_active_high: bool,
        lidar_motor_settle_time: float,
    ) -> None:
        self.device = device
        self.baudrate = baudrate
        self.heartbeat_interval = heartbeat_interval
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.lidar_policy = lidar_policy
        self.lidar_motor_dtr_control = lidar_motor_dtr_control
        self.lidar_motor_dtr_active_high = lidar_motor_dtr_active_high
        self.lidar_motor_settle_time = lidar_motor_settle_time
        self.serial = open_port(device, baudrate)
        self.parser = FrameParser()
        self.seq = 1
        self.host_state = 0
        self.current_status: dict | None = None
        self.system_mode = "booting"
        self.paused_by_pickup = False
        self.last_heartbeat_at = 0.0
        self.last_status_refresh_at = 0.0
        self.status_refresh_interval = 0.5
        self.shutdown_started = False
        self.last_event_code = 0
        self.last_event_name = "BOOT"
        self.event_counter = 0
        self.lidar_port = ""
        self.lidar_summary: dict | None = None
        self.vision_state: dict | None = None
        self.lidar_enabled = bool(self.lidar_policy.get("default_enabled", False))
        self.lidar_required = False
        self.host_state_supported = True
        self.lidar_worker: HostLidarWorker | None = None
        self.last_lidar_demand_at = 0.0
        self.state_lock = threading.Lock()
        self.backend_server: CoordinatorBackendServer | None = None
        self.backend_thread: threading.Thread | None = None

    def close(self) -> None:
        self.stop_backend_server()
        if self.lidar_worker is not None:
            self.lidar_worker.stop()
        self.serial.close()

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

    def write_frame(self, cmd: int, payload: bytes = b"") -> int:
        seq = self.next_seq()
        self.serial.write(build_frame(cmd, seq, payload))
        self.serial.flush()
        return seq

    def read_once(self, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            data = self.serial.read(64)
            if not data:
                continue
            for value in data:
                frame = self.parser.feed(value)
                if frame:
                    return frame
        return None

    def wait_for_cmd(self, expected_cmd: int, expected_seq: int, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self.read_once(deadline - time.time())
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
            return True
        seq = self.write_frame(CMD_SET_HOST_STATE, bytes([self.host_state]))
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self.read_once(deadline - time.time())
            if not frame:
                continue
            self.handle_frame(frame)
            if not frame["ok"]:
                continue
            payload = frame["payload"]
            if frame["cmd"] == CMD_ACK and len(payload) >= 2:
                if payload[0] == CMD_SET_HOST_STATE and payload[1] == seq:
                    return True
            if frame["cmd"] == CMD_NACK and len(payload) >= 3:
                if payload[0] == CMD_SET_HOST_STATE and payload[1] == seq:
                    error_code = payload[2]
                    if error_code == 3:
                        self.host_state_supported = False
                        log("stm32 firmware does not support SET_HOST_STATE, fallback to backend-only readiness flags")
                        return True
                    error_name = ERROR_NAME.get(error_code, str(error_code))
                    log(f"stm32 rejected host state flags=0x{self.host_state:02X} err={error_name}")
                    return False
        return False

    def query_status(self, timeout: float = 1.0) -> dict | None:
        seq = self.write_frame(CMD_QUERY_STATUS)
        frame = self.wait_for_cmd(CMD_STATUS, seq, timeout)
        if frame and frame["ok"]:
            with self.state_lock:
                self.current_status = parse_status_payload(frame["payload"])
        return self.current_status

    def query_vision(self, timeout: float = 1.0) -> dict | None:
        seq = self.write_frame(CMD_QUERY_VISION)
        frame = self.wait_for_cmd(CMD_VISION_STATUS, seq, timeout)
        if frame and frame["ok"]:
            with self.state_lock:
                self.vision_state = parse_vision_payload(frame["payload"])
        return self.vision_state

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
            _, lidar_summary, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
            with self.state_lock:
                self.lidar_enabled = lidar_enabled
                self.lidar_summary = lidar_summary
                pi_ready = True
                lidar_bit = HOST_STATE_LIDAR_READY if lidar_ready else 0
                system_ready = pi_ready and (not self.lidar_required or lidar_ready)
                self.host_state = HOST_STATE_PI_READY | lidar_bit | (HOST_STATE_SYSTEM_READY if system_ready else 0)

    def heartbeat(self) -> None:
        seq = self.write_frame(CMD_HEARTBEAT)
        self.wait_for_cmd(CMD_HEARTBEAT_ACK, seq, 0.3)
        self.last_heartbeat_at = time.time()

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
            self.last_event_code = event_code
            self.last_event_name = event_name
            self.event_counter += 1
        log(f"event={event_name}")

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
            self.paused_by_pickup = True
            self.refresh_lidar_state()
            log("pause by pickup")
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

    def snapshot_state(self) -> dict:
        if self.lidar_worker is not None:
            lidar_port, lidar_summary, lidar_enabled, lidar_ready = self.lidar_worker.snapshot()
            with self.state_lock:
                self.lidar_port = lidar_port or self.lidar_port
                self.lidar_enabled = lidar_enabled
                self.lidar_summary = lidar_summary
                if lidar_ready:
                    self.host_state |= HOST_STATE_LIDAR_READY
                else:
                    self.host_state &= ~HOST_STATE_LIDAR_READY
                if not self.lidar_required or lidar_ready:
                    self.host_state |= HOST_STATE_SYSTEM_READY
                else:
                    self.host_state &= ~HOST_STATE_SYSTEM_READY
        with self.state_lock:
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
                "last_event_code": self.last_event_code,
                "last_event_name": self.last_event_name,
                "event_counter": self.event_counter,
                "stm32_device": self.device,
                "lidar_port": self.lidar_port,
                "lidar_enabled": self.lidar_enabled,
                "lidar_required": self.lidar_required,
                "lidar_summary": self.lidar_summary,
                "vision_state": self.vision_state,
                "car_state": self.current_status,
                "host_state_supported": self.host_state_supported,
                "timestamp": time.time(),
            }

    def handle_backend_request(self, request: dict) -> dict:
        command = request.get("cmd", "get_state")

        if command == "ping":
            return {"ok": True, "reply": "pong", "timestamp": time.time()}

        if command == "get_state":
            return self.snapshot_state()

        if command == "get_vision":
            return {"ok": True, "vision_state": self.vision_state, "timestamp": time.time()}

        return {"ok": False, "error": "unsupported_cmd", "cmd": command}

    def run(self, sdk_root: Path, lidar_device: str | None, lidar_baudrate: int, scan_frequency: float) -> int:
        log(f"stm32 device={self.device} baudrate={self.baudrate}")
        self.start_backend_server()
        while not self.send_host_state(HOST_STATE_PI_READY):
            log("stm32 not ready for PI_READY, retry in 1s")
            time.sleep(1.0)
        log("reported PI_READY")

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
        self.system_mode = "mode_select"
        self.query_status(timeout=1.0)
        self.query_vision(timeout=1.0)
        self.refresh_lidar_state()
        log("reported SYSTEM_READY")

        self.last_heartbeat_at = time.time()
        self.last_status_refresh_at = time.time()

        while not self.shutdown_started:
            if time.time() - self.last_heartbeat_at >= self.heartbeat_interval:
                self.heartbeat()

            if time.time() - self.last_status_refresh_at >= self.status_refresh_interval:
                self.query_status(timeout=0.2)
                self.query_vision(timeout=0.2)
                self.last_status_refresh_at = time.time()
                self.refresh_lidar_state()
                if self.lidar_worker is not None:
                    lidar_port, lidar_summary, lidar_enabled, _ = self.lidar_worker.snapshot()
                    with self.state_lock:
                        if lidar_port:
                            self.lidar_port = lidar_port
                        self.lidar_enabled = lidar_enabled
                        self.lidar_summary = lidar_summary

            frame = self.read_once(0.05)
            if frame:
                self.handle_frame(frame)

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


def main() -> int:
    parser = argparse.ArgumentParser(description="Raspberry Pi coordinator for STM32 balance car")
    parser.add_argument("--stm32-device", default=None, help="STM32 serial device path, auto-detect if omitted")
    parser.add_argument("--stm32-baudrate", type=int, default=115200)
    parser.add_argument("--heartbeat-interval", type=float, default=0.1)
    parser.add_argument("--lidar-device", default=None, help="Lidar serial device path, auto-detect if omitted")
    parser.add_argument("--lidar-baudrate", type=int, default=230400)
    parser.add_argument("--lidar-scan-frequency", type=float, default=10.0)
    parser.add_argument("--backend-host", default=BACKEND_DEFAULT_HOST)
    parser.add_argument("--backend-port", type=int, default=BACKEND_DEFAULT_PORT)
    parser.add_argument("--sdk-root", default=str(resolve_sdk_root()))
    parser.add_argument("--lidar-policy-config", default=str(DEFAULT_LIDAR_POLICY_PATH))
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
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        lidar_policy=lidar_policy,
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
