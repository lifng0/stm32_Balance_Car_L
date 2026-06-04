#!/usr/bin/env python3
"""
Balance Car Interface Verification Tool

Usage:
  # Full verification (with safety confirmation for movement tests)
  python3 tools/verify_car_interfaces.py --coordinator-host 127.0.0.1 --coordinator-port 8765

  # Quick backend-only check (no ROS 2, no serial)
  python3 tools/verify_car_interfaces.py --quick

  # Check specific layer only
  python3 tools/verify_car_interfaces.py --check-backend
  python3 tools/verify_car_interfaces.py --check-ros-topics
  python3 tools/verify_car_interfaces.py --check-serial --serial-device /dev/serial/by-id/xxx
"""

import argparse
import json
import socket
import subprocess
import sys
import textwrap
import time


# ── ANSI Colors ──────────────────────────────────────────────────────────────
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def ok(msg: str) -> str:
    return f"{GREEN}[PASS]{RESET} {msg}"


def fail(msg: str) -> str:
    return f"{RED}[FAIL]{RESET} {msg}"


def warn(msg: str) -> str:
    return f"{YELLOW}[WARN]{RESET} {msg}"


def info(msg: str) -> str:
    return f"{CYAN}[INFO]{RESET} {msg}"


# ── Helpers ──────────────────────────────────────────────────────────────────


def request_backend(host: str, port: int, payload: dict, timeout: float = 2.0) -> dict | None:
    try:
        with socket.create_connection((host, port), timeout=timeout) as conn:
            conn.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
            conn.shutdown(socket.SHUT_WR)
            chunks = []
            while True:
                data = conn.recv(65535)
                if not data:
                    break
                chunks.append(data)
        raw = b"".join(chunks).decode("utf-8").strip()
        if not raw:
            return None
        return json.loads(raw)
    except (socket.timeout, ConnectionRefusedError, OSError, json.JSONDecodeError) as exc:
        return {"_error": str(exc)}


def check_dict_fields(data: dict, required_fields: list[str]) -> list[str]:
    missing = [f for f in required_fields if f not in data]
    return missing


# ── Section Checks ───────────────────────────────────────────────────────────


def check_backend_connection(host: str, port: int) -> tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=3.0) as conn:
            conn.sendall(b'{"cmd":"get_state"}\n')
            conn.shutdown(socket.SHUT_WR)
            raw = conn.recv(65535)
            if raw:
                return True, f"connected to {host}:{port}, got {len(raw)} bytes response"
            return False, "connected but empty response"
    except ConnectionRefusedError:
        return False, f"Connection refused on {host}:{port} — coordinator not running?"
    except socket.timeout:
        return False, f"Connection timed out on {host}:{port}"
    except OSError as exc:
        return False, str(exc)


def check_get_state(host: str, port: int) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "get_state"}, timeout=3.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp:
        return False, resp["_error"]
    required = ["system_mode", "pi_ready", "system_ready", "car_state"]
    missing = check_dict_fields(resp, required)
    if missing:
        return False, f"missing fields: {missing}"
    state = resp.get("car_state", {})
    mode_name = state.get("mode_name", "?")
    pi_ready = resp.get("pi_ready", False)
    sys_ready = resp.get("system_ready", False)
    lidar_ready = resp.get("lidar_ready", False)
    battery = state.get("battery", "?")
    angle = state.get("angle", "?")
    return True, (
        f"pi_ready={pi_ready} system_ready={sys_ready} lidar_ready={lidar_ready} "
        f"mode={mode_name} battery={battery}V angle={angle}"
    )


def check_get_vision(host: str, port: int) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "get_vision"}, timeout=3.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp:
        return False, resp["_error"]
    return True, f"vision data: {json.dumps(resp, ensure_ascii=False)}"


def check_backend_stop(host: str, port: int) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "stop"}, timeout=2.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp and "refused" in resp["_error"].lower():
        return False, "coordinator not running"
    if isinstance(resp, dict) and resp.get("ok"):
        return True, "stop command accepted"
    if isinstance(resp, dict) and resp.get("result") == "ok":
        return True, "stop command accepted"
    return True, f"response: {json.dumps(resp, ensure_ascii=False)}"


def check_backend_enable(host: str, port: int, value: bool) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "set_enable", "value": 1 if value else 0}, timeout=2.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp:
        return False, resp["_error"]
    if isinstance(resp, dict) and resp.get("ok"):
        return True, f"set_enable({value}) accepted"
    return True, f"response: {json.dumps(resp, ensure_ascii=False)}"


def check_backend_move_zero(host: str, port: int) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "set_move", "move_x": 0.0, "move_z": 0.0}, timeout=2.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp:
        return False, resp["_error"]
    return True, f"set_move(0, 0) accepted: {json.dumps(resp, ensure_ascii=False)}"


def check_lidar_summary_via_backend(host: str, port: int) -> tuple[bool, str]:
    resp = request_backend(host, port, {"cmd": "get_state"}, timeout=3.0)
    if resp is None:
        return False, "no response"
    if "_error" in resp:
        return False, resp["_error"]
    summary = resp.get("lidar_summary")
    if not summary:
        lidar_ready = resp.get("lidar_ready", False)
        lidar_enabled = resp.get("lidar_enabled", False)
        return False, f"no lidar_summary in state (lidar_ready={lidar_ready}, lidar_enabled={lidar_enabled})"
    front = summary.get("front_min_distance_m", "?")
    fl = summary.get("front_left_min_distance_m", "?")
    fr = summary.get("front_right_min_distance_m", "?")
    closest_angle = summary.get("closest_target_angle_deg", "?")
    closest_dist = summary.get("closest_target_distance_m", "?")
    return True, (
        f"front={front}m front_left={fl}m front_right={fr}m "
        f"closest_angle={closest_angle} closest_dist={closest_dist}m"
    )


# ── ROS 2 Topic Checks ──────────────────────────────────────────────────────


def ros2_topic_echo(topic: str, timeout: float = 5.0) -> tuple[bool, str]:
    try:
        proc = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once", "--timeout", str(int(timeout))],
            capture_output=True,
            text=True,
            timeout=timeout + 2.0,
        )
        if proc.returncode != 0:
            stderr = proc.stderr.strip()
            if "not yet registered" in stderr.lower() or "unknown topic" in stderr.lower():
                return False, f"topic not available (not yet registered)"
            if "timeout" in stderr.lower():
                return False, f"no message received within {timeout}s"
            return False, stderr[:200]
        output = proc.stdout.strip()
        if not output:
            return False, "empty topic data"
        return True, output[:200]
    except FileNotFoundError:
        return False, "ros2 command not found (ROS 2 not sourced?)"
    except subprocess.TimeoutExpired:
        return False, "ros2 topic echo timed out"


def check_ros2_topics(timeout: float) -> list[tuple[str, bool, str]]:
    topics_to_check = [
        "/car/system_state_json",
        "/car/state_json",
        "/lidar/summary_json",
        "/car/event_json",
        "/behavior/status_json",
        "/vision/status_json",
        "/car/control_json",
    ]
    results = []
    for topic in topics_to_check:
        ok_flag, msg = ros2_topic_echo(topic, timeout)
        results.append((topic, ok_flag, msg))
        time.sleep(0.2)
    return results


# ── Serial / pi_serial_bridge Checks ────────────────────────────────────────


def check_serial_ping(device: str, bridge_path: str) -> tuple[bool, str]:
    try:
        proc = subprocess.run(
            ["python3", bridge_path, "--device", device, "ping"],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        if proc.returncode != 0:
            return False, proc.stderr.strip()[:200] or f"exit code {proc.returncode}"
        output = proc.stdout.strip()
        if "ACK" in output:
            return True, output[:200]
        return False, output[:200]
    except FileNotFoundError:
        return False, "python3 not found"
    except subprocess.TimeoutExpired:
        return False, "ping timed out (no response from STM32)"


def check_serial_status(device: str, bridge_path: str) -> tuple[bool, str]:
    try:
        proc = subprocess.run(
            ["python3", bridge_path, "--device", device, "status"],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        if proc.returncode != 0:
            return False, proc.stderr.strip()[:200] or f"exit code {proc.returncode}"
        output = proc.stdout.strip()
        if "STATUS" in output:
            return True, output[:200]
        return False, output[:200]
    except subprocess.TimeoutExpired:
        return False, "status query timed out"


# ── Coordinator service check ──────────────────────────────────────────────


def check_coordinator_service() -> tuple[bool, str]:
    try:
        proc = subprocess.run(
            ["systemctl", "is-active", "balance-car-coordinator.service"],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        status = proc.stdout.strip()
        if status == "active":
            return True, "service is active"
        return False, f"service status: {status}"
    except FileNotFoundError:
        return False, "systemctl not available (not Linux/systemd?)"
    except subprocess.TimeoutExpired:
        return False, "systemctl timed out"


# ── Main ─────────────────────────────────────────────────────────────────────


def print_section(title: str):
    width = 60
    print(f"\n{BOLD}{'=' * width}{RESET}")
    print(f"{BOLD}  {title}{RESET}")
    print(f"{BOLD}{'=' * width}{RESET}")


def confirm_movement() -> bool:
    print(f"\n{YELLOW}{BOLD}⚠  Movement Safety Check{RESET}")
    answer = input("  Car is on blocks/stand and safe to move? [y/N]: ").strip().lower()
    return answer in ("y", "yes")


def main():
    parser = argparse.ArgumentParser(
        description="Balance Car Interface Verification Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              # Full check
              %(prog)s

              # Quick backend-only
              %(prog)s --quick

              # Specify custom coordinator address
              %(prog)s --coordinator-host 192.168.1.100 --coordinator-port 8765

              # Check serial directly
              %(prog)s --check-serial --serial-device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
        """),
    )
    parser.add_argument("--coordinator-host", default="127.0.0.1", help="Coordinator TCP host (default: 127.0.0.1)")
    parser.add_argument("--coordinator-port", type=int, default=8765, help="Coordinator TCP port (default: 8765)")
    parser.add_argument("--quick", action="store_true", help="Skip ROS 2 topic and serial checks")
    parser.add_argument("--check-backend", action="store_true", help="Only check coordinator backend")
    parser.add_argument("--check-ros-topics", action="store_true", help="Only check ROS 2 topics")
    parser.add_argument("--check-serial", action="store_true", help="Only check serial/STM32 interface")
    parser.add_argument("--serial-device", default=None, help="STM32 serial device path")
    parser.add_argument(
        "--bridge-script",
        default="/home/lifng0/workspace/balance_car/scripts/pi_serial_bridge.py",
        help="Path to pi_serial_bridge.py",
    )
    parser.add_argument("--ros-topic-timeout", type=float, default=3.0, help="ROS 2 topic echo timeout (default: 3s)")
    parser.add_argument("--skip-movement-tests", action="store_true", help="Skip movement-related backend tests")
    parser.add_argument("--no-color", action="store_true", help="Disable ANSI color output")

    args = parser.parse_args()

    if args.no_color:
        global GREEN, RED, YELLOW, CYAN, BOLD, RESET
        GREEN = RED = YELLOW = CYAN = BOLD = RESET = ""

    host = args.coordinator_host
    port = args.coordinator_port

    mode = "full"
    if args.quick:
        mode = "quick"
    if args.check_backend:
        mode = "backend"
    if args.check_ros_topics:
        mode = "ros"
    if args.check_serial:
        mode = "serial"

    passed = 0
    failed = 0
    skipped = 0

    def result(ok_flag: bool, msg: str):
        nonlocal passed, failed
        if ok_flag:
            passed += 1
            print(f"  {ok(msg)}")
        else:
            failed += 1
            print(f"  {fail(msg)}")

    def skip(msg: str):
        nonlocal skipped
        skipped += 1
        print(f"  {warn(f'SKIP: {msg}')}")

    print(f"\n{BOLD}Balance Car Interface Verification{RESET}")
    print(f"  Coordinator: {host}:{port}")
    print(f"  Mode: {mode}")
    print(f"  Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    # ── 1. Coordinator Backend ──────────────────────────────────────────────
    if mode in ("full", "quick", "backend"):
        print_section("1. Coordinator Backend (TCP)")

        # 1a. Service status
        if mode in ("full", "backend"):
            print("  Checking coordinator service...")
            ok_flag, msg = check_coordinator_service()
            result(ok_flag, f"balance-car-coordinator.service: {msg}")

        # 1b. TCP connection
        print(f"  Connecting to {host}:{port}...")
        ok_flag, msg = check_backend_connection(host, port)
        result(ok_flag, f"TCP connection: {msg}")

        if ok_flag:
            # 1c. get_state
            print("  Sending get_state...")
            ok_flag, msg = check_get_state(host, port)
            result(ok_flag, f"get_state response: {msg}")

            # 1d. lidar_summary via backend
            print("  Checking lidar summary in state...")
            ok_flag, msg = check_lidar_summary_via_backend(host, port)
            result(ok_flag, f"lidar_summary: {msg}")

            # 1e. get_vision
            print("  Sending get_vision...")
            ok_flag, msg = check_get_vision(host, port)
            result(ok_flag, f"get_vision response: {msg}")

            # 1f. stop (safe, always works)
            print("  Sending stop...")
            ok_flag, msg = check_backend_stop(host, port)
            result(ok_flag, f"stop: {msg}")

            # 1g. set_move(0, 0) (safe)
            print("  Sending set_move(0, 0)...")
            ok_flag, msg = check_backend_move_zero(host, port)
            result(ok_flag, f"set_move(0,0): {msg}")

            # 1h. set_enable (movement test - needs confirmation)
            if not args.skip_movement_tests:
                print("  Testing set_enable...")
                if confirm_movement():
                    ok_flag, msg = check_backend_enable(host, port, False)
                    result(ok_flag, f"set_enable(False): {msg}")
                    time.sleep(0.5)
                    ok_flag, msg = check_backend_enable(host, port, True)
                    result(ok_flag, f"set_enable(True): {msg}")
                else:
                    skip("set_enable test requires safety confirmation")
            else:
                skip("set_enable test (-skip-movement-tests)")

    # ── 2. ROS 2 Topics ────────────────────────────────────────────────────
    if mode in ("full", "ros"):
        print_section("2. ROS 2 Topics")

        results = check_ros2_topics(args.ros_topic_timeout)
        for topic, ok_flag, msg in results:
            result(ok_flag, f"{topic}: {msg}")

    # ── 3. Direct Serial (STM32) ────────────────────────────────────────────
    if mode in ("full", "serial"):
        print_section("3. Direct Serial (STM32 via pi_serial_bridge)")

        if not args.serial_device:
            skip("serial device not specified (use --serial-device)")
        else:
            print(f"  Device: {args.serial_device}")
            print(f"  Bridge: {args.bridge_script}")

            ok_flag, msg = check_serial_ping(args.serial_device, args.bridge_script)
            result(ok_flag, f"ping: {msg}")

            if ok_flag:
                ok_flag, msg = check_serial_status(args.serial_device, args.bridge_script)
                result(ok_flag, f"status: {msg}")

    # ── Summary ─────────────────────────────────────────────────────────────
    print_section("Summary")
    total = passed + failed + skipped
    print(f"  {GREEN}Passed:{RESET} {passed}")
    print(f"  {RED}Failed:{RESET} {failed}")
    print(f"  {YELLOW}Skipped:{RESET} {skipped}")
    print(f"  Total:  {total}")

    if failed:
        print(f"\n{RED}{BOLD}Some checks failed. Review the details above.{RESET}")
    else:
        print(f"\n{GREEN}{BOLD}All checks passed!{RESET}")

    print()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
