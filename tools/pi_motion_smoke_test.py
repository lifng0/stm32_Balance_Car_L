#!/usr/bin/env python3
"""Send a short, observable motion sequence through the Pi coordinator backend."""

import argparse
import json
import socket
import sys
import time


def backend_request(host: str, port: int, payload: dict, timeout: float) -> dict:
    message = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(message)
        sock.shutdown(socket.SHUT_WR)
        chunks = []
        while True:
            data = sock.recv(4096)
            if not data:
                break
            chunks.append(data)
    raw = b"".join(chunks).decode("utf-8").strip()
    if not raw:
        raise RuntimeError("empty backend response")
    return json.loads(raw)


def get_state(args: argparse.Namespace) -> dict:
    return backend_request(args.host, args.port, {"cmd": "get_state"}, args.timeout)


def set_move(args: argparse.Namespace, move_x: float, move_z: float, label: str) -> dict:
    response = backend_request(
        args.host,
        args.port,
        {
            "cmd": "set_move",
            "move_x": move_x,
            "move_z": move_z,
            "wait_ack": args.strict_ack,
            "source": "motion_smoke_test",
        },
        args.timeout,
    )
    ok = "OK" if response.get("ok") else "REJECT"
    car_state = response.get("car_state") or {}
    print(
        f"{ok:6s} {label:12s} request=({move_x:5.1f}, {move_z:6.1f}) "
        f"ack_move=({float(car_state.get('move_x', 0.0) or 0.0):5.1f}, "
        f"{float(car_state.get('move_z', 0.0) or 0.0):6.1f}) "
        f"mode={car_state.get('mode_name', car_state.get('mode'))} "
        f"stop={car_state.get('stop_flag')} "
        f"error={response.get('error', '')} "
        f"detail={response.get('error_name', response.get('message', ''))} "
        f"verified_timeout={response.get('verified_after_timeout', False)} "
        f"verified_retry={response.get('verified_after_retry', False)}"
    )
    return response


def backend_simple(args: argparse.Namespace, payload: dict) -> dict:
    return backend_request(args.host, args.port, payload, args.timeout)


def ready_reason(state: dict) -> str:
    car_state = state.get("car_state") or {}
    reasons = []
    if state.get("system_mode") != "running":
        reasons.append(f"system_mode={state.get('system_mode')}")
    if not state.get("ros_ready"):
        reasons.append(f"ros_ready=false:{state.get('ros_ready_reason')}")
    if bool(car_state.get("stop_flag", True)):
        reasons.append("stop_flag=1")
    if not reasons:
        return "ready"
    return ", ".join(reasons)


def wait_until_motion_ready(args: argparse.Namespace) -> None:
    deadline = time.time() + args.wait_ready_sec
    last_reason = ""
    while True:
        state = get_state(args)
        reason = ready_reason(state)
        if reason == "ready":
            car_state = state.get("car_state") or {}
            print(
                "READY "
                f"mode={car_state.get('mode_name', car_state.get('mode'))} "
                f"battery={float(car_state.get('battery', 0.0) or 0.0):.2f}V "
                f"angle={float(car_state.get('angle', 0.0) or 0.0):.1f}"
            )
            return
        if reason != last_reason:
            print(f"WAIT  {reason}")
            last_reason = reason
        if time.time() >= deadline:
            raise RuntimeError(f"motion not ready after {args.wait_ready_sec:.1f}s: {reason}")
        time.sleep(0.2)


def sleep_with_countdown(seconds: float) -> None:
    end = time.time() + seconds
    while time.time() < end:
        time.sleep(min(0.1, end - time.time()))


def run_sequence(args: argparse.Namespace) -> int:
    sequence = [
        ("stop", 0.0, 0.0, args.pause_sec),
        ("forward", args.linear_speed, 0.0, args.move_sec),
        ("stop", 0.0, 0.0, args.pause_sec),
        ("backward", -args.linear_speed, 0.0, args.move_sec),
        ("stop", 0.0, 0.0, args.pause_sec),
        ("turn_left", 0.0, -args.turn_speed, args.turn_sec),
        ("stop", 0.0, 0.0, args.pause_sec),
        ("turn_right", 0.0, args.turn_speed, args.turn_sec),
        ("final_stop", 0.0, 0.0, args.pause_sec),
    ]

    print("Motion smoke test will start in 2 seconds.")
    print("Expected order: forward -> backward -> left turn -> right turn, with stops between.")
    if args.strict_ack:
        print("Strict ACK mode: each command waits for STM32 ACK and immediate status readback.")
    else:
        print("Fast control mode: commands are sent fire-and-forget, matching the normal control path.")
    time.sleep(2.0)

    try:
        total_duration = sum(item[3] for item in sequence) + 6.0
        backend_simple(args, {"cmd": "begin_motion_test", "duration_sec": total_duration})
        for label, move_x, move_z, duration in sequence:
            end = time.time() + duration
            first = True
            while first or time.time() < end:
                response = set_move(args, move_x, move_z, label)
                if not response.get("ok"):
                    return 2
                first = False
                time.sleep(min(args.refresh_sec, max(0.0, end - time.time())))
    finally:
        try:
            set_move(args, 0.0, 0.0, "safety_stop")
        except Exception as exc:
            print(f"WARN  safety stop failed: {exc}", file=sys.stderr)
        try:
            backend_simple(args, {"cmd": "end_motion_test"})
        except Exception as exc:
            print(f"WARN  end motion test failed: {exc}", file=sys.stderr)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Balance car Pi-to-STM32 motion smoke test")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--wait-ready-sec", type=float, default=20.0)
    parser.add_argument("--linear-speed", type=float, default=15.0, help="Move_X command magnitude")
    parser.add_argument("--turn-speed", type=float, default=300.0, help="Move_Z command magnitude")
    parser.add_argument("--move-sec", type=float, default=0.8)
    parser.add_argument("--turn-sec", type=float, default=0.8)
    parser.add_argument("--pause-sec", type=float, default=0.6)
    parser.add_argument("--refresh-sec", type=float, default=0.10)
    parser.add_argument("--strict-ack", action="store_true", help="Wait for STM32 ACK and status after each command.")
    parser.add_argument("--no-wait-ready", action="store_true")
    parser.add_argument("--run", action="store_true", help="Actually send motion commands. Without this, only check readiness.")
    args = parser.parse_args()

    if not args.no_wait_ready:
        wait_until_motion_ready(args)
    if not args.run:
        print("CHECK_ONLY ready check passed. Re-run with --run to send the motion sequence.")
        return 0
    return run_sequence(args)


if __name__ == "__main__":
    sys.exit(main())
