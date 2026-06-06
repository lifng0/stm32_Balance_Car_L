#!/usr/bin/env python3
import argparse
import json
import socket
import sys


def request_backend(host: str, port: int, payload: dict, timeout: float) -> dict:
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


def main() -> int:
    parser = argparse.ArgumentParser(description="TCP backend client for balance car coordinator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--timeout", type=float, default=3.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ping")
    subparsers.add_parser("get-state")
    subparsers.add_parser("get-vision")
    subparsers.add_parser("get-k210-link")

    k210_text_parser = subparsers.add_parser("set-k210-text")
    k210_text_parser.add_argument("text")

    mode_parser = subparsers.add_parser("set-mode")
    mode_parser.add_argument("value", type=int)

    move_parser = subparsers.add_parser("set-move")
    move_parser.add_argument("move_x", type=float)
    move_parser.add_argument("move_z", type=float)

    args = parser.parse_args()

    payload = {"cmd": args.command.replace("-", "_")}
    if args.command == "set-mode":
        payload["cmd"] = "set_mode"
        payload["value"] = args.value
    elif args.command == "set-move":
        payload["cmd"] = "set_move"
        payload["move_x"] = args.move_x
        payload["move_z"] = args.move_z
    elif args.command == "get-state":
        payload["cmd"] = "get_state"
    elif args.command == "get-vision":
        payload["cmd"] = "get_vision"
    elif args.command == "get-k210-link":
        payload["cmd"] = "get_k210_link"
    elif args.command == "set-k210-text":
        payload["cmd"] = "set_k210_text"
        payload["text"] = args.text

    response = request_backend(args.host, args.port, payload, args.timeout)
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
